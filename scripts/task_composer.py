#!/usr/bin/env python
import rospy
from robot_enums import TaskResult
from behavior_factory import BehaviorFactory
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

class TaskNode:
    """任务节点基类"""
    def __init__(self, name):
        self.name = name
        self.children = []
        self._cancelled = False
        
    def add_child(self, child):
        self.children.append(child)
    
    def execute(self):
        raise NotImplementedError
    
    def cancel(self):
        """取消任务执行"""
        self._cancelled = True
        for child in self.children:
            child.cancel()

class SequenceNode(TaskNode):
    """顺序执行节点 - 所有子节点必须成功"""
    def execute(self):
        rospy.loginfo(f"执行序列: {self.name}")
        for child in self.children:
            if self._cancelled:
                return TaskResult.CANCELLED
            result = child.execute()
            if result != TaskResult.SUCCESS:
                rospy.logwarn(f"序列节点 {self.name} 中的 {child.name} 执行失败")
                return result
        return TaskResult.SUCCESS

class SelectorNode(TaskNode):
    """选择执行节点 - 任一子节点成功即可"""
    def execute(self):
        rospy.loginfo(f"执行选择: {self.name}")
        for child in self.children:
            if self._cancelled:
                return TaskResult.CANCELLED
            result = child.execute()
            if result == TaskResult.SUCCESS:
                return result
        return TaskResult.FAILURE

class ParallelNode(TaskNode):
    """并行执行节点 - 同时执行多个子节点"""
    def execute(self):
        rospy.loginfo(f"执行并行: {self.name}")
        import threading
        results = []
        threads = []
        
        def execute_child(child, results, index):
            results[index] = child.execute()
        
        results = [None] * len(self.children)
        for i, child in enumerate(self.children):
            thread = threading.Thread(target=execute_child, args=(child, results, i))
            threads.append(thread)
            thread.start()
        
        for thread in threads:
            thread.join()
        
        # 所有任务都必须成功
        return TaskResult.SUCCESS if all(r == TaskResult.SUCCESS for r in results) else TaskResult.FAILURE

class ActionNode(TaskNode):
    """原子行为执行节点"""
    def __init__(self, name, behavior_type, **params):
        super().__init__(name)
        self.behavior_type = behavior_type
        self.params = params
        self.behavior = None
        
    def execute(self):
        rospy.loginfo(f"执行原子行为: {self.name}")
        try:
            self.behavior = BehaviorFactory.get_behavior(self.behavior_type)
            
            # 处理位姿参数
            if 'x' in self.params and 'y' in self.params:
                pose = self.create_pose(
                    self.params.pop('x'),
                    self.params.pop('y'),
                    self.params.pop('theta', 0.0)
                )
                self.params['target_pose'] = pose
            
            return self.behavior.execute(**self.params)
        except Exception as e:
            rospy.logerr(f"行为执行错误: {e}")
            return TaskResult.FAILURE
    
    def cancel(self):
        if self.behavior:
            self.behavior.cancel()
    
    def create_pose(self, x, y, theta):
        """创建位姿消息"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        q = quaternion_from_euler(0, 0, theta)
        pose.pose.orientation = Quaternion(*q)
        
        return pose

class TaskComposer:
    """任务组合器"""
    
    # ...existing code...
    
    @staticmethod
    def create_comprehensive_task():
        """创建综合测试任务"""
        main_task = SequenceNode("综合测试任务")
        
        # 初始化
        main_task.add_child(ActionNode("系统初始化", "speak", text="机器人系统启动"))
        main_task.add_child(ActionNode("LED指示", "peripheral_control", device_type="led", state=True))
        
        # 巡逻任务
        patrol = SequenceNode("巡逻任务")
        patrol.add_child(ActionNode("开始巡逻", "speak", text="开始执行巡逻任务"))
        patrol.add_child(ActionNode("导航1", "navigate", x=2.0, y=1.0, theta=0.0))
        patrol.add_child(ActionNode("拍照1", "capture_image", save_path="/tmp/patrol_1.jpg"))
        patrol.add_child(ActionNode("导航2", "navigate", x=1.0, y=2.0, theta=1.57))
        
        main_task.add_child(patrol)
        
        # 检查任务（选择性）
        inspection = SelectorNode("检查任务")
        for i in range(3):
            check_point = SequenceNode(f"检查点{i+1}")
            check_point.add_child(ActionNode(f"导航检查{i+1}", "navigate", x=float(i), y=float(i), theta=0.0))
            check_point.add_child(ActionNode(f"检查拍照{i+1}", "capture_image"))
            inspection.add_child(check_point)
        
        main_task.add_child(inspection)
        
        # 完成
        main_task.add_child(ActionNode("任务完成", "speak", text="所有任务已完成"))
        main_task.add_child(ActionNode("LED关闭", "peripheral_control", device_type="led", state=False))
        
        return main_task