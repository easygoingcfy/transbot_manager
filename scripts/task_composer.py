#!/usr/bin/env python
# -*- coding: utf-8 -*-
# filepath: /home/jetson/transbot_ws/src/transbot_manager/scripts/task_composer.py

import rospy
import math
from robot_enums import TaskResult
from behavior_factory import BehaviorFactory
from geometry_msgs.msg import PoseStamped, Quaternion

# 简单的四元数转换函数，避免tf依赖
def quaternion_from_euler(roll, pitch, yaw):
    """欧拉角转四元数"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [x, y, z, w]

class TaskNode(object):
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
        rospy.loginfo("执行序列: {}".format(self.name))
        for child in self.children:
            if self._cancelled:
                return TaskResult.CANCELLED
            result = child.execute()
            if result != TaskResult.SUCCESS:
                # 修复字符串格式化
                rospy.logwarn("序列节点 {} 中的 {} 执行失败".format(self.name, child.name))
                return result
        return TaskResult.SUCCESS

class SelectorNode(TaskNode):
    """选择执行节点 - 任一子节点成功即可"""
    def execute(self):
        rospy.loginfo("执行选择: {}".format(self.name))
        for child in self.children:
            if self._cancelled:
                return TaskResult.CANCELLED
            result = child.execute()
            if result == TaskResult.SUCCESS:
                return result
        return TaskResult.FAILURE

class ParallelNode(TaskNode):
    """并行执行节点 - 简化为顺序执行（避免复杂性）"""
    def execute(self):
        rospy.loginfo("执行并行（简化为顺序）: {}".format(self.name))
        results = []
        for child in self.children:
            if self._cancelled:
                return TaskResult.CANCELLED
            result = child.execute()
            results.append(result)
        
        # 所有任务都必须成功
        return TaskResult.SUCCESS if all(r == TaskResult.SUCCESS for r in results) else TaskResult.FAILURE

class ActionNode(TaskNode):
    """原子行为执行节点"""
    def __init__(self, name, behavior_type, **params):
        super(ActionNode, self).__init__(name)
        self.behavior_type = behavior_type
        self.params = params
        self.behavior = None
        
    def execute(self):
        rospy.loginfo("执行原子行为: {}".format(self.name))
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
            rospy.logerr("行为执行错误: {}".format(e))
            return TaskResult.FAILURE
    
    def cancel(self):
        if self.behavior:
            self.behavior.cancel()
        super(ActionNode, self).cancel()
    
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

class TaskComposer(object):
    """任务组合器"""
    
    @staticmethod
    def create_simple_test():
        """创建简单测试任务"""
        main_task = SequenceNode("简单测试")
        main_task.add_child(ActionNode("开始", "speak", text="开始测试"))
        main_task.add_child(ActionNode("等待", "wait", duration=2.0))
        main_task.add_child(ActionNode("结束", "speak", text="测试完成"))
        return main_task