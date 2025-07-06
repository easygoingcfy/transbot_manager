#!/usr/bin/env python
# coding=utf-8
import rospy
import json
import time
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist, PoseStamped
from diagnostic_msgs.msg import DiagnosticStatus

class SystemTester:
    """系统测试工具"""
    
    def __init__(self):
        rospy.init_node('system_tester')
        
        # 测试发布器
        self.mode_pub = rospy.Publisher('/remote_control/mode', String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/remote_control/cmd_vel', Twist, queue_size=1)
        self.goal_pub = rospy.Publisher('/remote_control/goal', PoseStamped, queue_size=1)
        self.elevator_pub = rospy.Publisher('/remote_control/elevator', Float32, queue_size=1)
        self.camera_pub = rospy.Publisher('/remote_control/camera', Bool, queue_size=1)
        self.emergency_pub = rospy.Publisher('/remote_control/emergency_stop', Bool, queue_size=1)
        self.task_pub = rospy.Publisher('/cloud_tasks', String, queue_size=1)
        
        # 状态订阅器
        rospy.Subscriber('/robot_status', DiagnosticStatus, self._status_callback)
        rospy.Subscriber('/control_feedback', String, self._feedback_callback)
        rospy.Subscriber('/task_execution_status', String, self._task_status_callback)
        
        # 测试状态
        self.system_ready = False
        self.last_status = None
        self.test_results = {}
        
        rospy.loginfo("系统测试器已启动")
        
        # 等待系统启动
        rospy.sleep(3.0)
        self.run_comprehensive_test()
    
    def _status_callback(self, msg):
        """系统状态回调"""
        self.last_status = msg
        if msg.level == DiagnosticStatus.OK:
            self.system_ready = True
    
    def _feedback_callback(self, msg):
        """控制反馈回调"""
        try:
            feedback = json.loads(msg.data)
            rospy.loginfo(f"控制反馈: {feedback['message']}")
        except:
            rospy.logwarn("控制反馈格式错误")
    
    def _task_status_callback(self, msg):
        """任务状态回调"""
        try:
            status = json.loads(msg.data)
            rospy.loginfo(f"任务状态: {status}")
        except:
            rospy.logwarn("任务状态格式错误")
    
    def run_comprehensive_test(self):
        """运行综合测试"""
        rospy.loginfo("开始系统综合测试...")
        
        # 等待系统就绪
        self.wait_for_system_ready()
        
        # 测试序列
        tests = [
            ("模式切换测试", self.test_mode_switching),
            ("手动控制测试", self.test_manual_control),
            ("升降杆测试", self.test_elevator_control),
            ("相机测试", self.test_camera_control),
            ("任务执行测试", self.test_task_execution),
            ("紧急停止测试", self.test_emergency_stop),
        ]
        
        for test_name, test_func in tests:
            rospy.loginfo(f"执行测试: {test_name}")
            try:
                result = test_func()
                self.test_results[test_name] = result
                rospy.loginfo(f"测试 {test_name} {'通过' if result else '失败'}")
            except Exception as e:
                rospy.logerr(f"测试 {test_name} 异常: {e}")
                self.test_results[test_name] = False
            
            rospy.sleep(2.0)  # 测试间隔
        
        self.print_test_summary()
    
    def wait_for_system_ready(self, timeout=10.0):
        """等待系统就绪"""
        rospy.loginfo("等待系统就绪...")
        start_time = time.time()
        
        while not rospy.is_shutdown():
            if self.system_ready:
                rospy.loginfo("系统已就绪")
                return True
            
            if time.time() - start_time > timeout:
                rospy.logwarn("系统就绪超时")
                return False
            
            rospy.sleep(0.5)
        
        return False
    
    def test_mode_switching(self):
        """测试模式切换"""
        try:
            modes = ['manual', 'auto', 'pause', 'stop']
            
            for mode in modes:
                self.mode_pub.publish(String(mode))
                rospy.sleep(1.0)
                
                if self.last_status and mode in self.last_status.message.lower():
                    rospy.loginfo(f"模式切换到 {mode} 成功")
                else:
                    return False
            
            return True
        except Exception as e:
            rospy.logerr(f"模式切换测试错误: {e}")
            return False
    
    def test_manual_control(self):
        """测试手动控制"""
        try:
            # 切换到手动模式
            self.mode_pub.publish(String('manual'))
            rospy.sleep(1.0)
            
            # 发送速度指令
            cmd = Twist()
            cmd.linear.x = 0.1
            self.cmd_vel_pub.publish(cmd)
            rospy.sleep(2.0)
            
            # 停止
            cmd.linear.x = 0.0
            self.cmd_vel_pub.publish(cmd)
            rospy.sleep(1.0)
            
            return True
        except Exception as e:
            rospy.logerr(f"手动控制测试错误: {e}")
            return False
    
    def test_elevator_control(self):
        """测试升降杆控制"""
        try:
            # 测试不同位置
            positions = [0.2, 0.5, 0.8, 0.0]
            
            for pos in positions:
                self.elevator_pub.publish(Float32(pos))
                rospy.sleep(3.0)  # 等待移动完成
            
            return True
        except Exception as e:
            rospy.logerr(f"升降杆测试错误: {e}")
            return False
    
    def test_camera_control(self):
        """测试相机控制"""
        try:
            # 触发拍照
            self.camera_pub.publish(Bool(True))
            rospy.sleep(2.0)
            
            return True
        except Exception as e:
            rospy.logerr(f"相机测试错误: {e}")
            return False
    
    def test_task_execution(self):
        """测试任务执行"""
        try:
            # 创建简单测试任务
            test_task = {
                "task_id": "test_001",
                "name": "系统测试任务",
                "type": "sequence",
                "children": [
                    {
                        "type": "action",
                        "name": "等待测试",
                        "behavior_type": "wait",
                        "params": {"duration": 2.0}
                    },
                    {
                        "type": "action",
                        "name": "语音测试",
                        "behavior_type": "speak",
                        "params": {"text": "系统测试中"}
                    }
                ]
            }
            
            # 发送任务
            self.task_pub.publish(json.dumps(test_task, ensure_ascii=False))
            rospy.sleep(10.0)  # 等待任务完成
            
            return True
        except Exception as e:
            rospy.logerr(f"任务执行测试错误: {e}")
            return False
    
    def test_emergency_stop(self):
        """测试紧急停止"""
        try:
            # 激活紧急停止
            self.emergency_pub.publish(Bool(True))
            rospy.sleep(2.0)
            
            # 解除紧急停止
            self.emergency_pub.publish(Bool(False))
            rospy.sleep(2.0)
            
            return True
        except Exception as e:
            rospy.logerr(f"紧急停止测试错误: {e}")
            return False
    
    def print_test_summary(self):
        """打印测试总结"""
        rospy.loginfo("=== 系统测试总结 ===")
        
        total_tests = len(self.test_results)
        passed_tests = sum(1 for result in self.test_results.values() if result)
        
        for test_name, result in self.test_results.items():
            status = "通过" if result else "失败"
            rospy.loginfo(f"{test_name}: {status}")
        
        rospy.loginfo(f"总计: {passed_tests}/{total_tests} 测试通过")
        
        if passed_tests == total_tests:
            rospy.loginfo("🎉 所有测试通过！系统运行正常")
        else:
            rospy.logwarn("⚠️ 部分测试失败，请检查系统配置")

if __name__ == '__main__':
    try:
        tester = SystemTester()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass