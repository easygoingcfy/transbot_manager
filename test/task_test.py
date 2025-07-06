#!/usr/bin/env python
# -*- coding: utf-8 -*-
# filepath: /home/jetson/transbot_ws/src/transbot_manager/test/task_test.py

import rospy
import json
import sys
import os
from std_msgs.msg import String

# Python 2编码修复
if sys.version_info[0] == 2:
    reload(sys)
    sys.setdefaultencoding('utf-8')

# 添加scripts目录到Python路径
script_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'scripts')
sys.path.insert(0, script_path)

# 现在可以正常导入
from task_parser import TaskParser

class TaskTester:
    """任务测试工具"""
    
    def __init__(self):
        rospy.init_node('task_tester')
        
        # 任务发布器
        self.task_pub = rospy.Publisher('/cloud_tasks', String, queue_size=1)
        
        # 状态订阅器
        rospy.Subscriber('/task_execution_status', String, self.status_callback)
        rospy.Subscriber('/task_feedback', String, self.feedback_callback)
        
        rospy.sleep(1.0)  # 等待发布器准备
        rospy.loginfo("任务测试器已启动")
    
    def safe_json_dumps(self, data):
        """安全的JSON序列化"""
        try:
            json_str = json.dumps(data, ensure_ascii=False, indent=2)
            if sys.version_info[0] == 2 and isinstance(json_str, unicode):
                json_str = json_str.encode('utf-8')
            return json_str
        except Exception as e:
            rospy.logerr("JSON序列化失败: {}".format(repr(e)))
            return "{}"
    
    def status_callback(self, msg):
        """任务状态回调"""
        try:
            status = json.loads(msg.data)
            rospy.loginfo("[状态] {}: {} - {:.1f}%".format(
                status['name'], status['status'], status['progress']))
        except:
            rospy.loginfo("[状态] {}".format(msg.data))
    
    def feedback_callback(self, msg):
        """任务反馈回调"""
        try:
            feedback = json.loads(msg.data)
            rospy.loginfo("[反馈] {}".format(feedback['message']))
        except:
            rospy.loginfo("[反馈] {}".format(msg.data))
    
    def test_simple_task(self):
        """测试简单任务 - 使用ASCII字符避免编码问题"""
        task = {
            "task_id": "test_simple",
            "type": "sequence", 
            "name": "Simple Test Task",  # 使用英文避免编码问题
            "children": [
                {
                    "type": "action",
                    "name": "start",
                    "behavior": "speak",
                    "params": {"text": "Starting simple test"}
                },
                {
                    "type": "action",
                    "name": "Navigate to Point 1",
                    "behavior": "navigate",
                    "params": {"x": 0.2, "y": 0.2, "theta": 0.0}
                },
                {
                    "type": "action",
                    "name": "wait",
                    "behavior": "wait",
                    "params": {"duration": 3.0}
                },
                {
                    "type": "action",
                    "name": "end",
                    "behavior": "speak",
                    "params": {"text": "Simple test completed"}
                }
            ]
        }
        
        rospy.loginfo("发送简单测试任务...")
        json_str = self.safe_json_dumps(task)
        self.task_pub.publish(String(data=json_str))
    
    def test_navigation_task(self):
        """测试导航任务"""
        task = {
            "task_id": "test_navigation",
            "type": "sequence",
            "name": "Navigation Test",
            "children": [
                {
                    "type": "action",
                    "name": "start_nav",
                    "behavior": "speak",
                    "params": {"text": "Starting navigation"}
                },
                {
                    "type": "action",
                    "name": "navigate",
                    "behavior": "navigate",
                    "params": {"x": 0.2, "y": 0.2, "theta": 0.0}
                },
                {
                    "type": "action",
                    "name": "nav_complete",
                    "behavior": "speak",
                    "params": {"text": "Navigation completed"}
                }
            ]
        }
        
        rospy.loginfo("发送导航测试任务...")
        json_str = self.safe_json_dumps(task)
        self.task_pub.publish(String(data=json_str))
    
    def test_complex_task(self):
        """测试复杂任务"""
        task = {
            "task_id": "test_complex",
            "type": "sequence",
            "name": "Complex Patrol Task",
            "children": [
                {
                    "type": "action",
                    "name": "start_patrol",
                    "behavior": "speak",
                    "params": {"text": "Starting patrol task"}
                },
                {
                    "type": "sequence",
                    "name": "patrol_sequence",
                    "children": [
                        {
                            "type": "action",
                            "name": "go_to_point1",
                            "behavior": "navigate",
                            "params": {"x": 0.2, "y": 0.2, "theta": 30.0}
                        },
                        {
                            "type": "action",
                            "name": "wait_check",
                            "behavior": "wait",
                            "params": {"duration": 2.0}
                        },
                        {
                            "type": "action",
                            "name": "capture",
                            "behavior": "capture_image",
                            "params": {}
                        },
                        {
                            "type": "action",
                            "name": "return_home",
                            "behavior": "navigate",
                            "params": {"x": 0.0, "y": 0.0, "theta": 0.0}
                        }
                    ]
                },
                {
                    "type": "action",
                    "name": "patrol_complete",
                    "behavior": "speak",
                    "params": {"text": "Patrol task completed"}
                }
            ]
        }
        
        rospy.loginfo("发送复杂测试任务...")
        json_str = self.safe_json_dumps(task)
        self.task_pub.publish(String(data=json_str))
    
    def run_tests(self, test_type="all"):
        """运行测试"""
        if test_type == "simple" or test_type == "all":
            self.test_simple_task()
            rospy.sleep(8.0)
        
        if test_type == "navigation" or test_type == "all":
            self.test_navigation_task()
            rospy.sleep(15.0)
        
        if test_type == "complex" or test_type == "all":
            self.test_complex_task()
            rospy.sleep(30.0)

if __name__ == '__main__':
    try:
        tester = TaskTester()
        
        test_type = "simple"
        if len(sys.argv) > 1:
            test_type = sys.argv[1]
        
        rospy.loginfo("开始执行 {} 测试".format(test_type))
        tester.run_tests(test_type)
        
        rospy.loginfo("测试完成，等待结果...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("测试被中断")