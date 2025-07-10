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

    def test_patrol_with_camera(self):
        """测试巡逻+拍照任务"""
        task = {
            "task_id": "test_patrol_camera",
            "type": "sequence",
            "name": "Patrol with Camera Task",
            "children": [
                {
                    "type": "action",
                    "name": "start_patrol",
                    "behavior": "speak",
                    "params": {"text": "Starting patrol with camera"}
                },
                {
                    "type": "sequence",
                    "name": "patrol_point_1",
                    "children": [
                        {
                            "type": "action",
                            "name": "go_to_point1",
                            "behavior": "navigate",
                            "params": {"x": 0.1, "y": 0.1, "theta": 0.0}
                        },
                        {
                            "type": "action",
                            "name": "arrive_point1",
                            "behavior": "speak",
                            "params": {"text": "Arrived at point 1"}
                        },
                        {
                            "type": "action",
                            "name": "capture_at_point1",
                            "behavior": "capture_image",
                            "params": {"camera_type": "rgb", "timeout": 5.0}
                        },
                        {
                            "type": "action",
                            "name": "rotate_for_depth",
                            "behavior": "rotate",
                            "params": {"angle_delta": 1.57, "angular_speed": 0.5}
                        },
                        {
                            "type": "action",
                            "name": "capture_depth_at_point1",
                            "behavior": "capture_image",
                            "params": {"camera_type": "depth", "timeout": 5.0}
                        }
                    ]
                },
                {
                    "type": "sequence",
                    "name": "patrol_point_2", 
                    "children": [
                        {
                            "type": "action",
                            "name": "go_to_point2",
                            "behavior": "navigate",
                            "params": {"x": -0.1, "y": 0.1, "theta": 1.57}
                        },
                        {
                            "type": "action",
                            "name": "arrive_point2",
                            "behavior": "speak",
                            "params": {"text": "Arrived at point 2"}
                        },
                        {
                            "type": "action",
                            "name": "capture_at_point2",
                            "behavior": "capture_image",
                            "params": {"camera_type": "rgb", "timeout": 5.0}
                        }
                    ]
                },
                {
                    "type": "action",
                    "name": "return_home",
                    "behavior": "navigate", 
                    "params": {"x": 0.0, "y": 0.0, "theta": 0.0}
                },
                {
                    "type": "action",
                    "name": "patrol_complete",
                    "behavior": "speak",
                    "params": {"text": "Patrol with camera completed"}
                }
            ]
        }
        
        rospy.loginfo("发送巡逻+拍照测试任务...")
        json_str = self.safe_json_dumps(task)
        self.task_pub.publish(String(data=json_str))

    def test_camera_task(self):
        """测试拍照任务"""
        task = {
            "task_id": "test_camera",
            "type": "sequence",
            "name": "Camera Test Task",
            "children": [
                {
                    "type": "action",
                    "name": "start_camera_test",
                    "behavior": "speak",
                    "params": {"text": "Starting camera test"}
                },
                {
                    "type": "action",
                    "name": "capture_rgb",
                    "behavior": "capture_image",
                    "params": {"camera_type": "rgb", "timeout": 5.0}
                },
                {
                    "type": "action",
                    "name": "wait_between_captures",
                    "behavior": "wait",
                    "params": {"duration": 2.0}
                },
                {
                    "type": "action",
                    "name": "capture_depth",
                    "behavior": "capture_image", 
                    "params": {"camera_type": "depth", "timeout": 5.0}
                },
                {
                    "type": "action",
                    "name": "wait_again",
                    "behavior": "wait",
                    "params": {"duration": 2.0}
                },
                {
                    "type": "action",
                    "name": "capture_ir",
                    "behavior": "capture_image",
                    "params": {"camera_type": "ir", "timeout": 5.0}
                },
                {
                    "type": "action",
                    "name": "camera_test_complete",
                    "behavior": "speak",
                    "params": {"text": "Camera test completed"}
                }
            ]
        }
        
        rospy.loginfo("发送拍照测试任务...")
        json_str = self.safe_json_dumps(task)
        self.task_pub.publish(String(data=json_str))

    def test_camera_survey_task(self):
        """测试相机全方位拍摄任务"""
        task = {
            "task_id": "test_camera_survey",
            "type": "sequence",
            "name": "Camera Survey Task",
            "children": [
                {
                    "type": "action",
                    "name": "start_survey",
                    "behavior": "speak",
                    "params": {"text": "Starting camera survey"}
                },
                {
                    "type": "sequence",
                    "name": "360_degree_capture",
                    "children": [
                        {
                            "type": "action",
                            "name": "capture_0_degree",
                            "behavior": "capture_image",
                            "params": {"camera_type": "rgb", "timeout": 5.0}
                        },
                        {
                            "type": "action",
                            "name": "rotate_90",
                            "behavior": "rotate",
                            "params": {"angle_delta": 1.57, "angular_speed": 0.3}
                        },
                        {
                            "type": "action",
                            "name": "capture_90_degree", 
                            "behavior": "capture_image",
                            "params": {"camera_type": "rgb", "timeout": 5.0}
                        },
                        {
                            "type": "action",
                            "name": "rotate_180",
                            "behavior": "rotate", 
                            "params": {"angle_delta": 1.57, "angular_speed": 0.3}
                        },
                        {
                            "type": "action",
                            "name": "capture_180_degree",
                            "behavior": "capture_image",
                            "params": {"camera_type": "rgb", "timeout": 5.0}
                        },
                        {
                            "type": "action",
                            "name": "rotate_270",
                            "behavior": "rotate",
                            "params": {"angle_delta": 1.57, "angular_speed": 0.3}
                        },
                        {
                            "type": "action",
                            "name": "capture_270_degree",
                            "behavior": "capture_image", 
                            "params": {"camera_type": "rgb", "timeout": 5.0}
                        },
                        {
                            "type": "action",
                            "name": "rotate_back_to_start",
                            "behavior": "rotate",
                            "params": {"angle_delta": 1.57, "angular_speed": 0.3}
                        }
                    ]
                },
                {
                    "type": "action",
                    "name": "capture_final_depth",
                    "behavior": "capture_image",
                    "params": {"camera_type": "depth", "timeout": 5.0}
                },
                {
                    "type": "action",
                    "name": "survey_complete",
                    "behavior": "speak",
                    "params": {"text": "Camera survey completed"}
                }
            ]
        }
        
        rospy.loginfo("发送相机全方位拍摄任务...")
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
        
        if test_type == "camera" or test_type == "all":
            self.test_camera_task()
            rospy.sleep(20.0)
        
        if test_type == "patrol_camera" or test_type == "all":
            self.test_patrol_with_camera()
            rospy.sleep(45.0)
        
        if test_type == "camera_survey" or test_type == "all":
            self.test_camera_survey_task()
            rospy.sleep(60.0)
        
        if test_type == "complex" or test_type == "all":
            self.test_complex_task()
            rospy.sleep(30.0)

if __name__ == '__main__':
    try:
        tester = TaskTester()
        
        test_type = "patrol_camera"
        if len(sys.argv) > 1:
            test_type = sys.argv[1]
        
        # 打印可用的测试类型
        available_tests = ["simple", "navigation", "camera", "patrol_camera", "camera_survey", "complex", "all"]
        rospy.loginfo("可用测试类型: {}".format(", ".join(available_tests)))
        rospy.loginfo("开始执行 {} 测试".format(test_type))
        
        tester.run_tests(test_type)
        
        rospy.loginfo("测试完成，等待结果...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("测试被中断")