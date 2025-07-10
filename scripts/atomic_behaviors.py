#!/usr/bin/env python
# -*- coding: utf-8 -*-
# filepath: /home/jetson/transbot_ws/src/transbot_manager/scripts/atomic_behaviors.py

import rospy
import actionlib
from abc import ABCMeta, abstractmethod
from robot_enums import TaskResult
import cv2
import numpy as np
from cv_bridge import CvBridge
import os
from datetime import datetime
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image

class AtomicBehavior(object):
    """原子行为抽象基类"""
    __metaclass__ = ABCMeta
    
    def __init__(self, name):
        self.name = name
        self.status_pub = rospy.Publisher('/task_status', String, queue_size=1)
        self._cancelled = False
        
    @abstractmethod
    def execute(self, **kwargs):
        """执行行为，返回TaskResult"""
        pass
    
    @abstractmethod
    def cancel(self):
        """取消行为执行"""
        pass
    
    def publish_status(self, status):
        self.status_pub.publish("{}: {}".format(self.name, status))

class NavigateBehavior(AtomicBehavior):
    def __init__(self):
        super(NavigateBehavior, self).__init__("Navigate")
        self.client = None
        try:
            self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            # 非阻塞等待，避免启动卡住
            if not self.client.wait_for_server(rospy.Duration(2.0)):
                rospy.logwarn("move_base服务器未找到，导航行为将模拟执行")
                self.client = None
        except Exception as e:
            rospy.logwarn("导航客户端初始化失败: {}".format(e))
            self.client = None
        
    def execute(self, target_pose=None, timeout=30.0, **kwargs):
        """执行导航"""
        self.publish_status("开始导航")
        
        if not self.client:
            rospy.loginfo("模拟导航执行")
            rospy.sleep(3.0)  # 模拟导航时间
            return TaskResult.SUCCESS
        
        try:
            goal = MoveBaseGoal()
            goal.target_pose = target_pose
            
            self.client.send_goal(goal)
            finished = self.client.wait_for_result(rospy.Duration(timeout))
            
            if finished and self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                return TaskResult.SUCCESS
            else:
                self.client.cancel_goal()
                return TaskResult.FAILURE
        except Exception as e:
            rospy.logerr("导航执行错误: {}".format(e))
            return TaskResult.FAILURE
    
    def cancel(self):
        self._cancelled = True
        if self.client:
            self.client.cancel_goal()

class RotateBehavior(AtomicBehavior):
    def __init__(self):
        super(RotateBehavior, self).__init__("Rotate")
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
    def execute(self, angle_delta=0.0, angular_speed=0.5, **kwargs):
        """执行旋转"""
        self.publish_status("旋转 {} 弧度".format(angle_delta))
        
        if angle_delta == 0.0:
            return TaskResult.SUCCESS
        
        twist = Twist()
        twist.angular.z = angular_speed if angle_delta > 0 else -angular_speed
        
        duration = abs(angle_delta / angular_speed)
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < duration:
            if self._cancelled:
                break
            self.cmd_pub.publish(twist)
            rate.sleep()
        
        self.cmd_pub.publish(Twist())  # 停止
        return TaskResult.SUCCESS if not self._cancelled else TaskResult.CANCELLED
    
    def cancel(self):
        self._cancelled = True
        self.cmd_pub.publish(Twist())

class CaptureImageBehavior(AtomicBehavior):
    def __init__(self):
        super(CaptureImageBehavior, self).__init__("CaptureImage")
        self.bridge = CvBridge()
        
    def execute(self, camera_type="rgb", save_path=None, timeout=5.0, **kwargs):
        """执行图像捕获
        Args:
            camera_type: "rgb", "depth", "ir" 选择相机类型
            save_path: 保存路径，如果为None则自动生成
            timeout: 等待图像的超时时间
        """
        # 根据相机类型选择话题
        topic_map = {
            "rgb": "/camera/rgb/image_raw",
            "depth": "/camera/depth/image_raw", 
            "ir": "/camera/ir/image_raw"
        }
        
        image_topic = topic_map.get(camera_type, "/camera/rgb/image_raw")
        
        self.publish_status("拍照中 - {}".format(camera_type.upper()))
        
        try:
            # 直接等待图像消息（无需触发）
            image_msg = rospy.wait_for_message(image_topic, Image, timeout=timeout)
            
            # 转换图像格式
            if camera_type == "depth":
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, "16UC1")
            elif camera_type == "ir":
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, "mono8")
            else:  # rgb
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # 生成保存路径
            if not save_path:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_dir = "/tmp/robot_captures"
                if not os.path.exists(save_dir):
                    os.makedirs(save_dir)
                save_path = os.path.join(save_dir, "{}_{}.jpg".format(camera_type, timestamp))
            
            # 保存图像
            success = self._save_image(cv_image, save_path, camera_type)
            if success:
                self.publish_status("图像已保存: {}".format(save_path))
                rospy.loginfo("图像已保存到: {}".format(save_path))
                return TaskResult.SUCCESS
            else:
                rospy.logwarn("图像保存失败: {}".format(save_path))
                return TaskResult.FAILURE
            
        except rospy.ROSException:
            self.publish_status("拍照超时")
            rospy.logwarn("图像捕获超时 - 话题: {}".format(image_topic))
            return TaskResult.TIMEOUT
        except Exception as e:
            self.publish_status("拍照失败")
            rospy.logerr("图像捕获错误: {}".format(e))
            return TaskResult.FAILURE
    
    def _save_image(self, cv_image, save_path, camera_type):
        """保存图像到指定路径"""
        try:
            # 确保目录存在
            directory = os.path.dirname(save_path)
            if directory and not os.path.exists(directory):
                os.makedirs(directory)
            
            # 根据相机类型进行不同处理
            if camera_type == "depth":
                # 深度图像需要特殊处理
                depth_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                return cv2.imwrite(save_path, depth_colored)
            else:
                # RGB和红外图像直接保存
                return cv2.imwrite(save_path, cv_image)
                
        except Exception as e:
            rospy.logerr("保存图像失败: {}".format(e))
            return False
    
    def capture_all_cameras(self, base_path="/tmp/robot_captures", timeout=5.0):
        """捕获所有相机的图像"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        results = {}
        
        for camera_type in ["rgb", "depth", "ir"]:
            save_path = os.path.join(base_path, "{}_{}.jpg".format(camera_type, timestamp))
            result = self.execute(camera_type=camera_type, save_path=save_path, timeout=timeout)
            results[camera_type] = {
                'result': result,
                'path': save_path if result == TaskResult.SUCCESS else None
            }
        
        return results
    
    def cancel(self):
        self._cancelled = True


class SpeakBehavior(AtomicBehavior):
    def __init__(self):
        super(SpeakBehavior, self).__init__("Speak")
        
    def execute(self, text="", volume=0.8, **kwargs):
        """执行语音播放"""
        self.publish_status("播放语音: {}".format(text))
        # 模拟语音播放时间
        duration = max(1.0, len(text) * 0.1)  # 根据文本长度估算播放时间
        rospy.sleep(duration)
        return TaskResult.SUCCESS
    
    def cancel(self):
        self._cancelled = True

class WaitBehavior(AtomicBehavior):
    def __init__(self):
        super(WaitBehavior, self).__init__("Wait")
        
    def execute(self, duration=1.0, **kwargs):
        """执行等待"""
        self.publish_status("等待 {} 秒".format(duration))
        self._cancelled = False
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < duration:
            if self._cancelled:
                return TaskResult.CANCELLED
            rate.sleep()
        
        return TaskResult.SUCCESS
    
    def cancel(self):
        self._cancelled = True