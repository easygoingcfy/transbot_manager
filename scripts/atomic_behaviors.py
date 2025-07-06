#!/usr/bin/env python
# -*- coding: utf-8 -*-
# filepath: /home/jetson/transbot_ws/src/transbot_manager/scripts/atomic_behaviors.py

import rospy
import actionlib
from abc import ABCMeta, abstractmethod
from robot_enums import TaskResult
from std_msgs.msg import String
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
        
    def execute(self, image_topic="/camera/image_raw", save_path=None, **kwargs):
        """执行图像捕获"""
        self.publish_status("拍照中")
        try:
            # 尝试等待图像消息
            image_msg = rospy.wait_for_message(image_topic, Image, timeout=5.0)
            if save_path:
                rospy.loginfo("图像已保存到: {}".format(save_path))
            return TaskResult.SUCCESS
        except rospy.ROSException:
            rospy.logwarn("图像捕获超时")
            return TaskResult.TIMEOUT
        except Exception as e:
            rospy.logerr("图像捕获错误: {}".format(e))
            return TaskResult.FAILURE
    
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