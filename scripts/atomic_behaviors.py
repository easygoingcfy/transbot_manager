#!/usr/bin/env python
import rospy
import actionlib
from abc import ABC, abstractmethod
from robot_enums import TaskResult
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image

class AtomicBehavior(ABC):
    """原子行为抽象基类"""
    def __init__(self, name):
        self.name = name
        self.status_pub = rospy.Publisher('/task_status', String, queue_size=1)
        
    @abstractmethod
    def execute(self, **kwargs):
        """执行行为，返回TaskResult"""
        pass
    
    @abstractmethod
    def cancel(self):
        """取消行为执行"""
        pass
    
    def publish_status(self, status):
        self.status_pub.publish(f"{self.name}: {status}")

class NavigateBehavior(AtomicBehavior):
    def __init__(self):
        super().__init__("Navigate")
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
    def execute(self, target_pose, timeout=30.0):
        self.publish_status("开始导航")
        goal = MoveBaseGoal()
        goal.target_pose = target_pose
        
        self.client.send_goal(goal)
        finished = self.client.wait_for_result(rospy.Duration(timeout))
        
        if finished and self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return TaskResult.SUCCESS
        else:
            self.client.cancel_goal()
            return TaskResult.FAILURE
    
    def cancel(self):
        self.client.cancel_goal()

class RotateBehavior(AtomicBehavior):
    def __init__(self):
        super().__init__("Rotate")
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
    def execute(self, angle_delta, angular_speed=0.5):
        self.publish_status(f"旋转 {angle_delta} 弧度")
        # 实现精确旋转逻辑
        twist = Twist()
        twist.angular.z = angular_speed if angle_delta > 0 else -angular_speed
        
        duration = abs(angle_delta / angular_speed)
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.cmd_pub.publish(twist)
            rate.sleep()
        
        self.cmd_pub.publish(Twist())  # 停止
        return TaskResult.SUCCESS
    
    def cancel(self):
        self.cmd_pub.publish(Twist())

class CaptureImageBehavior(AtomicBehavior):
    def __init__(self):
        super().__init__("CaptureImage")
        # 这里可以集成实际的相机Action
        
    def execute(self, image_topic="/camera/image_raw", save_path=None):
        self.publish_status("拍照中")
        try:
            # 实现图像捕获逻辑
            image_msg = rospy.wait_for_message(image_topic, Image, timeout=5.0)
            if save_path:
                # 保存图像到指定路径
                pass
            return TaskResult.SUCCESS
        except rospy.ROSException:
            return TaskResult.TIMEOUT
    
    def cancel(self):
        pass

class SpeakBehavior(AtomicBehavior):
    def __init__(self):
        super().__init__("Speak")
        # 集成TTS或音频播放
        
    def execute(self, text, volume=0.8):
        self.publish_status(f"播放语音: {text}")
        # 实现语音播放逻辑
        rospy.sleep(2.0)  # 模拟播放时间
        return TaskResult.SUCCESS
    
    def cancel(self):
        pass

class WaitBehavior(AtomicBehavior):
    def __init__(self):
        super().__init__("Wait")
        self._cancelled = False
        
    def execute(self, duration):
        self.publish_status(f"等待 {duration} 秒")
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