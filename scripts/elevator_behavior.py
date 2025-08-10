#!/usr/bin/env python
#coding=utf-8

import rospy
from atomic_behaviors import AtomicBehavior
from robot_enums import TaskResult
from std_msgs.msg import Int32

class ElevatorControlBehavior(AtomicBehavior):
    """升降杆控制行为（离散命令：上/下/停）"""
    
    def __init__(self):
        super().__init__("ElevatorControl")
        # 升降杆命令发布器：1=上升,2=下降,0=停止
        self.elevator_cmd_pub = rospy.Publisher('/elevator/command', Int32, queue_size=1)
    
    def execute(self, action, timeout=10.0):
        """执行升降杆动作
        :param action: 'up'|'down'|'stop'
        :param timeout: 超时时间
        """
        mapping = {'up': 1, 'down': 2, 'stop': 0}
        if action not in mapping:
            self.publish_status("无效的升降杆动作: {}".format(action))
            return TaskResult.FAILURE
        
        cmd = mapping[action]
        self.publish_status("升降杆动作: {}".format(action))
        self.elevator_cmd_pub.publish(Int32(cmd))
        
        # 简单等待确认时间（继电器无位置反馈，直接返回成功）
        rospy.sleep(min(1.0, timeout))
        return TaskResult.SUCCESS
    
    def cancel(self):
        super().cancel()
        # 取消时发送停止
        self.elevator_cmd_pub.publish(Int32(0))