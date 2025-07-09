#!/usr/bin/env python
# -*- coding: utf-8 -*-
# filepath: /home/jetson/transbot_ws/src/transbot_manager/scripts/system_coordinator.py

import rospy
import json
from std_msgs.msg import String, Bool

class SystemCoordinator:
    """系统协调器 - 协调任务层和控制层"""
    
    def __init__(self):
        rospy.init_node('system_coordinator')
        
        # 任务层状态发布器
        self.task_status_pub = rospy.Publisher('/task_layer/status', String, queue_size=1)
        
        # 订阅任务执行状态
        rospy.Subscriber('/task_execution_status', String, self._task_execution_callback)
        
        # 订阅控制层状态 
        rospy.Subscriber('/robot_status', String, self._control_status_callback)
        
        # 系统状态
        self.task_active = False
        self.robot_status = {}
        
        # 定时发布任务层状态
        rospy.Timer(rospy.Duration(2.0), self._publish_task_status)
        
        rospy.loginfo("系统协调器已启动")
    
    def _task_execution_callback(self, msg):
        """任务执行状态回调"""
        try:
            data = json.loads(msg.data)
            status = data.get('status', 'unknown')
            
            if status == 'running':
                self.task_active = True
                rospy.loginfo("任务开始执行")
            elif status in ['completed', 'failed', 'cancelled']:
                self.task_active = False
                rospy.loginfo("任务执行结束: {}".format(status))
                
        except (json.JSONDecodeError, ValueError):
            rospy.logwarn("任务状态格式错误: {}".format(msg.data))
    
    def _control_status_callback(self, msg):
        """控制层状态回调 - 修复版本"""
        try:
            self.robot_status = json.loads(msg.data)
            
            # 检查机器人状态
            control_mode = self.robot_status.get('control_mode', 'unknown')
            emergency_stop = self.robot_status.get('emergency_stop', False)
            
            if emergency_stop:
                rospy.logwarn("机器人处于紧急停止状态")
                if self.task_active:
                    self.task_active = False
                    rospy.logwarn("由于紧急停止，取消当前任务")
            
        except (json.JSONDecodeError, ValueError):
            rospy.logwarn("机器人状态格式错误: {}".format(msg.data))
    
    def _publish_task_status(self, event):
        """发布任务层状态"""
        status = {
            "active": self.task_active,
            "timestamp": rospy.Time.now().to_sec(),
            "robot_ready": not self.robot_status.get('emergency_stop', True)
        }
        self.task_status_pub.publish(json.dumps(status, ensure_ascii=False))

if __name__ == '__main__':
    try:
        coordinator = SystemCoordinator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("系统协调器已停止")