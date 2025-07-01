#!/usr/bin/env python
import rospy
import actionlib
from atomic_behaviors import AtomicBehavior
from robot_enums import TaskResult
from std_msgs.msg import Float32, Bool
from task_status_manager import status_manager

class ElevatorControlBehavior(AtomicBehavior):
    """升降杆控制行为"""
    
    def __init__(self):
        super().__init__("ElevatorControl")
        # 升降杆控制发布器
        self.elevator_cmd_pub = rospy.Publisher('/elevator/position_cmd', Float32, queue_size=1)
        self.elevator_enable_pub = rospy.Publisher('/elevator/enable', Bool, queue_size=1)
        
        # 升降杆状态订阅器
        self.current_position = 0.0
        self.position_sub = rospy.Subscriber('/elevator/current_position', Float32, self._position_callback)
        
    def execute(self, target_position, speed=0.1, timeout=30.0):
        """
        执行升降杆控制
        :param target_position: 目标位置 (0.0-1.0, 0为最低，1为最高)
        :param speed: 移动速度
        :param timeout: 超时时间
        """
        self.publish_status(f"升降杆移动到位置: {target_position}")
        status_manager.update_progress("升降杆启动", 5.0)
        
        # 启用升降杆
        self.elevator_enable_pub.publish(Bool(True))
        rospy.sleep(0.5)  # 等待启用
        
        # 发送目标位置
        self.elevator_cmd_pub.publish(Float32(target_position))
        status_manager.update_progress("升降杆移动中", 20.0)
        
        # 监控移动进度
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        tolerance = 0.02  # 位置容差
        
        while not rospy.is_shutdown():
            if self._cancelled:
                self._stop_elevator()
                return TaskResult.CANCELLED
            
            # 检查是否到达目标位置
            position_error = abs(self.current_position - target_position)
            if position_error < tolerance:
                status_manager.update_progress("升降杆到位", 100.0, {
                    'final_position': self.current_position,
                    'target_position': target_position
                })
                return TaskResult.SUCCESS
            
            # 更新进度
            max_range = 1.0  # 最大行程
            progress = 20.0 + 70.0 * (1.0 - position_error / max_range)
            status_manager.update_progress("升降杆移动中", min(95.0, progress), {
                'current_position': self.current_position,
                'target_position': target_position
            })
            
            # 检查超时
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                self._stop_elevator()
                return TaskResult.TIMEOUT
            
            rate.sleep()
        
        return TaskResult.FAILURE
    
    def _position_callback(self, msg):
        """升降杆位置反馈回调"""
        self.current_position = msg.data
    
    def _stop_elevator(self):
        """停止升降杆"""
        self.elevator_enable_pub.publish(Bool(False))
        self.publish_status("升降杆已停止")
    
    def cancel(self):
        super().cancel()
        self._stop_elevator()