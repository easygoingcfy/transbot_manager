#!/usr/bin/env python
#coding=utf-8
# filepath: /home/jetson/transbot_ws/src/transbot_manager/scripts/behavior_factory.py

import rospy
from atomic_behaviors import (
    NavigateBehavior, RotateBehavior, CaptureImageBehavior, 
    SpeakBehavior, WaitBehavior
)

# 尝试导入电梯控制，如果不存在则跳过
try:
    from elevator_behavior import ElevatorControlBehavior
    ELEVATOR_AVAILABLE = True
except ImportError:
    rospy.logwarn("elevator_behavior模块未找到，电梯控制功能不可用")
    ELEVATOR_AVAILABLE = False

class BehaviorFactory:
    """行为工厂，负责创建和管理原子行为"""
    
    _behaviors = {}
    
    @classmethod
    def get_behavior(cls, behavior_type):
        """获取行为实例（单例模式）"""
        if behavior_type not in cls._behaviors:
            cls._behaviors[behavior_type] = cls._create_behavior(behavior_type)
        return cls._behaviors[behavior_type]
    
    @classmethod
    def _create_behavior(cls, behavior_type):
        """创建具体行为实例"""
        behavior_map = {
            'navigate': NavigateBehavior,
            'rotate': RotateBehavior,
            'capture_image': CaptureImageBehavior,
            'speak': SpeakBehavior,
            'wait': WaitBehavior,
        }
        
        # 如果电梯控制可用，添加到映射中
        if ELEVATOR_AVAILABLE:
            behavior_map['elevator_control'] = ElevatorControlBehavior
        
        if behavior_type not in behavior_map:
            rospy.logwarn("未知的行为类型: {}，使用等待行为".format(behavior_type))
            return WaitBehavior()
        
        rospy.loginfo("创建行为实例: {}".format(behavior_type))
        return behavior_map[behavior_type]()
    
    @classmethod
    def list_available_behaviors(cls):
        """列出所有可用的行为类型"""
        behaviors = ['navigate', 'rotate', 'capture_image', 'speak', 'wait']
        if ELEVATOR_AVAILABLE:
            behaviors.append('elevator_control')
        return behaviors
    
    @classmethod
    def cleanup(cls):
        """清理所有行为实例"""
        for behavior in cls._behaviors.values():
            if hasattr(behavior, 'cancel'):
                behavior.cancel()
        cls._behaviors.clear()
        rospy.loginfo("行为工厂已清理")