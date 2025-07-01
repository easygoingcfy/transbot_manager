#!/usr/bin/env python
from atomic_behaviors import *
from elevator_behavior import ElevatorControlBehavior

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
            'elevator_control': ElevatorControlBehavior,
            'capture_image': CaptureImageBehavior,
            'speak': SpeakBehavior,
            'wait': WaitBehavior,
        }
        
        if behavior_type not in behavior_map:
            raise ValueError(f"未知的行为类型: {behavior_type}")
        
        return behavior_map[behavior_type]()
    
    @classmethod
    def list_available_behaviors(cls):
        """列出所有可用的行为类型"""
        return ['navigate', 'rotate', 'capture_image', 'speak', 'wait']