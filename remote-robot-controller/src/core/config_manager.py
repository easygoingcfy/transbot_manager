#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import json
import yaml
from pathlib import Path
from .logger import Logger

class ConfigManager:
    """配置管理器"""
    
    def __init__(self):
        self.logger = Logger.get_logger(__name__)
        
        # 配置文件路径
        self.config_dir = Path(__file__).parent.parent.parent / "config"
        self.default_config_file = self.config_dir / "default_config.yaml"
        self.user_config_file = Path.home() / ".robot_controller_config.yaml"
        
        # 当前配置
        self.config = self.load_config()
        
        self.logger.info("配置管理器初始化完成")
    
    def load_config(self):
        """加载配置"""
        config = self.get_default_config()
        
        # 加载默认配置文件
        if self.default_config_file.exists():
            try:
                with open(self.default_config_file, 'r', encoding='utf-8') as f:
                    default_config = yaml.safe_load(f)
                if default_config:
                    config.update(default_config)
                    self.logger.info("已加载默认配置文件")
            except Exception as e:
                self.logger.error(f"加载默认配置文件失败: {e}")
        
        # 加载用户配置文件
        if self.user_config_file.exists():
            try:
                with open(self.user_config_file, 'r', encoding='utf-8') as f:
                    user_config = yaml.safe_load(f)
                if user_config:
                    config = self.merge_config(config, user_config)
                    self.logger.info("已加载用户配置文件")
            except Exception as e:
                self.logger.error(f"加载用户配置文件失败: {e}")
        
        return config
    
    def get_default_config(self):
        """获取默认配置"""
        return {
            'mqtt': {
                'host': 'localhost',
                'port': 1883,
                'username': '',
                'password': '',
                'keepalive': 60,
                'ssl': {
                    'enabled': False,
                    'ca_cert': ''
                }
            },
            'robot': {
                'id': 'transbot_001',
                'name': 'TransBot',
                'type': 'transbot',
                'control': {
                    'max_linear_velocity': 0.5,  # m/s
                    'max_angular_velocity': 1.0,  # rad/s
                    'command_timeout': 60
                },
                'safety': {
                    'emergency_enabled': True,
                    'collision_detection': False
                }
            },
            'ui': {
                'theme': '默认',
                'language': '中文',
                'font_size': 10,
                'status_update_rate': 10,  # Hz
                'image_update_rate': 5,    # Hz
                'logging': {
                    'level': 'INFO',
                    'to_file': False
                }
            },
            'network': {
                'connect_timeout': 10,
                'reconnect_interval': 5,
                'auto_reconnect': True
            },
            'topic_mapping': {
                'commands': {
                    'mode': 'robot/{robot_id}/command/mode',
                    'velocity': 'robot/{robot_id}/command/velocity',
                    'navigation': 'robot/{robot_id}/command/navigation',
                    'elevator': 'robot/{robot_id}/command/elevator',
                    'camera': 'robot/{robot_id}/command/camera',
                    'emergency': 'robot/{robot_id}/command/emergency',
                    'task': 'robot/{robot_id}/command/task'
                },
                'reports': {
                    'status': 'robot/{robot_id}/status',
                    'feedback': 'robot/{robot_id}/feedback',
                    'telemetry': 'robot/{robot_id}/telemetry',
                    'heartbeat': 'robot/{robot_id}/heartbeat',
                    'task_status': 'robot/{robot_id}/task_status',
                    'task_feedback': 'robot/{robot_id}/task_feedback'
                }
            }
        }
    
    def merge_config(self, base_config, user_config):
        """递归合并配置"""
        merged = base_config.copy()
        
        for key, value in user_config.items():
            if key in merged and isinstance(merged[key], dict) and isinstance(value, dict):
                merged[key] = self.merge_config(merged[key], value)
            else:
                merged[key] = value
        
        return merged
    
    def get_config(self):
        """获取当前配置"""
        return self.config.copy()
    
    def save_config(self, config=None):
        """保存配置到用户配置文件"""
        if config is not None:
            self.config = config
        
        try:
            # 确保配置目录存在
            self.user_config_file.parent.mkdir(parents=True, exist_ok=True)
            
            with open(self.user_config_file, 'w', encoding='utf-8') as f:
                yaml.dump(self.config, f, default_flow_style=False, 
                         allow_unicode=True, sort_keys=True)
            
            self.logger.info("配置已保存到用户配置文件")
            return True
        except Exception as e:
            self.logger.error(f"保存配置失败: {e}")
            return False
    
    def load_config_from_file(self, filepath):
        """从指定文件加载配置"""
        try:
            filepath = Path(filepath)
            if not filepath.exists():
                raise FileNotFoundError(f"配置文件不存在: {filepath}")
            
            with open(filepath, 'r', encoding='utf-8') as f:
                if filepath.suffix.lower() in ['.yaml', '.yml']:
                    loaded_config = yaml.safe_load(f)
                elif filepath.suffix.lower() == '.json':
                    loaded_config = json.load(f)
                else:
                    raise ValueError("不支持的配置文件格式")
            
            if loaded_config:
                default_config = self.get_default_config()
                self.config = self.merge_config(default_config, loaded_config)
                self.logger.info(f"从文件加载配置: {filepath}")
                return True
            else:
                raise ValueError("配置文件内容为空")
                
        except Exception as e:
            self.logger.error(f"从文件加载配置失败: {e}")
            return False
    
    def export_config(self, filepath):
        """导出配置到指定文件"""
        try:
            filepath = Path(filepath)
            filepath.parent.mkdir(parents=True, exist_ok=True)
            
            with open(filepath, 'w', encoding='utf-8') as f:
                if filepath.suffix.lower() in ['.yaml', '.yml']:
                    yaml.dump(self.config, f, default_flow_style=False, 
                             allow_unicode=True, sort_keys=True)
                elif filepath.suffix.lower() == '.json':
                    json.dump(self.config, f, ensure_ascii=False, indent=2)
                else:
                    raise ValueError("不支持的配置文件格式")
            
            self.logger.info(f"配置已导出到: {filepath}")
            return True
        except Exception as e:
            self.logger.error(f"导出配置失败: {e}")
            return False
    
    def reset_to_defaults(self):
        """重置为默认配置"""
        self.config = self.get_default_config()
        self.logger.info("配置已重置为默认值")
    
    def get_mqtt_config(self):
        """获取MQTT配置"""
        return self.config.get('mqtt', {})
    
    def get_robot_config(self):
        """获取机器人配置"""
        return self.config.get('robot', {})
    
    def get_ui_config(self):
        """获取UI配置"""
        return self.config.get('ui', {})
    
    def get_network_config(self):
        """获取网络配置"""
        return self.config.get('network', {})
    
    def update_mqtt_config(self, mqtt_config):
        """更新MQTT配置"""
        self.config['mqtt'] = mqtt_config
        return self.save_config()
    
    def update_robot_config(self, robot_config):
        """更新机器人配置"""
        self.config['robot'] = robot_config
        return self.save_config()
    
    def validate_config(self, config=None):
        """验证配置有效性"""
        if config is None:
            config = self.config
        
        errors = []
        
        # 验证MQTT配置
        mqtt_config = config.get('mqtt', {})
        if not mqtt_config.get('host'):
            errors.append("MQTT服务器地址不能为空")
        if not isinstance(mqtt_config.get('port'), int) or not (1 <= mqtt_config.get('port') <= 65535):
            errors.append("MQTT端口必须是1-65535之间的整数")
        
        # 验证机器人配置
        robot_config = config.get('robot', {})
        if not robot_config.get('id'):
            errors.append("机器人ID不能为空")
        
        # 验证控制参数
        control_config = robot_config.get('control', {})
        if not isinstance(control_config.get('max_linear_velocity'), (int, float)) or control_config.get('max_linear_velocity') <= 0:
            errors.append("最大线速度必须是正数")
        if not isinstance(control_config.get('max_angular_velocity'), (int, float)) or control_config.get('max_angular_velocity') <= 0:
            errors.append("最大角速度必须是正数")
        
        return len(errors) == 0, errors