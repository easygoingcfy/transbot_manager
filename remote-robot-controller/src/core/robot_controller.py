#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import time
from PyQt5.QtCore import QObject, pyqtSignal, QTimer

# 修复导入 - 使用绝对导入
from core.mqtt_client import MQTTClient
from core.logger import Logger

class RobotController(QObject):
    """机器人控制器 - 统一管理机器人控制逻辑"""
    
    # 定义信号
    status_updated = pyqtSignal(dict)
    feedback_received = pyqtSignal(dict)
    image_received = pyqtSignal(object)  # QImage or bytes
    connection_changed = pyqtSignal(bool)
    error_occurred = pyqtSignal(str)
    
    def __init__(self, config):
        super().__init__()
        self.config = config
        self.logger = Logger.get_logger(__name__)
        
        # 创建MQTT客户端
        self.mqtt_client = MQTTClient(config)
        
        # 连接MQTT客户端信号
        self.mqtt_client.message_received.connect(self.handle_mqtt_message)
        self.mqtt_client.connection_changed.connect(self.on_connection_changed)
        self.mqtt_client.error_occurred.connect(self.error_occurred.emit)
        
        # 机器人状态
        self.robot_status = {
            'mode': 'pause',
            'system_status': 'unknown',
            'position': {'x': 0, 'y': 0, 'z': 0},
            'velocity': {'linear': 0, 'angular': 0},
            'battery': 100,
            'charging': False
        }
        
        self.logger.info("机器人控制器初始化完成")
    
    def connect(self):
        """连接到机器人"""
        self.mqtt_client.connect()
    
    def disconnect(self):
        """断开与机器人的连接"""
        self.mqtt_client.disconnect()
    
    def is_connected(self):
        """检查是否已连接"""
        return self.mqtt_client.is_connected()
    
    def set_mode(self, mode):
        """设置机器人模式"""
        success = self.mqtt_client.send_mode_command(mode)
        if success:
            self.robot_status['mode'] = mode
            self.logger.info(f"模式已设置为: {mode}")
        return success
    
    def send_velocity(self, linear_x, angular_z):
        """发送速度控制命令"""
        return self.mqtt_client.send_velocity_command(linear_x, 0.0, angular_z)
    
    def send_navigation(self, x, y, yaw=0.0):
        """发送导航命令"""
        import math
        qw = math.cos(yaw / 2)
        qz = math.sin(yaw / 2)
        return self.mqtt_client.send_navigation_command(x, y, 0.0, 0.0, 0.0, qz, qw)
    
    def send_elevator(self, position):
        """发送升降杆命令"""
        return self.mqtt_client.send_elevator_command(position)
    
    def send_camera(self, capture=True):
        """发送相机命令"""
        return self.mqtt_client.send_camera_command(capture)
    
    def emergency_stop(self, emergency=True):
        """紧急停止"""
        success = self.mqtt_client.send_emergency_stop(emergency)
        if success and emergency:
            self.robot_status['mode'] = 'pause'
            self.logger.warning("紧急停止已激活")
        return success
    
    def send_task(self, task_data):
        """发送任务"""
        return self.mqtt_client.send_task_command(task_data)
    
    def update_status(self):
        """更新状态（外部调用）"""
        self.mqtt_client.request_status()
    
    def handle_mqtt_message(self, topic, payload):
        """处理MQTT消息"""
        try:
            # 简化的消息处理
            data = json.loads(payload) if isinstance(payload, str) else payload
            
            # 发射状态更新信号
            self.status_updated.emit(self.robot_status.copy())
            
        except Exception as e:
            self.logger.error(f"处理MQTT消息失败: {e}")
    
    def on_connection_changed(self, connected):
        """连接状态变化处理"""
        self.connection_changed.emit(connected)
        
        if connected:
            self.logger.info("已连接到机器人")
        else:
            self.logger.warning("与机器人连接断开")
    
    def update_config(self, new_config):
        """更新配置"""
        self.config = new_config
        self.mqtt_client.update_config(new_config)
        self.logger.info("机器人控制器配置已更新")