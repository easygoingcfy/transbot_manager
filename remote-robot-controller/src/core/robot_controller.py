#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import time
from PyQt5.QtCore import QObject, pyqtSignal, QTimer

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
    image_data_received = pyqtSignal(dict)  # 图像数据接收
    mapping_status_updated = pyqtSignal(str)  # 建图状态更新
    mapping_response_received = pyqtSignal(dict)  # 建图响应
    
    def __init__(self, config):
        super().__init__()
        self.config = config
        self.logger = Logger.get_logger(__name__)

        # 添加调试标志
        self.debug_mode = True
        
        # 创建MQTT客户端
        self.mqtt_client = MQTTClient(config)
        
        # 连接MQTT客户端信号
        self.mqtt_client.message_received.connect(self.handle_mqtt_message)
        self.mqtt_client.connection_changed.connect(self.on_connection_changed)
        self.mqtt_client.error_occurred.connect(self.error_occurred.emit)
        self.mqtt_client.status_received.connect(self.handle_status_response)
        self.mqtt_client.image_received.connect(self.handle_image_response)
        self.mqtt_client.mapping_status_received.connect(self.mapping_status_updated.emit)
        self.mqtt_client.mapping_response_received.connect(self.mapping_response_received.emit)
        
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
    
    def send_mapping_command(self, action, map_name=None):
        """发送建图命令
        action: 'start' | 'save' | 'set_name'
        map_name: 地图名称（对于save和set_name动作）
        """
        if not self.is_connected():
            self.logger.warning("建图命令发送失败：MQTT未连接")
            return False
            
        try:
            success = self.mqtt_client.send_mapping_command(action, map_name)
            if success:
                self.logger.info("建图命令已发送: {} {}".format(action, map_name or ''))
            else:
                self.logger.warning("建图命令发送失败")
            return success
        except Exception as e:
            self.logger.error("发送建图命令出错: {}".format(e))
            return False
    
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
    
    def handle_status_response(self, status_data):
        """处理状态响应"""
        try:
            if 'status' in status_data:
                self.robot_status.update(status_data['status'])
                self.status_updated.emit(self.robot_status.copy())
                
                if self.debug_mode:
                    print(f"[ROBOT_CONTROLLER] 收到状态响应: {status_data}")
                
        except Exception as e:
            self.logger.error(f"处理状态响应失败: {e}")
    
    def handle_image_response(self, image_data):
        """处理图像响应"""
        try:
            if self.debug_mode:
                print(f"[ROBOT_CONTROLLER] 收到图像响应: 大小={len(image_data.get('image_data', ''))//1024}KB")
            
            self.image_data_received.emit(image_data)
            
        except Exception as e:
            self.logger.error(f"处理图像响应失败: {e}")
    
    def request_current_status(self):
        """请求当前状态"""
        def status_callback(response):
            if self.debug_mode:
                print(f"[ROBOT_CONTROLLER] 状态请求回调: {response}")
        
        return self.mqtt_client.request_robot_status('full', status_callback)
    
    def request_robot_image(self, quality=80):
        """请求机器人图像"""
        def image_callback(response):
            if self.debug_mode:
                print(f"[ROBOT_CONTROLLER] 图像请求回调: 收到{len(response.get('image_data', ''))//1024}KB图像")
        
        return self.mqtt_client.request_robot_image('rgb', quality, image_callback)
    
    def test_robot_connection(self):
        """测试机器人连接"""
        def test_callback(response):
            latency = response.get('latency_ms', 0)
            if self.debug_mode:
                print(f"[ROBOT_CONTROLLER] 连接测试结果: 延迟={latency:.2f}ms")
        
        return self.mqtt_client.test_connection(test_callback)
    
    def update_config(self, new_config):
        """更新配置"""
        self.config = new_config
        self.mqtt_client.update_config(new_config)
        self.logger.info("机器人控制器配置已更新")