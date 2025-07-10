#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import threading
import json
import paho.mqtt.client as mqtt
from PyQt5.QtCore import QObject, pyqtSignal

from .logger import Logger

class MQTTClient(QObject):
    """MQTT客户端 - 负责MQTT通信"""
    
    # 定义信号
    message_received = pyqtSignal(str, str)  # topic, payload
    connection_changed = pyqtSignal(bool)
    error_occurred = pyqtSignal(str)
    
    def __init__(self, config):
        super().__init__()
        self.config = config
        self.logger = Logger.get_logger(__name__)
        
        # MQTT配置
        mqtt_config = config.get('mqtt', {})
        self.host = mqtt_config.get('host', 'localhost')
        self.port = mqtt_config.get('port', 1883)
        self.username = mqtt_config.get('username', '')
        self.password = mqtt_config.get('password', '')
        self.keepalive = mqtt_config.get('keepalive', 60)
        
        # 机器人配置
        robot_config = config.get('robot', {})
        self.robot_id = robot_config.get('id', 'transbot_001')
        
        # SSL配置
        ssl_config = mqtt_config.get('ssl', {})
        self.ssl_enabled = ssl_config.get('enabled', False)
        self.ca_cert = ssl_config.get('ca_cert', '')
        
        # 连接状态
        self.connected = False
        self.connecting = False
        
        # 创建MQTT客户端
        self.client = mqtt.Client(client_id=f"robot_controller_{int(time.time())}")
        self.setup_client()
        
        # 重连参数
        network_config = config.get('network', {})
        self.auto_reconnect = network_config.get('auto_reconnect', True)
        self.reconnect_interval = network_config.get('reconnect_interval', 5)
        self.connect_timeout = network_config.get('connect_timeout', 10)
        
        # 重连线程
        self.reconnect_thread = None
        self.stop_reconnect = False
        
        # 订阅的主题列表
        self.subscribed_topics = []
        
        self.logger.info(f"MQTT客户端初始化完成 - 服务器: {self.host}:{self.port}")
    
    def setup_client(self):
        """设置MQTT客户端"""
        # 设置回调函数
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.on_publish = self.on_publish
        self.client.on_subscribe = self.on_subscribe
        self.client.on_log = self.on_log
        
        # 设置用户名和密码
        if self.username and self.password:
            self.client.username_pw_set(self.username, self.password)
            self.logger.info("已设置MQTT用户名和密码")
        
        # 设置SSL
        if self.ssl_enabled:
            try:
                if self.ca_cert:
                    self.client.tls_set(ca_certs=self.ca_cert)
                else:
                    self.client.tls_set()
                self.logger.info("已启用SSL/TLS")
            except Exception as e:
                self.logger.error(f"SSL配置失败: {e}")
                self.error_occurred.emit(f"SSL配置失败: {e}")
    
    def connect(self):
        """连接到MQTT服务器"""
        if self.connecting or self.connected:
            return
        
        try:
            self.connecting = True
            self.logger.info(f"正在连接到MQTT服务器: {self.host}:{self.port}")
            
            # 设置连接超时
            self.client.connect_async(self.host, self.port, self.keepalive)
            self.client.loop_start()
            
        except Exception as e:
            self.connecting = False
            self.logger.error(f"MQTT连接失败: {e}")
            self.error_occurred.emit(f"连接失败: {e}")
    
    def disconnect(self):
        """断开MQTT连接"""
        try:
            self.stop_reconnect = True
            if self.reconnect_thread and self.reconnect_thread.is_alive():
                self.reconnect_thread.join(timeout=1)
            
            if self.connected:
                self.client.disconnect()
            
            self.client.loop_stop()
            self.logger.info("已断开MQTT连接")
            
        except Exception as e:
            self.logger.error(f"断开连接失败: {e}")
    
    def publish(self, topic, payload, qos=0, retain=False):
        """发布消息"""
        if not self.connected:
            self.logger.warning("MQTT未连接，无法发布消息")
            return False
        
        try:
            # 如果payload是字典，转换为JSON字符串
            if isinstance(payload, dict):
                payload = json.dumps(payload, ensure_ascii=False)
            
            result = self.client.publish(topic, payload, qos, retain)
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.logger.debug(f"发布消息成功: {topic}")
                return True
            else:
                self.logger.error(f"发布消息失败: {topic}, 错误码: {result.rc}")
                return False
        except Exception as e:
            self.logger.error(f"发布消息异常: {e}")
            return False
    
    def subscribe(self, topic, qos=0):
        """订阅主题"""
        if not self.connected:
            self.logger.warning("MQTT未连接，无法订阅主题")
            return False
        
        try:
            result, mid = self.client.subscribe(topic, qos)
            if result == mqtt.MQTT_ERR_SUCCESS:
                self.logger.debug(f"订阅主题成功: {topic}")
                if topic not in self.subscribed_topics:
                    self.subscribed_topics.append(topic)
                return True
            else:
                self.logger.error(f"订阅主题失败: {topic}, 错误码: {result}")
                return False
        except Exception as e:
            self.logger.error(f"订阅主题异常: {e}")
            return False
    
    def unsubscribe(self, topic):
        """取消订阅主题"""
        if not self.connected:
            return False
        
        try:
            result, mid = self.client.unsubscribe(topic)
            if result == mqtt.MQTT_ERR_SUCCESS:
                self.logger.debug(f"取消订阅成功: {topic}")
                if topic in self.subscribed_topics:
                    self.subscribed_topics.remove(topic)
                return True
            else:
                self.logger.error(f"取消订阅失败: {topic}, 错误码: {result}")
                return False
        except Exception as e:
            self.logger.error(f"取消订阅异常: {e}")
            return False
    
    def subscribe_robot_topics(self):
        """订阅机器人相关的所有主题"""
        topics = [
            f"robot/{self.robot_id}/status",
            f"robot/{self.robot_id}/feedback",
            f"robot/{self.robot_id}/telemetry",
            f"robot/{self.robot_id}/heartbeat",
            f"robot/{self.robot_id}/task_status",
            f"robot/{self.robot_id}/task_feedback",
            "system/broadcast"
        ]
        
        for topic in topics:
            self.subscribe(topic)
    
    # ================== 机器人控制命令发送方法 ==================
    
    def send_mode_command(self, mode):
        """发送模式切换命令"""
        topic = f"robot/{self.robot_id}/command/mode"
        payload = {
            "mode": mode,
            "timestamp": time.time()
        }
        return self.publish(topic, payload)
    
    def send_velocity_command(self, linear_x, linear_y=0.0, angular_z=0.0):
        """发送速度控制命令"""
        topic = f"robot/{self.robot_id}/command/velocity"
        payload = {
            "linear_x": linear_x,
            "linear_y": linear_y,
            "angular_z": angular_z,
            "timestamp": time.time()
        }
        return self.publish(topic, payload)
    
    def send_navigation_command(self, x, y, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """发送导航命令"""
        topic = f"robot/{self.robot_id}/command/navigation"
        payload = {
            "x": x,
            "y": y,
            "z": z,
            "qx": qx,
            "qy": qy,
            "qz": qz,
            "qw": qw,
            "timestamp": time.time()
        }
        return self.publish(topic, payload)
    
    def send_elevator_command(self, position):
        """发送升降杆命令"""
        topic = f"robot/{self.robot_id}/command/elevator"
        payload = {
            "position": position,
            "timestamp": time.time()
        }
        return self.publish(topic, payload)
    
    def send_camera_command(self, capture=True):
        """发送相机命令"""
        topic = f"robot/{self.robot_id}/command/camera"
        payload = {
            "capture": capture,
            "timestamp": time.time()
        }
        return self.publish(topic, payload)
    
    def send_emergency_stop(self, emergency=True):
        """发送紧急停止命令"""
        topic = f"robot/{self.robot_id}/command/emergency"
        payload = {
            "emergency": emergency,
            "timestamp": time.time()
        }
        return self.publish(topic, payload)
    
    def send_task_command(self, task_data):
        """发送任务命令"""
        topic = f"robot/{self.robot_id}/command/task"
        # 确保task_data包含时间戳
        if isinstance(task_data, dict):
            task_data["timestamp"] = time.time()
        return self.publish(topic, task_data)
    
    def request_status(self):
        """请求机器人状态"""
        topic = f"robot/{self.robot_id}/request/status"
        payload = {
            "request_type": "status",
            "timestamp": time.time()
        }
        return self.publish(topic, payload)
    
    def request_image(self):
        """请求机器人图像"""
        topic = f"robot/{self.robot_id}/request/image"
        payload = {
            "request_type": "image",
            "timestamp": time.time()
        }
        return self.publish(topic, payload)
    
    # ================== MQTT回调函数 ==================
    
    def on_connect(self, client, userdata, flags, rc):
        """连接回调"""
        self.connecting = False
        
        if rc == 0:
            self.connected = True
            self.logger.info("MQTT连接成功")
            self.connection_changed.emit(True)
            
            # 自动订阅机器人主题
            self.subscribe_robot_topics()
            
        else:
            self.connected = False
            error_msg = f"MQTT连接失败，错误代码: {rc}"
            self.logger.error(error_msg)
            self.error_occurred.emit(error_msg)
            self.connection_changed.emit(False)
            
            # 启动自动重连
            if self.auto_reconnect:
                self.start_reconnect()
    
    def on_disconnect(self, client, userdata, rc):
        """断开连接回调"""
        self.connected = False
        self.logger.warning(f"MQTT连接断开，错误代码: {rc}")
        self.connection_changed.emit(False)
        
        # 启动自动重连
        if self.auto_reconnect and not self.stop_reconnect:
            self.start_reconnect()
    
    def on_message(self, client, userdata, msg):
        """消息接收回调"""
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')
            
            self.logger.debug(f"收到MQTT消息: {topic}")
            
            # 发射信号给GUI
            self.message_received.emit(topic, payload)
            
        except Exception as e:
            self.logger.error(f"处理MQTT消息失败: {e}")
            self.error_occurred.emit(f"消息处理失败: {e}")
    
    def on_publish(self, client, userdata, mid):
        """消息发布回调"""
        self.logger.debug(f"消息发布成功, MID: {mid}")
    
    def on_subscribe(self, client, userdata, mid, granted_qos):
        """订阅回调"""
        self.logger.debug(f"订阅成功, MID: {mid}, QoS: {granted_qos}")
    
    def on_log(self, client, userdata, level, buf):
        """日志回调"""
        # 只记录错误级别的日志
        if level == mqtt.MQTT_LOG_ERR:
            self.logger.error(f"MQTT错误: {buf}")
        elif level == mqtt.MQTT_LOG_WARNING:
            self.logger.warning(f"MQTT警告: {buf}")
    
    # ================== 重连机制 ==================
    
    def start_reconnect(self):
        """启动重连线程"""
        if self.reconnect_thread and self.reconnect_thread.is_alive():
            return
        
        self.stop_reconnect = False
        self.reconnect_thread = threading.Thread(target=self.reconnect_loop, daemon=True)
        self.reconnect_thread.start()
        self.logger.info("启动MQTT自动重连")
    
    def reconnect_loop(self):
        """重连循环"""
        while not self.stop_reconnect and not self.connected:
            try:
                self.logger.info(f"尝试重新连接MQTT服务器...")
                
                # 停止当前客户端
                self.client.loop_stop()
                
                # 重新连接
                self.client.connect(self.host, self.port, self.keepalive)
                self.client.loop_start()
                
                # 等待连接结果
                time.sleep(2)
                
                if self.connected:
                    self.logger.info("MQTT重连成功")
                    break
                    
            except Exception as e:
                self.logger.error(f"MQTT重连失败: {e}")
            
            # 等待重连间隔
            time.sleep(self.reconnect_interval)
    
    # ================== 实用方法 ==================
    
    def is_connected(self):
        """检查是否已连接"""
        return self.connected
    
    def get_connection_info(self):
        """获取连接信息"""
        return {
            "host": self.host,
            "port": self.port,
            "connected": self.connected,
            "robot_id": self.robot_id,
            "subscribed_topics": self.subscribed_topics.copy()
        }
    
    def update_config(self, new_config):
        """更新配置"""
        # 断开当前连接
        if self.connected:
            self.disconnect()
        
        # 更新配置
        self.config = new_config
        
        # 重新初始化配置
        mqtt_config = new_config.get('mqtt', {})
        self.host = mqtt_config.get('host', 'localhost')
        self.port = mqtt_config.get('port', 1883)
        self.username = mqtt_config.get('username', '')
        self.password = mqtt_config.get('password', '')
        
        robot_config = new_config.get('robot', {})
        self.robot_id = robot_config.get('id', 'transbot_001')
        
        # 重新设置客户端
        self.setup_client()
        
        self.logger.info("MQTT配置已更新")
    
    def get_robot_topics(self):
        """获取机器人相关主题列表"""
        return {
            'commands': {
                'mode': f"robot/{self.robot_id}/command/mode",
                'velocity': f"robot/{self.robot_id}/command/velocity",
                'navigation': f"robot/{self.robot_id}/command/navigation",
                'elevator': f"robot/{self.robot_id}/command/elevator",
                'camera': f"robot/{self.robot_id}/command/camera",
                'emergency': f"robot/{self.robot_id}/command/emergency",
                'task': f"robot/{self.robot_id}/command/task"
            },
            'reports': {
                'status': f"robot/{self.robot_id}/status",
                'feedback': f"robot/{self.robot_id}/feedback",
                'telemetry': f"robot/{self.robot_id}/telemetry",
                'heartbeat': f"robot/{self.robot_id}/heartbeat",
                'task_status': f"robot/{self.robot_id}/task_status",
                'task_feedback': f"robot/{self.robot_id}/task_feedback"
            },
            'requests': {
                'status': f"robot/{self.robot_id}/request/status",
                'image': f"robot/{self.robot_id}/request/image"
            }
        }