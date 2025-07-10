#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import threading
import json
import sys
import paho.mqtt.client as mqtt
from PyQt5.QtCore import QObject, pyqtSignal

from .logger import Logger

# Python 2/3 兼容性处理
if sys.version_info[0] == 2:
    def format_string(template, **kwargs):
        return template.format(**kwargs)
else:
    def format_string(template, **kwargs):
        return template.format(**kwargs)

class MQTTClient(QObject):
    """MQTT客户端 - 负责MQTT通信"""
    
    # 定义信号
    status_received = pyqtSignal(dict)  # 状态响应
    image_received = pyqtSignal(dict)   # 图像响应
    message_received = pyqtSignal(str, str)  # topic, payload
    connection_changed = pyqtSignal(bool)
    error_occurred = pyqtSignal(str)
    
    # 新增响应信号
    mode_response_received = pyqtSignal(dict)
    connection_test_response_received = pyqtSignal(dict)
    pong_received = pyqtSignal(dict)
    
    def __init__(self, config):
        super(MQTTClient, self).__init__()
        self.config = config
        self.logger = Logger.get_logger(__name__)
        
        # MQTT配置
        mqtt_config = config.get('mqtt', {})
        self.host = mqtt_config.get('host', 'localhost')
        self.port = mqtt_config.get('port', 1883)
        self.username = mqtt_config.get('username', '')
        self.password = mqtt_config.get('password', '')
        self.keepalive = mqtt_config.get('keepalive', 60)
        
        # 请求追踪
        self.pending_requests = {}
        self.request_timeout = 10  # 请求超时时间(秒)
        
        # 机器人配置
        robot_config = config.get('robot', {})
        self.robot_id = robot_config.get('id', 'transbot_001')
        
        # 主题映射配置
        self.topic_mapping = config.get('topic_mapping', {})
        
        # SSL配置
        ssl_config = mqtt_config.get('ssl', {})
        self.ssl_enabled = ssl_config.get('enabled', False)
        self.ca_cert = ssl_config.get('ca_cert', '')
        
        # 连接状态
        self.connected = False
        self.connecting = False
        
        # 创建MQTT客户端 - Python 2兼容
        client_id = "robot_controller_{}".format(int(time.time()))
        self.client = mqtt.Client(client_id=client_id)
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
        
        self.logger.info("MQTT客户端初始化完成 - 服务器: {}:{}".format(self.host, self.port))
    
    def get_topic(self, category, topic_name):
        """获取格式化的主题名称"""
        try:
            template = self.topic_mapping[category][topic_name]
            return template.format(robot_id=self.robot_id)
        except KeyError:
            # 如果配置不存在，使用默认格式
            return "robot/{}/{}".format(self.robot_id, topic_name)
    
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
                self.logger.error("SSL配置失败: {}".format(e))
                self.error_occurred.emit("SSL配置失败: {}".format(e))
    
    def connect(self):
        """连接到MQTT服务器"""
        if self.connecting or self.connected:
            return
        
        try:
            self.connecting = True
            self.logger.info("正在连接到MQTT服务器: {}:{}".format(self.host, self.port))
            
            # 设置连接超时
            self.client.connect_async(self.host, self.port, self.keepalive)
            self.client.loop_start()
            
        except Exception as e:
            self.connecting = False
            self.logger.error("MQTT连接失败: {}".format(e))
            self.error_occurred.emit("连接失败: {}".format(e))
    
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
            self.logger.error("断开连接失败: {}".format(e))
    
    def subscribe_robot_topics(self):
        """订阅机器人相关的所有主题"""
        # 报告主题
        report_topics = [
            self.get_topic('reports', 'status'),
            self.get_topic('reports', 'feedback'),
            self.get_topic('reports', 'telemetry'),
            self.get_topic('reports', 'heartbeat'),
            self.get_topic('reports', 'task_status'),
            self.get_topic('reports', 'task_feedback'),
        ]
        
        # 响应主题 - 重点修复
        response_topics = [
            self.get_topic('responses', 'status'),
            self.get_topic('responses', 'image'),
            self.get_topic('responses', 'mode'),
            self.get_topic('responses', 'connection_test'),
        ]
        
        # 系统主题
        system_topics = [
            self.get_topic('system', 'pong'),
            self.get_topic('system', 'broadcast'),
        ]
        
        all_topics = report_topics + response_topics + system_topics
        
        for topic in all_topics:
            self.subscribe(topic)
            self.logger.info("订阅主题: {}".format(topic))
    
    def subscribe(self, topic, qos=0):
        """订阅主题"""
        if not self.connected:
            self.logger.warning("MQTT未连接，无法订阅主题")
            return False
        
        try:
            result, mid = self.client.subscribe(topic, qos)
            if result == mqtt.MQTT_ERR_SUCCESS:
                self.logger.debug("订阅主题成功: {}".format(topic))
                if topic not in self.subscribed_topics:
                    self.subscribed_topics.append(topic)
                return True
            else:
                self.logger.error("订阅主题失败: {}, 错误码: {}".format(topic, result))
                return False
        except Exception as e:
            self.logger.error("订阅主题异常: {}".format(e))
            return False
    
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
                self.logger.debug("发布消息成功: {}".format(topic))
                return True
            else:
                self.logger.error("发布消息失败: {}, 错误码: {}".format(topic, result.rc))
                return False
        except Exception as e:
            self.logger.error("发布消息异常: {}".format(e))
            return False
    
    # ================== 机器人控制命令发送方法 ==================
    
    def send_mode_command(self, mode):
        """发送模式切换命令"""
        topic = self.get_topic('commands', 'mode')
        payload = {
            "mode": mode,
            "timestamp": time.time()
        }
        return self.publish(topic, payload)
    
    def send_velocity_command(self, linear_x, linear_y=0.0, angular_z=0.0):
        """发送速度控制命令"""
        topic = self.get_topic('commands', 'velocity')
        payload = {
            "linear_x": linear_x,
            "linear_y": linear_y,
            "angular_z": angular_z,
            "timestamp": time.time()
        }
        return self.publish(topic, payload)
    
    def send_navigation_command(self, x, y, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """发送导航命令"""
        topic = self.get_topic('commands', 'navigation')
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
        topic = self.get_topic('commands', 'elevator')
        payload = {
            "position": position,
            "timestamp": time.time()
        }
        return self.publish(topic, payload)
    
    def send_camera_command(self, capture=True):
        """发送相机命令"""
        topic = self.get_topic('commands', 'camera')
        payload = {
            "capture": capture,
            "timestamp": time.time()
        }
        return self.publish(topic, payload)
    
    def send_emergency_stop(self, emergency=True):
        """发送紧急停止命令"""
        topic = self.get_topic('commands', 'emergency')
        payload = {
            "emergency": emergency,
            "timestamp": time.time()
        }
        return self.publish(topic, payload)
    
    def send_task_command(self, task_data):
        """发送任务命令"""
        topic = self.get_topic('commands', 'task')
        # 确保task_data包含时间戳
        if isinstance(task_data, dict):
            task_data["timestamp"] = time.time()
        return self.publish(topic, task_data)
    
    # ================== 新增请求方法 ==================
    
    def request_robot_status(self, status_type='full', callback=None):
        """请求机器人状态"""
        request_id = "status_{}".format(int(time.time() * 1000))
        topic = self.get_topic('requests', 'status')
        payload = {
            "type": status_type,
            "request_id": request_id,
            "timestamp": time.time()
        }
        
        # 记录请求
        if callback:
            self.pending_requests[request_id] = {
                "callback": callback,
                "timestamp": time.time(),
                "type": "status"
            }
        
        self.logger.info("发送状态请求: {}".format(request_id))
        return self.publish(topic, payload)
    
    def request_robot_image(self, image_type='rgb', quality=80, callback=None):
        """请求机器人图像"""
        request_id = "image_{}".format(int(time.time() * 1000))
        topic = self.get_topic('requests', 'image')
        payload = {
            "type": image_type,
            "quality": quality,
            "request_id": request_id,
            "timestamp": time.time()
        }
        
        # 记录请求
        if callback:
            self.pending_requests[request_id] = {
                "callback": callback,
                "timestamp": time.time(),
                "type": "image"
            }
        
        self.logger.info("发送图像请求: {}".format(request_id))
        return self.publish(topic, payload)
    
    def test_connection(self, callback=None):
        """测试连接"""
        request_id = "test_{}".format(int(time.time() * 1000))
        topic = self.get_topic('requests', 'connection_test')
        payload = {
            "request_id": request_id,
            "timestamp": time.time()
        }
        
        # 记录请求
        if callback:
            self.pending_requests[request_id] = {
                "callback": callback,
                "timestamp": time.time(),
                "type": "connection_test"
            }
        
        self.logger.info("发送连接测试: {}".format(request_id))
        return self.publish(topic, payload)
    
    def ping_robot(self):
        """Ping机器人"""
        topic = self.get_topic('system', 'ping')
        payload = {
            "timestamp": time.time()
        }
        self.logger.info("发送ping到机器人")
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
            error_msg = "MQTT连接失败，错误代码: {}".format(rc)
            self.logger.error(error_msg)
            self.error_occurred.emit(error_msg)
            self.connection_changed.emit(False)
            
            # 启动自动重连
            if self.auto_reconnect:
                self.start_reconnect()
    
    def on_disconnect(self, client, userdata, rc):
        """断开连接回调"""
        self.connected = False
        self.logger.warning("MQTT连接断开，错误代码: {}".format(rc))
        self.connection_changed.emit(False)
        
        # 启动自动重连
        if self.auto_reconnect and not self.stop_reconnect:
            self.start_reconnect()
    
    def on_message(self, client, userdata, msg):
        """消息接收回调 - 重点修复"""
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')
            
            self.logger.info("收到MQTT消息: {} - {}".format(topic, payload[:100]))
            
            # 解析JSON数据
            try:
                data = json.loads(payload)
            except json.JSONDecodeError:
                self.logger.error("JSON解析失败: {}".format(payload))
                return
            
            # 处理响应消息
            if '/response/' in topic:
                self._handle_response_message(topic, data)
            elif topic.endswith('/pong'):
                self._handle_pong_message(topic, data)
            elif '/status' in topic:
                self.status_received.emit(data)
            elif '/feedback' in topic:
                self.message_received.emit(topic, payload)
            elif '/telemetry' in topic:
                self.message_received.emit(topic, payload)
            elif '/heartbeat' in topic:
                self.message_received.emit(topic, payload)
            elif '/task_status' in topic:
                self.message_received.emit(topic, payload)
            elif '/task_feedback' in topic:
                self.message_received.emit(topic, payload)
            else:
                # 其他消息
                self.message_received.emit(topic, payload)
            
        except Exception as e:
            self.logger.error("处理MQTT消息失败: {}".format(e))
            self.error_occurred.emit("消息处理失败: {}".format(e))
    
    def _handle_response_message(self, topic, data):
        """处理响应消息 - 重点修复"""
        try:
            request_id = data.get('request_id')
            
            # 处理待处理请求
            if request_id and request_id in self.pending_requests:
                request_info = self.pending_requests.pop(request_id)
                callback = request_info.get('callback')
                
                if callback:
                    callback(data)
                
                self.logger.info("处理请求响应: {} - {}".format(request_id, request_info.get('type')))
            
            # 根据响应类型发射相应信号
            if '/response/status' in topic:
                self.status_received.emit(data)
                self.logger.info("收到状态响应")
            elif '/response/image' in topic:
                self.image_received.emit(data)
                self.logger.info("收到图像响应")
            elif '/response/mode' in topic:
                self.mode_response_received.emit(data)
                self.logger.info("收到模式响应")
            elif '/response/connection_test' in topic:
                self.connection_test_response_received.emit(data)
                self.logger.info("收到连接测试响应")
            
        except Exception as e:
            self.logger.error("响应消息处理失败: {}".format(e))
    
    def _handle_pong_message(self, topic, data):
        """处理pong消息"""
        try:
            self.pong_received.emit(data)
            self.logger.info("收到pong响应")
        except Exception as e:
            self.logger.error("pong消息处理失败: {}".format(e))
    
    def on_publish(self, client, userdata, mid):
        """消息发布回调"""
        self.logger.debug("消息发布成功, MID: {}".format(mid))
    
    def on_subscribe(self, client, userdata, mid, granted_qos):
        """订阅回调"""
        self.logger.debug("订阅成功, MID: {}, QoS: {}".format(mid, granted_qos))
    
    def on_log(self, client, userdata, level, buf):
        """日志回调"""
        # 只记录错误级别的日志
        if level == mqtt.MQTT_LOG_ERR:
            self.logger.error("MQTT错误: {}".format(buf))
        elif level == mqtt.MQTT_LOG_WARNING:
            self.logger.warning("MQTT警告: {}".format(buf))
    
    # ================== 重连机制 ==================
    
    def start_reconnect(self):
        """启动重连线程"""
        if self.reconnect_thread and self.reconnect_thread.is_alive():
            return
        
        self.stop_reconnect = False
        self.reconnect_thread = threading.Thread(target=self.reconnect_loop)
        self.reconnect_thread.daemon = True
        self.reconnect_thread.start()
        self.logger.info("启动MQTT自动重连")
    
    def reconnect_loop(self):
        """重连循环"""
        while not self.stop_reconnect and not self.connected:
            try:
                self.logger.info("尝试重新连接MQTT服务器...")
                
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
                self.logger.error("MQTT重连失败: {}".format(e))
            
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