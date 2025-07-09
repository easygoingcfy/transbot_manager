#!/usr/bin/env python
#coding=utf-8

import rospy
import json
import time
import threading
import paho.mqtt.client as mqtt
from datetime import datetime
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64
import os

class MQTTBridge:
    """MQTT桥接器 - 负责机器人与云端系统的通信"""
    
    def __init__(self):
        rospy.init_node('mqtt_bridge')
        
        # 获取配置
        self._load_config()
        
        # 连接状态
        self.mqtt_connected = False
        self.last_heartbeat = time.time()
        
        # 图像处理
        self.bridge = CvBridge()
        
        # 线程锁
        self.publish_lock = threading.Lock()
        
        # 初始化MQTT客户端
        self._setup_mqtt_client()
        
        # 初始化ROS发布器
        self._setup_ros_publishers()
        
        # 初始化ROS订阅器
        self._setup_ros_subscribers()
        
        # 启动后台线程
        self._start_background_threads()
        
        rospy.loginfo("MQTT桥接器已启动 - 连接到: {}:{}".format(self.mqtt_broker, self.mqtt_port))
    
    def _load_config(self):
        """从参数服务器加载配置"""
        # MQTT基本配置
        self.mqtt_broker = rospy.get_param('~mqtt_broker', 'localhost')
        self.mqtt_port = rospy.get_param('~mqtt_port', 1883)
        self.mqtt_keepalive = rospy.get_param('~mqtt_keepalive', 60)
        self.mqtt_username = rospy.get_param('~mqtt_username', '')
        self.mqtt_password = rospy.get_param('~mqtt_password', '')
        
        # 机器人ID
        self.robot_id = rospy.get_param('robot_id', 'transbot_001')
        
        # 从YAML配置文件加载主题配置（如果存在）
        try:
            self.topic_config = rospy.get_param('~topics', {})
            rospy.loginfo("已加载主题配置: {}".format(self.topic_config))
        except:
            # 如果没有YAML配置，使用默认配置
            self.topic_config = {
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
            rospy.loginfo("使用默认主题配置")

    def _get_topic(self, category, topic_name):
        """获取格式化的主题名称"""
        try:
            template = self.topic_config[category][topic_name]
            return template.format(robot_id=self.robot_id)
        except KeyError:
            rospy.logwarn("主题配置不存在: {}/{}".format(category, topic_name))
            return "robot/{}/{}".format(self.robot_id, topic_name)
    
    def _setup_mqtt_client(self):
        """设置MQTT客户端"""
        self.mqtt_client = mqtt.Client(client_id=self.robot_id)
        
        # 设置认证
        if self.mqtt_username:
            self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
        
        # 设置回调函数
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message
        
        # 连接到MQTT服务器
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, self.mqtt_keepalive)
            self.mqtt_client.loop_start()
        except Exception as e:
            rospy.logerr("MQTT连接失败: {}".format(e))
    
    def _setup_ros_publishers(self):
        """设置ROS发布器"""
        # 控制指令发布器
        self.mode_pub = rospy.Publisher('/remote_control/mode', String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/remote_control/cmd_vel', Twist, queue_size=1)
        self.goal_pub = rospy.Publisher('/remote_control/goal', PoseStamped, queue_size=1)
        self.elevator_pub = rospy.Publisher('/remote_control/elevator', Float32, queue_size=1)
        self.camera_pub = rospy.Publisher('/remote_control/camera', Bool, queue_size=1)
        self.emergency_pub = rospy.Publisher('/remote_control/emergency_stop', Bool, queue_size=1)
        
        # 任务指令发布器
        self.cloud_task_pub = rospy.Publisher('/cloud_tasks', String, queue_size=1)
        
        # 连接状态发布器
        self.mqtt_status_pub = rospy.Publisher('/mqtt_status', String, queue_size=1)
    
    def _setup_ros_subscribers(self):
        """设置ROS订阅器"""
        # 机器人状态订阅
        rospy.Subscriber('/robot_status', String, self._robot_status_callback)
        rospy.Subscriber('/control_feedback', String, self._control_feedback_callback)
        rospy.Subscriber('/cloud_report', String, self._cloud_report_callback)
        rospy.Subscriber('/robot_telemetry', String, self._telemetry_callback)
        
        # 任务状态订阅
        rospy.Subscriber('/task_execution_status', String, self._task_execution_status_callback)
        rospy.Subscriber('/task_feedback', String, self._task_feedback_callback)
        
        # 图像数据订阅（用于云端图像传输）
        rospy.Subscriber('/camera/rgb/image_raw', Image, self._image_callback)
    
    def _start_background_threads(self):
        """启动后台线程"""
        threads = [
            # threading.Thread(target=self._heartbeat_loop, name="HeartbeatLoop"),
            threading.Thread(target=self._connection_monitor_loop, name="ConnectionMonitor"),
        ]
        
        for thread in threads:
            thread.daemon = True
            thread.start()
            rospy.loginfo("启动MQTT线程: {}".format(thread.name))
    
    # ================== MQTT回调函数 ==================

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT连接回调"""
        if rc == 0:
            self.mqtt_connected = True
            rospy.loginfo("MQTT连接成功")
            
            # 使用配置的主题订阅
            topics = [
                (self._get_topic('commands', 'mode'), 0),
                (self._get_topic('commands', 'velocity'), 0),
                (self._get_topic('commands', 'navigation'), 0),
                (self._get_topic('commands', 'elevator'), 0),
                (self._get_topic('commands', 'camera'), 0),
                (self._get_topic('commands', 'emergency'), 0),
                (self._get_topic('commands', 'task'), 0),
                ("robot/{}/request/status".format(self.robot_id), 0),
                ("robot/{}/request/image".format(self.robot_id), 0),
                ("system/broadcast", 0),
            ]
            
            for topic, qos in topics:
                client.subscribe(topic, qos)
                rospy.loginfo("订阅MQTT主题: {}".format(topic))
                
            # 发布连接状态
            self._publish_mqtt_status("connected")
            
        else:
            rospy.logerr("MQTT连接失败，错误代码: {}".format(rc))
            self.mqtt_connected = False
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT断开回调"""
        self.mqtt_connected = False
        rospy.logwarn("MQTT连接断开，错误代码: {}".format(rc))
        self._publish_mqtt_status("disconnected")
    
    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT消息回调"""
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')
            
            rospy.loginfo("收到MQTT消息: {} - {}".format(topic, payload))
            
            # 解析消息
            if topic.endswith('/command/mode'):
                self._handle_mode_command(payload)
            elif topic.endswith('/command/velocity'):
                self._handle_velocity_command(payload)
            elif topic.endswith('/command/navigation'):
                self._handle_navigation_command(payload)
            elif topic.endswith('/command/elevator'):
                self._handle_elevator_command(payload)
            elif topic.endswith('/command/camera'):
                self._handle_camera_command(payload)
            elif topic.endswith('/command/emergency'):
                self._handle_emergency_command(payload)
            elif topic.endswith('/command/task'):
                self._handle_task_command(payload)
            elif topic.endswith('/request/status'):
                self._handle_status_request(payload)
            elif topic.endswith('/request/image'):
                self._handle_image_request(payload)
            elif topic == 'system/broadcast':
                self._handle_broadcast_message(payload)
            else:
                rospy.logwarn("未知MQTT主题: {}".format(topic))
                
        except Exception as e:
            rospy.logerr("MQTT消息处理错误: {}".format(e))
    
    # ================== 云端指令处理 ==================
    
    def _handle_mode_command(self, payload):
        """处理模式切换指令"""
        try:
            data = json.loads(payload)
            mode = data.get('mode', 'pause')
            
            msg = String()
            msg.data = mode
            self.mode_pub.publish(msg)
            
            rospy.loginfo("模式切换指令: {}".format(mode))
            
        except json.JSONDecodeError:
            rospy.logerr("模式指令格式错误: {}".format(payload))
    
    def _handle_velocity_command(self, payload):
        """处理速度控制指令"""
        try:
            data = json.loads(payload)
            
            twist = Twist()
            twist.linear.x = data.get('linear_x', 0.0)
            twist.linear.y = data.get('linear_y', 0.0)
            twist.angular.z = data.get('angular_z', 0.0)
            
            self.cmd_vel_pub.publish(twist)
            
            rospy.loginfo("速度控制指令: linear_x={}, angular_z={}".format(
                twist.linear.x, twist.angular.z))
            
        except json.JSONDecodeError:
            rospy.logerr("速度指令格式错误: {}".format(payload))
    
    def _handle_navigation_command(self, payload):
        """处理导航指令"""
        try:
            data = json.loads(payload)
            
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            
            goal.pose.position.x = data.get('x', 0.0)
            goal.pose.position.y = data.get('y', 0.0)
            goal.pose.position.z = data.get('z', 0.0)
            
            goal.pose.orientation.x = data.get('qx', 0.0)
            goal.pose.orientation.y = data.get('qy', 0.0)
            goal.pose.orientation.z = data.get('qz', 0.0)
            goal.pose.orientation.w = data.get('qw', 1.0)
            
            self.goal_pub.publish(goal)
            
            rospy.loginfo("导航目标: x={}, y={}".format(
                goal.pose.position.x, goal.pose.position.y))
            
        except json.JSONDecodeError:
            rospy.logerr("导航指令格式错误: {}".format(payload))
    
    def _handle_elevator_command(self, payload):
        """处理升降杆指令"""
        try:
            data = json.loads(payload)
            position = data.get('position', 0.0)
            
            msg = Float32()
            msg.data = position
            self.elevator_pub.publish(msg)
            
            rospy.loginfo("升降杆指令: position={}".format(position))
            
        except json.JSONDecodeError:
            rospy.logerr("升降杆指令格式错误: {}".format(payload))
    
    def _handle_camera_command(self, payload):
        """处理相机指令"""
        try:
            data = json.loads(payload)
            capture = data.get('capture', False)
            
            msg = Bool()
            msg.data = capture
            self.camera_pub.publish(msg)
            
            rospy.loginfo("相机指令: capture={}".format(capture))
            
        except json.JSONDecodeError:
            rospy.logerr("相机指令格式错误: {}".format(payload))
    
    def _handle_emergency_command(self, payload):
        """处理紧急停止指令"""
        try:
            data = json.loads(payload)
            emergency = data.get('emergency', True)
            
            msg = Bool()
            msg.data = emergency
            self.emergency_pub.publish(msg)
            
            rospy.logwarn("紧急停止指令: emergency={}".format(emergency))
            
        except json.JSONDecodeError:
            rospy.logerr("紧急停止指令格式错误: {}".format(payload))
    
    def _handle_task_command(self, payload):
        """处理任务指令"""
        try:
            # 直接转发任务指令到云端任务接收器
            msg = String()
            msg.data = payload
            self.cloud_task_pub.publish(msg)
            
            rospy.loginfo("任务指令已转发到云端任务接收器")
            
        except Exception as e:
            rospy.logerr("任务指令处理错误: {}".format(e))

    def _handle_status_request(self, payload):
        """处理状态请求"""
        try:
            data = json.loads(payload)
            request_type = data.get('type', 'full')
            
            # 这里可以根据请求类型返回不同的状态信息
            rospy.loginfo("状态请求: type={}".format(request_type))
            
        except json.JSONDecodeError:
            rospy.logerr("状态请求格式错误: {}".format(payload))
    
    def _handle_image_request(self, payload):
        """处理图像请求"""
        try:
            data = json.loads(payload)
            image_type = data.get('type', 'rgb')
            
            # 触发图像捕获
            msg = Bool()
            msg.data = True
            self.camera_pub.publish(msg)
            
            rospy.loginfo("图像请求: type={}".format(image_type))
            
        except json.JSONDecodeError:
            rospy.logerr("图像请求格式错误: {}".format(payload))
    
    def _handle_broadcast_message(self, payload):
        """处理广播消息"""
        try:
            data = json.loads(payload)
            msg_type = data.get('type')
            
            if msg_type == 'system_update':
                rospy.loginfo("系统更新广播: {}".format(data.get('message')))
            elif msg_type == 'maintenance':
                rospy.logwarn("维护通知: {}".format(data.get('message')))
            else:
                rospy.loginfo("广播消息: {}".format(payload))
                
        except json.JSONDecodeError:
            rospy.logerr("广播消息格式错误: {}".format(payload))
    
    # ================== ROS消息处理 ==================
    
    def _robot_status_callback(self, msg):
        """机器人状态回调"""
        try:
            # 使用配置的主题转发到云端
            topic = self._get_topic('reports', 'status')
            self._publish_to_cloud(topic, msg.data)
            
        except Exception as e:
            rospy.logerr("状态上报错误: {}".format(e))

    def _control_feedback_callback(self, msg):
        """控制反馈回调"""
        try:
            topic = self._get_topic('reports', 'feedback')
            self._publish_to_cloud(topic, msg.data)
            
        except Exception as e:
            rospy.logerr("反馈上报错误: {}".format(e))
    
    def _cloud_report_callback(self, msg):
        """云端报告回调"""
        try:
            # 转发到云端
            topic = self._get_topic('reports', 'feedback')
            self._publish_to_cloud(topic, msg.data)
            
        except Exception as e:
            rospy.logerr("云端报告错误: {}".format(e))
    
    def _telemetry_callback(self, msg):
        """遥测数据回调"""
        try:
            # 转发到云端
            topic = self._get_topic('reports', 'telemetry')
            self._publish_to_cloud(topic, msg.data)
            
        except Exception as e:
            rospy.logerr("遥测数据错误: {}".format(e))

    def _task_execution_status_callback(self, msg):
        """任务执行状态回调"""
        try:
            topic = self._get_topic('reports', 'task_status')
            self._publish_to_cloud(topic, msg.data)
            
        except Exception as e:
            rospy.logerr("任务执行状态上报错误: {}".format(e))

    def _task_feedback_callback(self, msg):
        """任务反馈回调"""
        try:
            topic = self._get_topic('reports', 'task_feedback')
            self._publish_to_cloud(topic, msg.data)
            
        except Exception as e:
            rospy.logerr("任务反馈上报错误: {}".format(e))
    
    def _task_status_callback(self, msg):
        """任务状态回调"""
        try:
            topic = self._get_topic('reports', 'task_status')
            self._publish_to_cloud(topic, msg.data)
            
        except Exception as e:
            rospy.logerr("任务状态上报错误: {}".format(e))
    
    def _image_callback(self, msg):
        """图像数据回调"""
        # 这里可以实现图像压缩和传输
        # 通常图像数据较大，建议按需传输
        pass
    
    # ================== 后台线程 ==================
    
    def _heartbeat_loop(self):
        """心跳循环"""
        rate = rospy.Rate(0.5)  # 每2秒发送一次心跳
        while not rospy.is_shutdown():
            try:
                if self.mqtt_connected:
                    heartbeat = {
                        "timestamp": time.time(),
                        "robot_id": self.robot_id,
                        "status": "alive",
                        "uptime": time.time() - self.last_heartbeat
                    }
                    
                    topic = self._get_topic('reports', 'heartbeat')
                    self._publish_to_cloud(topic, json.dumps(heartbeat))
                    
                    self.last_heartbeat = time.time()
                    
            except Exception as e:
                rospy.logerr("心跳发送错误: {}".format(e))
            
            rate.sleep()
    
    def _connection_monitor_loop(self):
        """连接监控循环"""
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            try:
                # 检查MQTT连接状态
                if not self.mqtt_connected:
                    rospy.logwarn("MQTT连接断开，尝试重连...")
                    try:
                        self.mqtt_client.reconnect()
                    except Exception as e:
                        rospy.logerr("MQTT重连失败: {}".format(e))
                
                # 发布连接状态
                status = "connected" if self.mqtt_connected else "disconnected"
                self._publish_mqtt_status(status)
                
            except Exception as e:
                rospy.logerr("连接监控错误: {}".format(e))
            
            rate.sleep()
    
    # ================== 辅助方法 ==================
    
    def _publish_to_cloud(self, topic, payload):
        """发布消息到云端"""
        if self.mqtt_connected:
            with self.publish_lock:
                try:
                    self.mqtt_client.publish(topic, payload, qos=0)
                except Exception as e:
                    rospy.logerr("MQTT发布错误: {}".format(e))
        else:
            rospy.logwarn("MQTT未连接，无法发布消息")
    
    def _publish_mqtt_status(self, status):
        """发布MQTT状态"""
        status_msg = {
            "timestamp": time.time(),
            "status": status,
            "broker": self.mqtt_broker,
            "port": self.mqtt_port
        }
        
        msg = String()
        msg.data = json.dumps(status_msg)
        self.mqtt_status_pub.publish(msg)

if __name__ == '__main__':
    try:
        bridge = MQTTBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("MQTT桥接器已停止")