#!/usr/bin/env python
#coding=utf-8
import sys
import os
import rospy
import threading
import time
import json
import cv2
from cv_bridge import CvBridge
import os
from datetime import datetime
from robot_enums import ControlMode, SystemStatus
from std_msgs.msg import String, Header, Bool, Float32
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image, BatteryState

class EnhancedMotionController:
    """增强版机器人基础控制器 - 负责实时控制、状态监控、数据上报"""
    
    def __init__(self):
        rospy.init_node('enhanced_motion_controller')
        
        # 图像处理工具
        self.bridge = CvBridge()
        self.capture_lock = threading.Lock()
        
        # 控制状态
        self.control_mode = ControlMode.PAUSE  # auto, manual, pause, stop
        self.system_status = SystemStatus.NORMAL  # normal, warning, error
        self.task_layer_active = False  # 任务层是否激活
        
        # 状态锁
        self.mode_lock = threading.Lock()
        self.data_lock = threading.Lock()
        
        # 安全参数
        self.last_cmd_time = time.time()
        self.cmd_timeout = 60.0
        self.emergency_stop = False
        
        # 机器人状态数据
        self.robot_state = {
            'pose': None,
            'velocity': None,
            'battery': 100.0,
            'laser_scan': None,
            'camera_image': None,
            'system_health': 'healthy',
            'error_codes': [],
            'uptime': 0.0
        }
        
        self.start_time = time.time()
        
        # 初始化发布器
        self._setup_publishers()
        
        # 初始化订阅器
        self._setup_subscribers()
        
        # 启动后台线程
        self._start_threads()
        
        rospy.loginfo("增强版机器人控制器已启动")
    
    def _setup_publishers(self):
        """设置发布器"""
        # 基础控制
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        # 外设控制
        self.elevator_cmd_pub = rospy.Publisher('/elevator/position_cmd', Float32, queue_size=1)
        self.elevator_enable_pub = rospy.Publisher('/elevator/enable', Bool, queue_size=1)
        
        # 状态上报
        self.robot_status_pub = rospy.Publisher('/robot_status', String, queue_size=1)
        self.cloud_report_pub = rospy.Publisher('/cloud_report', String, queue_size=1)
        self.telemetry_pub = rospy.Publisher('/robot_telemetry', String, queue_size=1)
        
        # 控制反馈
        self.control_feedback_pub = rospy.Publisher('/control_feedback', String, queue_size=1)
    
    def _setup_subscribers(self):
        """设置订阅器"""
        # 外部控制指令
        rospy.Subscriber('/remote_control/mode', String, self._mode_callback)
        rospy.Subscriber('/remote_control/cmd_vel', Twist, self._remote_cmd_vel_callback)
        rospy.Subscriber('/remote_control/goal', PoseStamped, self._remote_goal_callback)
        rospy.Subscriber('/remote_control/elevator', Float32, self._remote_elevator_callback)
        rospy.Subscriber('/remote_control/camera', Bool, self._remote_camera_callback)
        rospy.Subscriber('/remote_control/emergency_stop', Bool, self._emergency_stop_callback)
        
        # 机器人状态数据
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self._pose_callback)
        rospy.Subscriber('/odom', Odometry, self._odom_callback)
        rospy.Subscriber('/scan', LaserScan, self._scan_callback)
        rospy.Subscriber('/camera/image_raw', Image, self._image_callback)
        rospy.Subscriber('/battery_state', BatteryState, self._battery_callback)
        
        # 任务层接口
        rospy.Subscriber('/task_layer/status', String, self._task_status_callback)
        rospy.Subscriber('/task_layer/control_request', String, self._task_control_callback)
    
    def _start_threads(self):
        """启动后台线程"""
        threads = [
            threading.Thread(target=self._status_publish_loop, name="StatusPublisher"),
            threading.Thread(target=self._safety_monitor_loop, name="SafetyMonitor"),
            threading.Thread(target=self._cloud_report_loop, name="CloudReporter"),
            threading.Thread(target=self._telemetry_loop, name="TelemetryLoop")
        ]
        
        for thread in threads:
            thread.daemon = True
            thread.start()
            rospy.loginfo("启动线程: {}".format(thread.name))
    
    # ================== 外部控制接口 ==================
    
    def _mode_callback(self, msg):
        """处理模式切换指令"""
        with self.mode_lock:
            try:
                new_mode = ControlMode(msg.data.lower())
                
                # 检查是否可以切换模式
                if self.task_layer_active and new_mode in [ControlMode.AUTO, ControlMode.MANUAL]:
                    rospy.logwarn("任务执行中，无法切换到自动/手动模式")
                    self._publish_control_feedback("模式切换失败：任务执行中")
                    return
                
                old_mode = self.control_mode
                self.control_mode = new_mode
                
                rospy.loginfo("控制模式切换: {} -> {}".format(old_mode.value, new_mode.value))
                
                # 模式切换处理
                if new_mode in [ControlMode.PAUSE, ControlMode.STOP]:
                    self.cmd_vel_pub.publish(Twist())  # 立即停止
                    
                self.last_cmd_time = time.time()
                self._publish_control_feedback("模式已切换到: {}".format(new_mode.value))
                
            except ValueError:
                rospy.logwarn("收到未知控制模式: {}".format(msg.data))
                self._publish_control_feedback("未知控制模式: {}".format(msg.data))
    
    def _remote_cmd_vel_callback(self, msg):
        """处理远程速度控制指令"""
        with self.mode_lock:
            if self.control_mode == ControlMode.MANUAL and not self.emergency_stop:
                self.cmd_vel_pub.publish(msg)
                self.last_cmd_time = time.time()
                self._publish_control_feedback("速度指令已执行")
            else:
                self._publish_control_feedback("速度指令被拒绝：当前模式 {}".format(self.control_mode.value))
    
    def _remote_goal_callback(self, msg):
        """处理远程导航目标"""
        with self.mode_lock:
            if self.control_mode == ControlMode.AUTO and not self.emergency_stop:
                self.goal_pub.publish(msg)
                self.last_cmd_time = time.time()
                self._publish_control_feedback("导航目标已设置")
            else:
                self._publish_control_feedback("导航指令被拒绝：当前模式 {}".format(self.control_mode.value))
    
    def _remote_elevator_callback(self, msg):
        """处理升降杆控制指令"""
        if not self.emergency_stop:
            self.elevator_enable_pub.publish(Bool(True))
            rospy.sleep(0.1)
            self.elevator_cmd_pub.publish(msg)
            self._publish_control_feedback("升降杆目标位置: {}".format(msg.data))
        else:
            self._publish_control_feedback("升降杆指令被拒绝：紧急停止状态")
    
    def _remote_camera_callback(self, msg):
        """处理相机拍照指令"""
        if msg.data:  # True表示触发拍照
            try:
                # 使用线程避免阻塞
                threading.Thread(
                    target=self._capture_and_save_image,
                    name="CameraCapture"
                ).start()
                
                self._publish_control_feedback("拍照指令已接收，正在处理...")
                
            except Exception as e:
                self._publish_control_feedback("拍照失败 - {}".format(str(e)))
                rospy.logerr("相机拍照错误: {}".format(e))
        else:
            self._publish_control_feedback("相机拍照指令已取消")

    def _capture_and_save_image(self):
        """捕获并保存图像"""
        with self.capture_lock:
            try:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_dir = "/tmp/robot_captures"
                
                # 确保目录存在
                if not os.path.exists(save_dir):
                    os.makedirs(save_dir)
                
                captured_images = {}
                
                # 捕获RGB图像
                try:
                    rgb_msg = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=3.0)
                    rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
                    rgb_path = os.path.join(save_dir, "rgb_{}.jpg".format(timestamp))
                    cv2.imwrite(rgb_path, rgb_image)
                    captured_images['rgb'] = rgb_path
                    rospy.loginfo("RGB图像已保存: {}".format(rgb_path))
                except rospy.ROSException:
                    rospy.logwarn("RGB图像捕获超时")
                except Exception as e:
                    rospy.logerr("RGB图像处理错误: {}".format(e))
                
                # 捕获深度图像
                try:
                    depth_msg = rospy.wait_for_message('/camera/depth/image_raw', Image, timeout=3.0)
                    depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
                    # 转换深度图为可视化图像
                    depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                    depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                    depth_path = os.path.join(save_dir, "depth_{}.jpg".format(timestamp))
                    cv2.imwrite(depth_path, depth_colored)
                    captured_images['depth'] = depth_path
                    rospy.loginfo("深度图像已保存: {}".format(depth_path))
                except rospy.ROSException:
                    rospy.logwarn("深度图像捕获超时")
                except Exception as e:
                    rospy.logerr("深度图像处理错误: {}".format(e))
                
                # 捕获红外图像
                try:
                    ir_msg = rospy.wait_for_message('/camera/ir/image_raw', Image, timeout=3.0)
                    ir_image = self.bridge.imgmsg_to_cv2(ir_msg, "mono8")
                    ir_path = os.path.join(save_dir, "ir_{}.jpg".format(timestamp))
                    cv2.imwrite(ir_path, ir_image)
                    captured_images['ir'] = ir_path
                    rospy.loginfo("红外图像已保存: {}".format(ir_path))
                except rospy.ROSException:
                    rospy.logwarn("红外图像捕获超时")
                except Exception as e:
                    rospy.logerr("红外图像处理错误: {}".format(e))
                
                # 更新机器人状态
                with self.data_lock:
                    self.robot_state['camera_image'] = {
                        'timestamp': rospy.Time.now().to_sec(),
                        'captured_images': captured_images,
                        'last_capture': True
                    }
                
                # 发送反馈
                if captured_images:
                    feedback_msg = "拍照成功 - 已保存: {}".format(", ".join(captured_images.keys()))
                    self._publish_control_feedback(feedback_msg)
                else:
                    self._publish_control_feedback("拍照失败 - 未能获取任何图像")
                    
            except Exception as e:
                self._publish_control_feedback("拍照处理失败 - {}".format(str(e)))
                rospy.logerr("图像捕获处理错误: {}".format(e))
    
    def _emergency_stop_callback(self, msg):
        """处理紧急停止指令"""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.cmd_vel_pub.publish(Twist())  # 立即停止
            self.elevator_enable_pub.publish(Bool(False))  # 停止升降杆
            self.system_status = SystemStatus.ERROR
            rospy.logwarn("紧急停止已激活")
            self._publish_control_feedback("紧急停止已激活")
        else:
            self.system_status = SystemStatus.NORMAL
            rospy.loginfo("紧急停止已解除")
            self._publish_control_feedback("紧急停止已解除")
    
    # ================== 状态数据回调 ==================
    
    def _pose_callback(self, msg):
        """位姿数据回调"""
        with self.data_lock:
            self.robot_state['pose'] = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z,
                'qx': msg.pose.pose.orientation.x,
                'qy': msg.pose.pose.orientation.y,
                'qz': msg.pose.pose.orientation.z,
                'qw': msg.pose.pose.orientation.w,
                'timestamp': msg.header.stamp.to_sec()
            }
    
    def _odom_callback(self, msg):
        """里程计数据回调"""
        with self.data_lock:
            self.robot_state['velocity'] = {
                'linear_x': msg.twist.twist.linear.x,
                'linear_y': msg.twist.twist.linear.y,
                'angular_z': msg.twist.twist.angular.z,
                'timestamp': msg.header.stamp.to_sec()
            }
    
    def _scan_callback(self, msg):
        """激光雷达数据回调"""
        with self.data_lock:
            # 只保存关键信息，避免数据过大
            ranges = msg.ranges
            self.robot_state['laser_scan'] = {
                'min_range': min([r for r in ranges if r > 0]),
                'front_distance': ranges[len(ranges)//2] if ranges else 0,
                'timestamp': msg.header.stamp.to_sec()
            }
    
    def _image_callback(self, msg):
        """相机图像回调"""
        with self.data_lock:
            # 只记录图像元信息
            self.robot_state['camera_image'] = {
                'width': msg.width,
                'height': msg.height,
                'encoding': msg.encoding,
                'timestamp': msg.header.stamp.to_sec()
            }
    
    def _battery_callback(self, msg):
        """电池状态回调"""
        with self.data_lock:
            self.robot_state['battery'] = msg.percentage * 100.0
    
    # ================== 任务层接口 ==================
    
    def _task_status_callback(self, msg):
        """任务层状态回调"""
        try:
            status_data = json.loads(msg.data)
            self.task_layer_active = status_data.get('active', False)
            
            if self.task_layer_active:
                # 任务执行时，切换到任务模式
                with self.mode_lock:
                    if self.control_mode != ControlMode.TASK:
                        rospy.loginfo("任务层激活，切换到任务模式")
                        self.control_mode = ControlMode.TASK
                        
        except json.JSONDecodeError:
            rospy.logwarn("任务状态数据格式错误")
    
    def _task_control_callback(self, msg):
        """处理任务层的控制请求"""
        try:
            request = json.loads(msg.data)
            control_type = request.get('type')
            
            if control_type == 'velocity':
                twist = Twist()
                twist.linear.x = request.get('linear_x', 0)
                twist.angular.z = request.get('angular_z', 0)
                self.cmd_vel_pub.publish(twist)
                
            elif control_type == 'goal':
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = request.get('x', 0)
                goal.pose.position.y = request.get('y', 0)
                # 设置orientation...
                self.goal_pub.publish(goal)
                
        except json.JSONDecodeError:
            rospy.logwarn("任务控制请求格式错误")
    
    # ================== 后台线程 ==================
    
    def _status_publish_loop(self):
        """状态发布循环"""
        rate = rospy.Rate(5)  # 5Hz
        while not rospy.is_shutdown():
            try:
                # 发布JSON格式的状态信息
                status_data = {
                    "timestamp": rospy.Time.now().to_sec(),
                    "node_name": "enhanced_motion_controller",
                    "control_mode": self.control_mode.value if hasattr(self.control_mode, 'value') else str(self.control_mode),
                    "system_status": self.system_status.value if hasattr(self.system_status, 'value') else str(self.system_status),
                    "emergency_stop": self.emergency_stop,
                    "task_active": self.task_layer_active
                }

                # 添加机器人状态数据
                with self.data_lock:
                    if self.robot_state['pose']:
                        status_data["position"] = {
                            "x": self.robot_state['pose']['x'],
                            "y": self.robot_state['pose']['y']
                        }
                    
                    if self.robot_state['velocity']:
                        status_data["velocity"] = {
                            "linear": self.robot_state['velocity']['linear_x'],
                            "angular": self.robot_state['velocity']['angular_z']
                        }
                    
                    status_data["battery"] = self.robot_state['battery']
                    status_data["uptime"] = self.robot_state['uptime']
                
                # 发布String格式的状态
                self.robot_status_pub.publish(json.dumps(status_data, ensure_ascii=False))
                
            except Exception as e:
                rospy.logerr("状态发布错误: {}".format(e))
            
            rate.sleep()
    
    def _safety_monitor_loop(self):
        """安全监控循环"""
        rate = rospy.Rate(10)  # 10Hz
        timeout_warned = False

        while not rospy.is_shutdown():
            try:
                with self.mode_lock:
                    # 指令超时检查
                    if self.control_mode == ControlMode.MANUAL:
                        if time.time() - self.last_cmd_time > self.cmd_timeout:
                            if not timeout_warned:  # 只在第一次超时时处理
                                self.cmd_vel_pub.publish(Twist())
                                # 修改：超时时切换到暂停模式
                                old_mode = self.control_mode
                                self.control_mode = ControlMode.PAUSE
                                rospy.logwarn("手动控制指令超时，切换到暂停模式: {} -> {}".format(old_mode.value, self.control_mode.value))
                                self._publish_control_feedback("指令超时，已切换到暂停模式")
                                timeout_warned = True
                                self.last_cmd_time = time.time()
                        else:
                            timeout_warned = False  # 有新指令时重置标志
                    
                    # 电池电量检查
                    if self.robot_state['battery'] < 20.0:
                        if self.system_status == SystemStatus.NORMAL:
                            self.system_status = SystemStatus.WARNING
                            battery_level = round(self.robot_state['battery'], 1)
                            rospy.logwarn("电池电量低: {}%".format(battery_level))
                    
                    # 传感器数据检查
                    current_time = time.time()
                    if self.robot_state['laser_scan']:
                        if current_time - self.robot_state['laser_scan']['timestamp'] > 2.0:
                            rospy.logwarn("激光雷达数据超时")
                    
                    # 更新运行时间
                    self.robot_state['uptime'] = current_time - self.start_time
                    
            except Exception as e:
                rospy.logerr("安全监控错误: {}".format(e))
            
            rate.sleep()
    
    def _cloud_report_loop(self):
        """云端上报循环"""
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            try:
                report = {
                    "timestamp": time.time(),
                    "robot_id": rospy.get_param('robot_id', 'transbot_001'),
                    "control_mode": self.control_mode.value,
                    "system_status": self.system_status.value,
                    "task_active": self.task_layer_active,
                    "emergency_stop": self.emergency_stop,
                    "robot_state": self.robot_state.copy()
                }
                
                self.cloud_report_pub.publish(json.dumps(report, ensure_ascii=False))
                
            except Exception as e:
                rospy.logerr("云端上报错误: {}".format(e))
            
            rate.sleep()
    
    def _telemetry_loop(self):
        """遥测数据循环"""
        rate = rospy.Rate(10)  # 10Hz 高频遥测
        while not rospy.is_shutdown():
            try:
                with self.data_lock:
                    telemetry = {
                        "timestamp": time.time(),
                        "pose": self.robot_state['pose'],
                        "velocity": self.robot_state['velocity'],
                        "battery": self.robot_state['battery'],
                        "mode": self.control_mode.value,
                        "emergency": self.emergency_stop
                    }
                
                self.telemetry_pub.publish(json.dumps(telemetry, ensure_ascii=False))
                
            except Exception as e:
                rospy.logerr("遥测数据错误: {}".format(e))
            
            rate.sleep()
    
    # ================== 辅助方法 ==================
    
    def _publish_control_feedback(self, message):
        """发布控制反馈"""
        feedback = {
            "timestamp": time.time(),
            "message": message,
            "mode": self.control_mode.value,
            "status": self.system_status.value
        }
        self.control_feedback_pub.publish(json.dumps(feedback, ensure_ascii=False))

if __name__ == '__main__':
    try:
        controller = EnhancedMotionController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("机器人控制器已停止")