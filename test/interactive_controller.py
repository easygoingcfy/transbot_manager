#!/usr/bin/env python
# -*- coding: utf-8 -*-
# filepath: /home/jetson/transbot_ws/src/transbot_manager/test/interactive_controller.py

import rospy
import threading
import sys
import termios
import tty
import json
import math
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult

# Python 2/3 兼容性处理
try:
    input = raw_input  # Python 2
except NameError:
    pass  # Python 3

# 使用自定义的四元数转换函数，避免tf依赖问题
def euler_from_quaternion_simple(x, y, z, w):
    """
    简单的四元数转欧拉角函数
    返回 (roll, pitch, yaw) 单位为弧度
    """
    # 计算yaw角 (绕Z轴旋转)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    # 计算pitch角 (绕Y轴旋转)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # 使用90度
    else:
        pitch = math.asin(sinp)
    
    # 计算roll角 (绕X轴旋转)  
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    return roll, pitch, yaw

class InteractiveController:
    """交互式小车控制节点"""
    
    def __init__(self):
        rospy.init_node('interactive_controller')
        
        # ===== 第一步：初始化所有数据变量 =====
        # 控制参数
        self.current_mode = "manual"  # 默认手动模式
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.running = True
        
        # 位置信息 - 地图坐标系（必须在订阅器之前初始化）
        self.map_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'timestamp': 0.0}
        # 速度信息 - 里程计坐标系
        self.current_velocity = {'linear_x': 0.0, 'angular_z': 0.0, 'timestamp': 0.0}
        
        # 导航状态
        self.target_position = None
        self.navigation_active = False
        self.last_position_time = 0
        
        # 预设导航点 - 地图坐标系 (相对于当前位置的偏移)
        self.preset_goals = {
            '1': {'x': 1.0, 'y': 0.0, 'name': '前方1米', 'relative': True},
            '2': {'x': 0.0, 'y': 1.0, 'name': '左侧1米', 'relative': True},
            '3': {'x': -1.0, 'y': 0.0, 'name': '后方1米', 'relative': True},
            '4': {'x': 0.0, 'y': -1.0, 'name': '右侧1米', 'relative': True},
            '5': {'x': 2.0, 'y': 2.0, 'name': '右前角', 'relative': True},
            '0': {'x': 0.0, 'y': 0.0, 'name': '地图原点', 'relative': False}
        }
        
        # 键盘输入设置
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # ===== 第二步：初始化发布器 =====
        self.mode_pub = rospy.Publisher('/remote_control/mode', String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.emergency_pub = rospy.Publisher('/remote_control/emergency_stop', Bool, queue_size=1)
        
        # ===== 第三步：初始化订阅器（在数据变量初始化后）=====
        # 等待一小段时间确保所有变量都已初始化
        rospy.sleep(0.1)
        
        # 订阅器设置 - 监听反馈
        rospy.Subscriber('/control_feedback', String, self.feedback_callback)
        rospy.Subscriber('/robot_status', String, self.status_callback)
        
        # 位置数据订阅 - 使用AMCL定位（地图坐标系）
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        # 速度数据订阅 - 使用里程计
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # 导航结果订阅
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.nav_result_callback)
        
        rospy.loginfo("交互式控制器已启动")
        self.print_instructions()
    
    def safe_print(self, message):
        """安全打印函数 - 确保输出格式正确"""
        # 清除当前行并移动到行首
        sys.stdout.write('\r\033[K')
        sys.stdout.flush()
        # 打印消息并换行
        print(message)
        sys.stdout.flush()
    
    def amcl_pose_callback(self, msg):
        """AMCL定位回调 - 获取机器人在地图中的位置"""
        try:
            # 更新地图坐标系中的位置
            self.map_position['x'] = msg.pose.pose.position.x
            self.map_position['y'] = msg.pose.pose.position.y
            self.map_position['timestamp'] = msg.header.stamp.to_sec()
            
            # 使用自定义函数计算在地图中的朝向角
            orientation = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion_simple(
                orientation.x, orientation.y, orientation.z, orientation.w)
            self.map_position['theta'] = math.degrees(yaw)
            
            # 定期显示位置（每5秒）
            current_time = rospy.Time.now().to_sec()
            if current_time - self.last_position_time > 5.0:
                self.last_position_time = current_time
                if self.navigation_active:
                    self.show_navigation_status()
                else:
                    # 非导航时也显示位置更新
                    self.safe_print("地图位置: ({:.3f}, {:.3f}, {:.1f}°)".format(
                        self.map_position['x'], self.map_position['y'], self.map_position['theta']))
        except Exception as e:
            rospy.logwarn("AMCL回调处理失败: {}".format(e))
    
    def odom_callback(self, msg):
        """里程计回调 - 获取当前运动速度"""
        try:
            # 仅获取速度信息用于状态显示
            self.current_velocity['linear_x'] = msg.twist.twist.linear.x
            self.current_velocity['angular_z'] = msg.twist.twist.angular.z
            self.current_velocity['timestamp'] = msg.header.stamp.to_sec()
        except Exception as e:
            rospy.logwarn("里程计回调处理失败: {}".format(e))
    
    def get_key(self):
        """获取键盘输入"""
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key
    
    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        """发布速度指令"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        
        if linear_x != 0 or angular_z != 0:
            self.safe_print("发送速度指令: linear={:.2f}, angular={:.2f}".format(linear_x, angular_z))
        else:
            self.safe_print("发送停止指令")
    
    def switch_mode(self, mode):
        """切换控制模式"""
        self.current_mode = mode
        self.mode_pub.publish(String(data=mode))
        self.safe_print("切换模式: {}".format(mode))
    
    def emergency_stop(self, stop=True):
        """紧急停止"""
        self.emergency_pub.publish(Bool(data=stop))
        if stop:
            self.safe_print("紧急停止激活！")
            self.navigation_active = False
            self.publish_twist(0.0, 0.0)  # 立即停止
        else:
            self.safe_print("紧急停止解除")
    
    def send_navigation_goal(self, x, y, theta=0.0, name="目标点"):
        """发送导航目标 - 地图坐标系"""
        # 先切换到自动模式
        self.switch_mode('auto')
        rospy.sleep(0.1)
        
        # 检查目标点合理性
        current_distance = math.sqrt((x - self.map_position['x'])**2 + (y - self.map_position['y'])**2)
        if current_distance > 10.0:
            self.safe_print("警告: 目标点距离过远 ({:.1f}m)".format(current_distance))
            return
        
        # 创建目标点 - 地图坐标系
        goal = PoseStamped()
        goal.header.frame_id = "map"  # 重要：使用地图坐标系
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        
        # 设置朝向
        theta_rad = math.radians(theta)
        goal.pose.orientation.z = math.sin(theta_rad / 2.0)
        goal.pose.orientation.w = math.cos(theta_rad / 2.0)
        
        # 发布目标
        self.goal_pub.publish(goal)
        
        # 更新状态
        self.target_position = {'x': x, 'y': y, 'theta': theta, 'name': name}
        self.navigation_active = True
        
        self.safe_print("")  # 空行分隔
        self.safe_print("[导航] 目标设置: {} -> ({:.2f}, {:.2f}, {:.1f}°)".format(name, x, y, theta))
        self.safe_print("当前位置: ({:.2f}, {:.2f}, {:.1f}°)".format(
            self.map_position['x'], self.map_position['y'], self.map_position['theta']))
        self.show_navigation_status()
    
    def show_current_position(self):
        """显示当前位置 - 使用地图坐标系"""
        pos = self.map_position
        vel = self.current_velocity
        
        self.safe_print("")
        self.safe_print("=== 机器人状态 ===")
        self.safe_print("地图位置: ({:.3f}, {:.3f})".format(pos['x'], pos['y']))
        self.safe_print("朝向角度: {:.1f}°".format(pos['theta']))
        self.safe_print("当前速度: 线速度={:.2f} m/s, 角速度={:.2f} rad/s".format(vel['linear_x'], vel['angular_z']))
        self.safe_print("控制模式: {}".format(self.current_mode))
        
        if self.navigation_active and self.target_position:
            distance = math.sqrt((self.target_position['x'] - pos['x'])**2 + 
                               (self.target_position['y'] - pos['y'])**2)
            self.safe_print("导航目标: {} -> 距离 {:.2f}m".format(self.target_position['name'], distance))
        
        # 数据时效性检查
        current_time = rospy.Time.now().to_sec()
        position_age = current_time - pos['timestamp']
        velocity_age = current_time - vel['timestamp']
        
        if position_age > 1.0:
            self.safe_print("警告: 位置数据过时 ({:.1f}s)".format(position_age))
        if velocity_age > 1.0:
            self.safe_print("警告: 速度数据过时 ({:.1f}s)".format(velocity_age))
            
        self.safe_print("==================")
    
    def show_navigation_status(self):
        """显示导航状态 - 使用地图坐标系"""
        if not self.navigation_active or not self.target_position:
            return
            
        pos = self.map_position
        target = self.target_position
        
        # 计算距离
        distance = math.sqrt((target['x'] - pos['x'])**2 + (target['y'] - pos['y'])**2)
        
        # 计算方向
        angle_to_target = math.degrees(math.atan2(target['y'] - pos['y'], target['x'] - pos['x']))
        angle_diff = angle_to_target - pos['theta']
        
        # 标准化角度差到[-180, 180]
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        self.safe_print("")
        self.safe_print("[导航状态]")
        self.safe_print("当前: ({:.2f}, {:.2f}, {:.1f}°)".format(pos['x'], pos['y'], pos['theta']))
        self.safe_print("目标: {} ({:.2f}, {:.2f}, {:.1f}°)".format(target['name'], target['x'], target['y'], target['theta']))
        self.safe_print("距离: {:.2f}m | 转向: {:.1f}°".format(distance, angle_diff))
        
        # 进度显示
        if distance < 0.3:
            self.safe_print("状态: 接近目标")
        elif distance < 1.0:
            self.safe_print("状态: 即将到达")
        else:
            self.safe_print("状态: 导航中")
    
    def navigation_menu(self):
        """导航菜单"""
        self.safe_print("")
        self.safe_print("=== 导航菜单 ===")
        self.safe_print("预设目标点:")
        for key, goal in self.preset_goals.items():
            if goal['relative']:
                # 相对位置 - 基于当前位置计算
                target_x = self.map_position['x'] + goal['x']
                target_y = self.map_position['y'] + goal['y']
                self.safe_print("  {} - {} -> ({:.1f}, {:.1f})".format(key, goal['name'], target_x, target_y))
            else:
                # 绝对位置
                self.safe_print("  {} - {} ({:.1f}, {:.1f})".format(key, goal['name'], goal['x'], goal['y']))
        self.safe_print("  c - 自定义坐标")
        self.safe_print("  x - 返回主控制")
        
        while True:
            try:
                choice = self.get_key()
                
                if choice in self.preset_goals:
                    goal = self.preset_goals[choice]
                    if goal['relative']:
                        # 相对位置转换为绝对位置
                        target_x = self.map_position['x'] + goal['x']
                        target_y = self.map_position['y'] + goal['y']
                    else:
                        target_x = goal['x']
                        target_y = goal['y']
                    
                    self.send_navigation_goal(target_x, target_y, name=goal['name'])
                    break
                elif choice == 'c' or choice == 'C':
                    self.custom_navigation_goal()
                    break
                elif choice == 'x' or choice == 'X':
                    self.safe_print("返回主控制")
                    break
                else:
                    self.safe_print("无效选择，请重新输入")
                    
            except KeyboardInterrupt:
                break
    
    def custom_navigation_goal(self):
        """自定义导航目标 - 地图坐标系"""
        self.safe_print("")
        self.safe_print("=== 自定义导航目标 ===")
        try:
            # 临时恢复终端设置以便输入
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            
            self.safe_print("当前地图位置: ({:.2f}, {:.2f}, {:.1f}°)".format(
                self.map_position['x'], self.map_position['y'], self.map_position['theta']))
            self.safe_print("输入目标位置:")
            
            # 修复：使用兼容的input函数
            x = float(input("目标X坐标: "))
            y = float(input("目标Y坐标: "))
            theta_input = input("目标朝向角度 (默认0): ")
            theta = float(theta_input) if theta_input else 0.0
            
            name = "自定义({:.1f},{:.1f})".format(x, y)
            self.send_navigation_goal(x, y, theta, name)
            
        except (ValueError, KeyboardInterrupt):
            self.safe_print("输入取消")
        except EOFError:
            self.safe_print("输入被中断")
        finally:
            # 恢复原始终端设置
            tty.setraw(sys.stdin.fileno())
    
    def cancel_navigation(self):
        """取消导航"""
        if self.navigation_active:
            # 发布取消导航指令
            from actionlib_msgs.msg import GoalID
            cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
            rospy.sleep(0.1)  # 等待发布器初始化
            cancel_pub.publish(GoalID())
            
            self.navigation_active = False
            self.target_position = None
            self.safe_print("")
            self.safe_print("[导航] 已取消当前导航任务")
            # 切换回手动模式
            self.switch_mode('manual')
        else:
            self.safe_print("")
            self.safe_print("[导航] 当前没有活动的导航任务")
    
    def nav_result_callback(self, msg):
        """导航结果回调"""
        try:
            if self.navigation_active:
                result = msg.status.status
                if result == 3:  # SUCCEEDED
                    self.safe_print("")
                    self.safe_print("[导航] 目标已到达！")
                    self.safe_print("最终位置: ({:.2f}, {:.2f})".format(self.map_position['x'], self.map_position['y']))
                    self.navigation_active = False
                    self.target_position = None
                    self.switch_mode('manual')  # 导航完成后切换回手动模式
                elif result == 4:  # ABORTED
                    self.safe_print("")
                    self.safe_print("[导航] 导航失败，目标无法到达")
                    self.safe_print("当前位置: ({:.2f}, {:.2f})".format(self.map_position['x'], self.map_position['y']))
                    self.navigation_active = False
                    self.target_position = None
                    self.switch_mode('manual')
                elif result == 5:  # REJECTED
                    self.safe_print("")
                    self.safe_print("[导航] 导航被拒绝")
                    self.navigation_active = False
                    self.target_position = None
                    self.switch_mode('manual')
        except Exception as e:
            rospy.logwarn("导航结果回调处理失败: {}".format(e))
    
    def feedback_callback(self, msg):
        """控制反馈回调"""
        try:
            feedback = json.loads(msg.data)
            self.safe_print("[反馈] {}".format(feedback.get('message', msg.data)))
        except:
            self.safe_print("[反馈] {}".format(msg.data))
    
    def status_callback(self, msg):
        """状态回调"""
        try:
            status = json.loads(msg.data)
            self.current_mode = status.get('control_mode', 'unknown')
        except:
            pass
    
    def print_instructions(self):
        """打印使用说明"""
        rospy.loginfo("使用说明：")
        rospy.loginfo("=== 模式控制 ===")
        rospy.loginfo("  m - 切换到手动模式")
        rospy.loginfo("  1 - 切换到自动模式") 
        rospy.loginfo("  p - 切换到暂停模式")
        rospy.loginfo("=== 运动控制 ===")
        rospy.loginfo("  w/s - 前进/后退")
        rospy.loginfo("  a/d - 左转/右转")
        rospy.loginfo("  空格 - 立即停止")
        rospy.loginfo("=== 导航控制 ===")
        rospy.loginfo("  g - 进入导航模式")
        rospy.loginfo("  n - 设置自定义导航点")
        rospy.loginfo("  c - 取消当前导航")
        rospy.loginfo("=== 其他 ===")
        rospy.loginfo("  i - 显示当前位置")
        rospy.loginfo("  h - 显示帮助")
        rospy.loginfo("  q - 退出")
    
    def run(self):
        """主运行循环"""
        self.safe_print("")
        self.safe_print("=== 交互式小车控制 ===")
        self.safe_print("当前模式: {}".format(self.current_mode))
        self.safe_print("当前速度: 线速度={:.2f} m/s, 角速度={:.2f} rad/s".format(self.linear_speed, self.angular_speed))
        self.safe_print("按任意键开始控制...")
        
        try:
            while self.running and not rospy.is_shutdown():
                key = self.get_key()
                
                # 模式控制
                if key == 'm' or key == 'M':
                    self.switch_mode('manual')
                elif key == '1':
                    self.switch_mode('auto')
                elif key == 'p' or key == 'P':
                    self.switch_mode('pause')
                
                # 运动控制 - 只在手动模式下工作
                elif key == 'w' or key == 'W':
                    if self.current_mode == 'manual':
                        self.publish_twist(self.linear_speed, 0.0)
                    else:
                        self.safe_print("需要切换到手动模式 (按m)")
                elif key == 's' or key == 'S':
                    if self.current_mode == 'manual':
                        self.publish_twist(-self.linear_speed, 0.0)
                    else:
                        self.safe_print("需要切换到手动模式 (按m)")
                elif key == 'a' or key == 'A':
                    if self.current_mode == 'manual':
                        self.publish_twist(0.0, self.angular_speed)
                    else:
                        self.safe_print("需要切换到手动模式 (按m)")
                elif key == 'd' or key == 'D':
                    if self.current_mode == 'manual':
                        self.publish_twist(0.0, -self.angular_speed)
                    else:
                        self.safe_print("需要切换到手动模式 (按m)")
                elif key == ' ':  # 空格键停止
                    self.publish_twist(0.0, 0.0)
                
                # 导航控制
                elif key == 'g' or key == 'G':
                    self.navigation_menu()
                elif key == 'n' or key == 'N':
                    self.custom_navigation_goal()
                elif key == 'c' or key == 'C':
                    self.cancel_navigation()
                
                # 信息显示
                elif key == 'i' or key == 'I':
                    self.show_current_position()
                
                # 紧急控制
                elif key == 'e' or key == 'E':
                    self.emergency_stop(True)
                elif key == 'r' or key == 'R':
                    self.emergency_stop(False)
                
                # 速度调节 - 方向键控制
                elif key == '\x1b':  # ESC序列（方向键前缀）
                    try:
                        tty.setraw(sys.stdin.fileno())
                        key2 = sys.stdin.read(1)
                        key3 = sys.stdin.read(1)
                    finally:
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
                    
                    if key2 == '[':
                        if key3 == 'C':  # 右箭头键 - 增加速度
                            self.linear_speed = min(1.0, self.linear_speed + 0.1)
                            self.angular_speed = min(2.0, self.angular_speed + 0.1)
                            self.safe_print("速度增加: 线速度={:.2f}, 角速度={:.2f}".format(self.linear_speed, self.angular_speed))
                        elif key3 == 'D':  # 左箭头键 - 减少速度
                            self.linear_speed = max(0.1, self.linear_speed - 0.1)
                            self.angular_speed = max(0.1, self.angular_speed - 0.1)
                            self.safe_print("速度减少: 线速度={:.2f}, 角速度={:.2f}".format(self.linear_speed, self.angular_speed))

                # 退出
                elif key == 'q' or key == 'Q':
                    self.safe_print("退出控制器...")
                    self.publish_twist(0.0, 0.0)  # 停止机器人
                    if self.navigation_active:
                        self.cancel_navigation()
                    self.running = False
                    break
                
                # 帮助信息
                elif key == 'h' or key == 'H':
                    self.print_help()
                
                # 未知按键
                else:
                    if ord(key) == 3:  # Ctrl+C
                        break
                    self.safe_print("未知按键: '{}' (按h查看帮助)".format(key))
                
        except KeyboardInterrupt:
            self.safe_print("收到中断信号，退出...")
        finally:
            self.cleanup()
    
    def print_help(self):
        """打印帮助信息"""
        self.safe_print("")
        self.safe_print("=== 控制说明 ===")
        self.safe_print("模式控制:")
        self.safe_print("  m - 手动模式    1 - 自动模式    p - 暂停模式")
        self.safe_print("运动控制: (仅手动模式)")
        self.safe_print("  w - 前进        s - 后退")
        self.safe_print("  a - 左转        d - 右转")
        self.safe_print("  空格 - 停止")
        self.safe_print("导航控制:")
        self.safe_print("  g - 导航菜单    n - 自定义目标   c - 取消导航")
        self.safe_print("信息显示:")
        self.safe_print("  i - 显示位置    h - 帮助")
        self.safe_print("安全控制:")
        self.safe_print("  e - 紧急停止    r - 解除紧急停止")
        self.safe_print("速度调节:")
        self.safe_print("  → - 增加速度    ← - 减少速度")
        self.safe_print("其他:")
        self.safe_print("  q - 退出")
        self.safe_print("当前状态: 模式={}, 速度={:.2f}/{:.2f}".format(
            self.current_mode, self.linear_speed, self.angular_speed))
        self.safe_print("当前位置: ({:.2f}, {:.2f}, {:.1f}°)".format(
            self.map_position['x'], self.map_position['y'], self.map_position['theta']))
        if self.navigation_active:
            self.safe_print("导航状态: 进行中 -> {}".format(self.target_position['name']))
        self.safe_print("================")
    
    def cleanup(self):
        """清理资源"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        self.publish_twist(0.0, 0.0)  # 确保停止
        if self.navigation_active:
            self.cancel_navigation()
        rospy.loginfo("交互式控制器已停止")

if __name__ == '__main__':
    try:
        controller = InteractiveController()
        controller.run()
    except rospy.ROSInterruptException:
        pass