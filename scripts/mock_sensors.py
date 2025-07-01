#!/usr/bin/env python
import rospy
import random
import math
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image, BatteryState
from tf.transformations import quaternion_from_euler

class MockSensors:
    """模拟传感器数据发布器 - 用于测试"""
    
    def __init__(self):
        rospy.init_node('mock_sensors')
        
        # 发布器
        self.pose_pub = rospy.Publisher('/amcl_pose', PoseWithCovarianceStamped, queue_size=1)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        self.battery_pub = rospy.Publisher('/battery_state', BatteryState, queue_size=1)
        
        # 模拟状态
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.battery_level = 100.0
        
        # 启动定时器
        rospy.Timer(rospy.Duration(0.1), self.publish_odom)  # 10Hz
        rospy.Timer(rospy.Duration(0.1), self.publish_scan)  # 10Hz
        rospy.Timer(rospy.Duration(0.2), self.publish_pose)  # 5Hz
        rospy.Timer(rospy.Duration(1.0), self.publish_battery)  # 1Hz
        rospy.Timer(rospy.Duration(0.5), self.publish_image)  # 2Hz
        
        rospy.loginfo("模拟传感器已启动")
    
    def publish_pose(self, event):
        """发布位姿数据"""
        msg = PoseWithCovarianceStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        
        # 添加少量随机噪声
        noise_x = random.uniform(-0.01, 0.01)
        noise_y = random.uniform(-0.01, 0.01)
        noise_theta = random.uniform(-0.01, 0.01)
        
        msg.pose.pose.position.x = self.robot_x + noise_x
        msg.pose.pose.position.y = self.robot_y + noise_y
        
        q = quaternion_from_euler(0, 0, self.robot_theta + noise_theta)
        msg.pose.pose.orientation = Quaternion(*q)
        
        # 设置协方差矩阵
        msg.pose.covariance = [0.1] * 36
        
        self.pose_pub.publish(msg)
    
    def publish_odom(self, event):
        """发布里程计数据"""
        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        
        # 模拟运动
        dt = 0.1
        linear_vel = random.uniform(-0.1, 0.1)
        angular_vel = random.uniform(-0.1, 0.1)
        
        self.robot_x += linear_vel * math.cos(self.robot_theta) * dt
        self.robot_y += linear_vel * math.sin(self.robot_theta) * dt
        self.robot_theta += angular_vel * dt
        
        msg.pose.pose.position.x = self.robot_x
        msg.pose.pose.position.y = self.robot_y
        
        q = quaternion_from_euler(0, 0, self.robot_theta)
        msg.pose.pose.orientation = Quaternion(*q)
        
        msg.twist.twist.linear.x = linear_vel
        msg.twist.twist.angular.z = angular_vel
        
        self.odom_pub.publish(msg)
    
    def publish_scan(self, event):
        """发布激光雷达数据"""
        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "laser"
        
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = math.pi / 180.0  # 1度
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0
        
        # 生成模拟距离数据
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        msg.ranges = []
        
        for i in range(num_readings):
            # 模拟环境：基础距离 + 随机噪声
            base_range = 2.0 + random.uniform(-0.5, 0.5)
            # 模拟一些障碍物
            if i % 90 == 0:  # 每90度一个近距离障碍物
                base_range = 0.5
            msg.ranges.append(base_range)
        
        self.scan_pub.publish(msg)
    
    def publish_image(self, event):
        """发布图像数据"""
        msg = Image()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera"
        
        msg.height = 480
        msg.width = 640
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = msg.width * 3
        
        # 生成模拟图像数据（简单的渐变）
        msg.data = []
        for y in range(msg.height):
            for x in range(msg.width):
                msg.data.extend([x % 256, y % 256, (x + y) % 256])
        
        self.image_pub.publish(msg)
    
    def publish_battery(self, event):
        """发布电池状态"""
        msg = BatteryState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        
        # 模拟电池缓慢放电
        self.battery_level -= 0.01
        if self.battery_level < 0:
            self.battery_level = 100.0  # 重置
        
        msg.voltage = 12.0
        msg.current = -2.0
        msg.charge = self.battery_level
        msg.capacity = 100.0
        msg.design_capacity = 100.0
        msg.percentage = self.battery_level / 100.0
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        
        self.battery_pub.publish(msg)

if __name__ == '__main__':
    try:
        mock_sensors = MockSensors()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass