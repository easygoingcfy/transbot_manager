#!/usr/bin/env python
#coding=utf-8

"""Mapping Manager for Transbot - 简洁的建图管理节点"""

import rospy
from cartographer_ros_msgs.srv import FinishTrajectory, WriteState
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String, Bool
import subprocess
import os

class MappingManager:
    def __init__(self):
        # 获取参数
        self.map_dir = rospy.get_param("~map_dir", "/home/jetson/transbot_ws/src/transbot_nav/maps")
        self.default_map_name = rospy.get_param("~map_name", "new_map")
        
        # 确保地图目录存在
        if not os.path.exists(self.map_dir):
            os.makedirs(self.map_dir)
            rospy.loginfo("Created map directory: %s", self.map_dir)
        
        # 服务
        rospy.Service('~save_map', Trigger, self.handle_save_map)
        rospy.Service('~start_mapping', Trigger, self.handle_start_mapping)
        rospy.Service('~set_map_name', String, self.handle_set_map_name)
        
        # 发布器
        self.status_pub = rospy.Publisher('~mapping_status', String, queue_size=1, latch=True)
        
        # 状态
        self.mapping_active = False
        
        rospy.loginfo("Mapping Manager initialized. Map dir: %s", self.map_dir)
        self.status_pub.publish(String(data="ready"))

    def handle_start_mapping(self, req):
        """开始建图状态标记"""
        self.mapping_active = True
        self.status_pub.publish(String(data="mapping"))
        return TriggerResponse(success=True, message="Mapping started")

    def handle_set_map_name(self, req):
        """设置地图名称"""
        if req.data:
            rospy.set_param("~map_name", req.data)
            return TriggerResponse(success=True, message="Map name set to: {}".format(req.data))
        else:
            return TriggerResponse(success=False, message="Empty map name")

    def handle_save_map(self, req):
        """保存地图的主要接口"""
        try:
            # 获取当前地图名
            map_name = rospy.get_param("~map_name", self.default_map_name)
            
            rospy.loginfo("Saving map as: %s", map_name)
            self.status_pub.publish(String(data="saving"))
            
            # 检测建图算法类型
            map_type = self.detect_mapping_type()
            
            if map_type == "cartographer":
                result = self.save_cartographer_map(map_name)
            elif map_type == "gmapping":
                result = self.save_gmapping_map(map_name)
            else:
                return TriggerResponse(success=False, message="Unsupported mapping type: {}".format(map_type))
            
            if result:
                self.mapping_active = False
                self.status_pub.publish(String(data="saved"))
                return TriggerResponse(success=True, message="Map '{}' saved successfully".format(map_name))
            else:
                self.status_pub.publish(String(data="error"))
                return TriggerResponse(success=False, message="Failed to save map")
                
        except Exception as e:
            rospy.logerr("Save map error: %s", str(e))
            self.status_pub.publish(String(data="error"))
            return TriggerResponse(success=False, message=str(e))

    def detect_mapping_type(self):
        """检测当前使用的建图算法"""
        # 检查cartographer服务
        try:
            rospy.wait_for_service('/finish_trajectory', timeout=1)
            return "cartographer"
        except:
            pass
        
        # 检查gmapping话题
        topics = rospy.get_published_topics()
        for topic, msg_type in topics:
            if 'map' in topic and msg_type == 'nav_msgs/OccupancyGrid':
                return "gmapping"
        
        return "unknown"

    def save_cartographer_map(self, map_name):
        """保存Cartographer地图"""
        try:
            pbstream_path = os.path.join(self.map_dir, "{}.pbstream".format(map_name))
            map_filestem = os.path.join(self.map_dir, map_name)

            # 1. 结束建图轨迹
            rospy.wait_for_service('/finish_trajectory', timeout=10)
            finish_srv = rospy.ServiceProxy('/finish_trajectory', FinishTrajectory)
            finish_srv(0)
            rospy.loginfo("Trajectory finished")

            # 2. 保存pbstream文件
            rospy.wait_for_service('/write_state', timeout=10)
            write_srv = rospy.ServiceProxy('/write_state', WriteState)
            write_srv(pbstream_path)
            rospy.loginfo("State written to %s", pbstream_path)

            # 3. 转换为ROS地图格式
            cmd = [
                "rosrun", "cartographer_ros", "cartographer_pbstream_to_ros_map",
                "-pbstream_filename=" + pbstream_path,
                "-map_filestem=" + map_filestem
            ]
            subprocess.check_call(cmd)
            rospy.loginfo("Map generated: %s(.pgm/.yaml)", map_filestem)
            return True

        except Exception as e:
            rospy.logerr("Cartographer save failed: %s", str(e))
            return False

    def save_gmapping_map(self, map_name):
        """保存gmapping地图"""
        try:
            map_file = os.path.join(self.map_dir, map_name)
            cmd = ["rosrun", "map_server", "map_saver", "-f", map_file]
            subprocess.check_call(cmd)
            rospy.loginfo("Gmapping map saved: %s", map_file)
            return True
        except Exception as e:
            rospy.logerr("Gmapping save failed: %s", str(e))
            return False

if __name__ == "__main__":
    rospy.init_node('mapping_manager')
    MappingManager()
    rospy.spin()