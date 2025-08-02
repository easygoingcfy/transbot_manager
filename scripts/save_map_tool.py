#!/usr/bin/env python
#coding=utf-8

"""简单的地图保存工具"""

import rospy
from std_srvs.srv import Trigger
from std_msgs.msg import String
import sys

def save_map(map_name=None):
    """保存当前地图"""
    rospy.init_node('save_map_tool', anonymous=True)
    
    try:
        # 设置地图名称
        if map_name:
            rospy.set_param('/mapping_manager/map_name', map_name)
            print("设置地图名称: {}".format(map_name))
        
        # 调用保存服务
        rospy.wait_for_service('/mapping_manager/save_map', timeout=5)
        save_service = rospy.ServiceProxy('/mapping_manager/save_map', Trigger)
        
        print("正在保存地图...")
        response = save_service()
        
        if response.success:
            print("✓ {}".format(response.message))
        else:
            print("✗ {}".format(response.message))
            
    except Exception as e:
        print("✗ 保存失败: {}".format(e))
        print("提示: 请确保建图系统正在运行")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        save_map(sys.argv[1])
    else:
        save_map()
