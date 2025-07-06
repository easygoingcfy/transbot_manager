#!/usr/bin/env python
# -*- coding: utf-8 -*-
# filepath: /home/jetson/transbot_ws/src/transbot_manager/map/map_viewer.py

import rospy
import cv2
import numpy as np
import yaml
import os
from nav_msgs.msg import OccupancyGrid

class MapViewer:
    def __init__(self, map_file):
        self.map_file = map_file
        self.load_map()
        
    def load_map(self):
        """加载地图文件"""
        # 读取yaml配置
        with open(self.map_file, 'r') as f:
            map_config = yaml.safe_load(f)
        
        # 获取图像文件路径
        map_dir = os.path.dirname(self.map_file)
        image_file = os.path.join(map_dir, map_config['image'])
        
        # 读取地图图像
        self.map_image = cv2.imread(image_file, cv2.IMREAD_GRAYSCALE)
        self.resolution = map_config['resolution']
        self.origin = map_config['origin']
        
        print("地图信息:")
        print(f"  分辨率: {self.resolution} m/pixel")
        print(f"  原点: {self.origin}")
        print(f"  尺寸: {self.map_image.shape}")
        
    def show_map(self):
        """显示地图"""
        # 创建彩色版本用于标注
        color_map = cv2.cvtColor(self.map_image, cv2.COLOR_GRAY2BGR)
        
        # 添加网格线（可选）
        self.add_grid(color_map)
        
        cv2.imshow('Map Viewer - Press ESC to exit', color_map)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
    def add_grid(self, image, grid_size=50):
        """添加网格线"""
        h, w = image.shape[:2]
        # 垂直线
        for x in range(0, w, grid_size):
            cv2.line(image, (x, 0), (x, h), (0, 255, 0), 1)
        # 水平线
        for y in range(0, h, grid_size):
            cv2.line(image, (0, y), (w, y), (0, 255, 0), 1)

if __name__ == '__main__':
    import sys
    if len(sys.argv) != 2:
        print("用法: python map_viewer.py <map_yaml_file>")
        sys.exit(1)
    
    viewer = MapViewer(sys.argv[1])
    viewer.show_map()