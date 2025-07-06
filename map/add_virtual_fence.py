#!/usr/bin/env python
# -*- coding: utf-8 -*-
# filepath: /home/jetson/transbot_ws/src/transbot_manager/map/add_virtual_fence.py

import cv2
import numpy as np
import yaml
import os

def add_rectangular_fence(map_file, x1, y1, x2, y2, output_name=None):
    """添加矩形电子围栏"""
    # 加载地图
    with open(map_file, 'r') as f:
        map_config = yaml.safe_load(f)
    
    map_dir = os.path.dirname(map_file)
    image_file = os.path.join(map_dir, map_config['image'])
    image = cv2.imread(image_file, cv2.IMREAD_GRAYSCALE)
    
    # 绘制围栏 (使用特殊值 127 表示虚拟障碍)
    cv2.rectangle(image, (x1, y1), (x2, y2), 127, 2)
    
    # 保存新地图
    if output_name is None:
        base_name = os.path.splitext(os.path.basename(map_file))[0]
        output_name = f"{base_name}_fenced"
    
    new_image_file = os.path.join(map_dir, f"{output_name}.pgm")
    new_yaml_file = os.path.join(map_dir, f"{output_name}.yaml")
    
    cv2.imwrite(new_image_file, image)
    
    # 更新yaml配置
    new_config = map_config.copy()
    new_config['image'] = f"{output_name}.pgm"
    
    with open(new_yaml_file, 'w') as f:
        yaml.dump(new_config, f)
    
    print(f"带围栏的地图已生成: {new_yaml_file}")
    return new_yaml_file

if __name__ == '__main__':
    # 示例：在house地图上添加围栏
    map_file = "~/transbot_ws/src/transbot_nav/maps/house.yaml"
    # 围栏坐标 (像素坐标)
    add_rectangular_fence(map_file, 100, 100, 300, 200, "house_with_fence")