#!/usr/bin/env python
# -*- coding: utf-8 -*-
# filepath: /home/jetson/transbot_ws/src/transbot_manager/map/map_editor.py

import cv2
import numpy as np
import yaml
import os

class MapEditor:
    def __init__(self, map_file):
        self.map_file = map_file
        self.load_map()
        self.drawing = False
        self.mode = 'wall'  # 'wall', 'free', 'fence'
        
    def load_map(self):
        """加载地图"""
        with open(self.map_file, 'r') as f:
            self.map_config = yaml.safe_load(f)
        
        map_dir = os.path.dirname(self.map_file)
        image_file = os.path.join(map_dir, self.map_config['image'])
        self.original_image = cv2.imread(image_file, cv2.IMREAD_GRAYSCALE)
        self.edited_image = self.original_image.copy()
        
    def mouse_callback(self, event, x, y, flags, param):
        """鼠标回调函数"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing:
                self.draw_at_position(x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            
    def draw_at_position(self, x, y):
        """在指定位置绘制"""
        if self.mode == 'wall':
            color = 0  # 黑色 - 障碍物
        elif self.mode == 'free':
            color = 254  # 白色 - 自由空间
        elif self.mode == 'fence':
            color = 127  # 灰色 - 电子围栏
        else:
            color = 128  # 默认
            
        cv2.circle(self.edited_image, (x, y), 3, color, -1)
        
    def add_virtual_fence(self, points):
        """添加虚拟围栏"""
        # points: [(x1,y1), (x2,y2), ...]
        for i in range(len(points)):
            start = points[i]
            end = points[(i + 1) % len(points)]
            cv2.line(self.edited_image, start, end, 127, 2)
            
    def edit_map(self):
        """交互式编辑地图"""
        cv2.namedWindow('Map Editor')
        cv2.setMouseCallback('Map Editor', self.mouse_callback)
        
        print("地图编辑器使用说明:")
        print("  鼠标左键 - 绘制")
        print("  w - 切换到墙壁模式 (黑色)")
        print("  f - 切换到自由空间模式 (白色)")
        print("  e - 切换到电子围栏模式 (灰色)")
        print("  r - 重置为原始地图")
        print("  s - 保存修改")
        print("  ESC - 退出")
        
        while True:
            # 显示当前模式 - 修复：使用.format()而不是f-string
            display_image = self.edited_image.copy()
            mode_text = "Mode: {}".format(self.mode)
            cv2.putText(display_image, mode_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, 128, 2)
            
            cv2.imshow('Map Editor', display_image)
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27:  # ESC
                break
            elif key == ord('w'):
                self.mode = 'wall'
                print("切换到墙壁模式")
            elif key == ord('f'):
                self.mode = 'free'
                print("切换到自由空间模式")
            elif key == ord('e'):
                self.mode = 'fence'
                print("切换到电子围栏模式")
            elif key == ord('r'):
                self.edited_image = self.original_image.copy()
                print("重置为原始地图")
            elif key == ord('s'):
                self.save_map()
                
        cv2.destroyAllWindows()
        
    def save_map(self):
        """保存修改后的地图"""
        # 保存图像
        map_dir = os.path.dirname(self.map_file)
        base_name = os.path.splitext(os.path.basename(self.map_file))[0]
        
        # 修复：使用.format()而不是f-string
        new_image_file = os.path.join(map_dir, "{}_edited.pgm".format(base_name))
        new_yaml_file = os.path.join(map_dir, "{}_edited.yaml".format(base_name))
        
        cv2.imwrite(new_image_file, self.edited_image)
        
        # 更新yaml配置
        new_config = self.map_config.copy()
        new_config['image'] = "{}_edited.pgm".format(base_name)
        
        with open(new_yaml_file, 'w') as f:
            yaml.dump(new_config, f)
            
        print("地图已保存: {}".format(new_yaml_file))

if __name__ == '__main__':
    import sys
    if len(sys.argv) != 2:
        print("用法: python map_editor.py <map_yaml_file>")
        sys.exit(1)
    
    editor = MapEditor(sys.argv[1])
    editor.edit_map()