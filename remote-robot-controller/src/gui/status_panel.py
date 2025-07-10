#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QLabel, QGroupBox, QProgressBar, QTextEdit,
                             QFrame, QPushButton)
from PyQt5.QtCore import Qt, pyqtSlot, QTimer
from PyQt5.QtGui import QFont, QColor, QPalette

class StatusPanel(QWidget):
    """状态面板 - 显示机器人状态信息"""
    
    def __init__(self):
        super().__init__()
        self.robot_status = {
            'mode': 'unknown',
            'system_status': 'offline',
            'position': {'x': 0, 'y': 0, 'z': 0},
            'velocity': {'linear': 0, 'angular': 0},
            'battery': 0,
            'charging': False,
            'connection': False
        }
        
        self.init_ui()
        
        # 状态更新定时器
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)  # 100ms更新一次显示
    
    def init_ui(self):
        """初始化用户界面"""
        layout = QVBoxLayout(self)
        
        # 连接状态组
        connection_group = self.create_connection_group()
        layout.addWidget(connection_group)
        
        # 机器人状态组
        robot_group = self.create_robot_status_group()
        layout.addWidget(robot_group)
        
        layout.addStretch()
    
    def create_connection_group(self):
        """创建连接状态组"""
        group = QGroupBox("连接状态")
        layout = QGridLayout(group)
        
        # MQTT连接状态
        layout.addWidget(QLabel("MQTT连接:"), 0, 0)
        self.mqtt_status_label = QLabel("离线")
        self.mqtt_status_label.setStyleSheet("color: red; font-weight: bold;")
        layout.addWidget(self.mqtt_status_label, 0, 1)
        
        return group
    
    def create_robot_status_group(self):
        """创建机器人状态组"""
        group = QGroupBox("机器人状态")
        layout = QGridLayout(group)
        
        # 运行模式
        layout.addWidget(QLabel("运行模式:"), 0, 0)
        self.mode_label = QLabel("未知")
        self.mode_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(self.mode_label, 0, 1)
        
        return group
    
    @pyqtSlot(dict)
    def update_status(self, status_data):
        """更新状态数据"""
        if isinstance(status_data, dict):
            for key, value in status_data.items():
                if key in self.robot_status:
                    self.robot_status[key] = value
    
    @pyqtSlot(bool)
    def update_connection_status(self, connected):
        """更新连接状态"""
        self.robot_status['connection'] = connected
    
    def update_display(self):
        """更新显示"""
        # 更新连接状态
        if self.robot_status['connection']:
            self.mqtt_status_label.setText("在线")
            self.mqtt_status_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.mqtt_status_label.setText("离线")
            self.mqtt_status_label.setStyleSheet("color: red; font-weight: bold;")
        
        # 更新机器人状态
        mode = self.robot_status.get('mode', 'unknown')
        self.mode_label.setText(mode.upper())