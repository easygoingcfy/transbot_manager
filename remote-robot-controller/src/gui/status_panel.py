#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QLabel, QGroupBox, QProgressBar, QTextEdit,
                             QFrame, QPushButton)
from PyQt5.QtCore import Qt, pyqtSlot, QTimer
from PyQt5.QtGui import QFont, QColor, QPalette
import json

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
            'connection': False,
            'last_update': None
        }
        
        self.init_ui()
        
        # 状态更新定时器
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(500)  # 500ms更新一次显示
    
    def init_ui(self):
        """初始化用户界面"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)
        
        # 连接状态组
        connection_group = self.create_connection_group()
        layout.addWidget(connection_group)
        
        # 机器人状态组
        robot_group = self.create_robot_status_group()
        layout.addWidget(robot_group)
        
        # 位置信息组
        position_group = self.create_position_group()
        layout.addWidget(position_group)
        
        # 系统信息组
        system_group = self.create_system_group()
        layout.addWidget(system_group)
        
        # 状态详情
        details_group = self.create_details_group()
        layout.addWidget(details_group)
        
        layout.addStretch()
    
    def create_connection_group(self):
        """创建连接状态组"""
        group = QGroupBox("连接状态")
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 14px;
                border: 2px solid #E0E0E0;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 8px 0 8px;
            }
        """)
        layout = QGridLayout(group)
        layout.setSpacing(8)
        
        # MQTT连接状态
        layout.addWidget(QLabel("MQTT连接:"), 0, 0)
        self.mqtt_status_label = QLabel("离线")
        self.mqtt_status_label.setStyleSheet("color: red; font-weight: bold; font-size: 14px;")
        layout.addWidget(self.mqtt_status_label, 0, 1)
        
        # 机器人响应状态
        layout.addWidget(QLabel("机器人响应:"), 1, 0)
        self.robot_response_label = QLabel("无响应")
        self.robot_response_label.setStyleSheet("color: orange; font-weight: bold; font-size: 14px;")
        layout.addWidget(self.robot_response_label, 1, 1)
        
        return group
    
    def create_robot_status_group(self):
        """创建机器人状态组"""
        group = QGroupBox("机器人状态")
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 14px;
                border: 2px solid #E0E0E0;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 8px 0 8px;
            }
        """)
        layout = QGridLayout(group)
        layout.setSpacing(8)
        
        # 运行模式
        layout.addWidget(QLabel("运行模式:"), 0, 0)
        self.mode_label = QLabel("未知")
        self.mode_label.setStyleSheet("font-weight: bold; font-size: 14px; color: #2196F3;")
        layout.addWidget(self.mode_label, 0, 1)
        
        # 系统状态
        layout.addWidget(QLabel("系统状态:"), 1, 0)
        self.system_status_label = QLabel("离线")
        self.system_status_label.setStyleSheet("font-weight: bold; font-size: 14px; color: #FF5722;")
        layout.addWidget(self.system_status_label, 1, 1)
        
        return group
    
    def create_position_group(self):
        """创建位置信息组"""
        group = QGroupBox("位置信息")
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 14px;
                border: 2px solid #E0E0E0;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 8px 0 8px;
            }
        """)
        layout = QGridLayout(group)
        layout.setSpacing(8)
        
        # 坐标
        layout.addWidget(QLabel("X坐标:"), 0, 0)
        self.x_label = QLabel("0.00 m")
        self.x_label.setStyleSheet("font-family: 'Consolas', monospace; font-size: 13px;")
        layout.addWidget(self.x_label, 0, 1)
        
        layout.addWidget(QLabel("Y坐标:"), 1, 0)
        self.y_label = QLabel("0.00 m")
        self.y_label.setStyleSheet("font-family: 'Consolas', monospace; font-size: 13px;")
        layout.addWidget(self.y_label, 1, 1)
        
        # 速度
        layout.addWidget(QLabel("线速度:"), 0, 2)
        self.linear_vel_label = QLabel("0.00 m/s")
        self.linear_vel_label.setStyleSheet("font-family: 'Consolas', monospace; font-size: 13px;")
        layout.addWidget(self.linear_vel_label, 0, 3)
        
        layout.addWidget(QLabel("角速度:"), 1, 2)
        self.angular_vel_label = QLabel("0.00 rad/s")
        self.angular_vel_label.setStyleSheet("font-family: 'Consolas', monospace; font-size: 13px;")
        layout.addWidget(self.angular_vel_label, 1, 3)
        
        return group
    
    def create_system_group(self):
        """创建系统信息组"""
        group = QGroupBox("系统信息")
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 14px;
                border: 2px solid #E0E0E0;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 8px 0 8px;
            }
        """)
        layout = QGridLayout(group)
        layout.setSpacing(8)
        
        # 电池电量
        layout.addWidget(QLabel("电池电量:"), 0, 0)
        self.battery_progress = QProgressBar()
        self.battery_progress.setRange(0, 100)
        self.battery_progress.setValue(0)
        self.battery_progress.setStyleSheet("""
            QProgressBar {
                border: 2px solid #E0E0E0;
                border-radius: 5px;
                text-align: center;
                font-weight: bold;
            }
            QProgressBar::chunk {
                background-color: #4CAF50;
                border-radius: 3px;
            }
        """)
        layout.addWidget(self.battery_progress, 0, 1)
        
        # 充电状态
        layout.addWidget(QLabel("充电状态:"), 1, 0)
        self.charging_label = QLabel("未充电")
        self.charging_label.setStyleSheet("font-weight: bold; font-size: 13px;")
        layout.addWidget(self.charging_label, 1, 1)
        
        return group
    
    def create_details_group(self):
        """创建状态详情组"""
        group = QGroupBox("状态详情")
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 14px;
                border: 2px solid #E0E0E0;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 8px 0 8px;
            }
        """)
        layout = QVBoxLayout(group)
        
        self.details_text = QTextEdit()
        self.details_text.setMaximumHeight(120)
        self.details_text.setReadOnly(True)
        self.details_text.setStyleSheet("""
            QTextEdit {
                border: 1px solid #E0E0E0;
                border-radius: 5px;
                background-color: #FAFAFA;
                font-family: 'Consolas', monospace;
                font-size: 12px;
                padding: 8px;
            }
        """)
        layout.addWidget(self.details_text)
        
        return group
    
    @pyqtSlot(dict)
    def update_status(self, status_data):
        """更新状态数据"""
        print(f"[STATUS_PANEL] 收到状态更新: {status_data}")
        
        if isinstance(status_data, dict):
            # 更新最后更新时间
            import time
            self.robot_status['last_update'] = time.time()
            
            # 更新状态数据
            for key, value in status_data.items():
                if key in self.robot_status:
                    self.robot_status[key] = value
                    print(f"[STATUS_PANEL] 更新 {key} = {value}")
            
            # 立即更新显示
            self.update_display()
            
            # 更新详情显示
            self.update_details(status_data)
    
    @pyqtSlot(bool)
    def update_connection_status(self, connected):
        """更新连接状态"""
        print(f"[STATUS_PANEL] 连接状态更新: {connected}")
        self.robot_status['connection'] = connected
        
        # 如果断开连接，重置机器人响应状态
        if not connected:
            self.robot_status['last_update'] = None
    
    def update_display(self):
        """更新显示"""
        # 更新连接状态
        if self.robot_status['connection']:
            self.mqtt_status_label.setText("在线")
            self.mqtt_status_label.setStyleSheet("color: green; font-weight: bold; font-size: 14px;")
        else:
            self.mqtt_status_label.setText("离线")
            self.mqtt_status_label.setStyleSheet("color: red; font-weight: bold; font-size: 14px;")
        
        # 更新机器人响应状态
        import time
        if self.robot_status['last_update']:
            time_diff = time.time() - self.robot_status['last_update']
            if time_diff < 10:  # 10秒内有响应
                self.robot_response_label.setText("正常")
                self.robot_response_label.setStyleSheet("color: green; font-weight: bold; font-size: 14px;")
            else:
                self.robot_response_label.setText("超时")
                self.robot_response_label.setStyleSheet("color: orange; font-weight: bold; font-size: 14px;")
        else:
            self.robot_response_label.setText("无响应")
            self.robot_response_label.setStyleSheet("color: red; font-weight: bold; font-size: 14px;")
        
        # 更新机器人状态
        mode = self.robot_status.get('mode', 'unknown')
        self.mode_label.setText(mode.upper())
        
        system_status = self.robot_status.get('system_status', 'offline')
        self.system_status_label.setText(system_status.upper())
        if system_status == 'online':
            self.system_status_label.setStyleSheet("font-weight: bold; font-size: 14px; color: #4CAF50;")
        else:
            self.system_status_label.setStyleSheet("font-weight: bold; font-size: 14px; color: #FF5722;")
        
        # 更新位置信息
        position = self.robot_status.get('position', {'x': 0, 'y': 0, 'z': 0})
        self.x_label.setText(f"{position.get('x', 0):.2f} m")
        self.y_label.setText(f"{position.get('y', 0):.2f} m")
        
        # 更新速度信息
        velocity = self.robot_status.get('velocity', {'linear': 0, 'angular': 0})
        self.linear_vel_label.setText(f"{velocity.get('linear', 0):.2f} m/s")
        self.angular_vel_label.setText(f"{velocity.get('angular', 0):.2f} rad/s")
        
        # 更新电池信息
        battery = self.robot_status.get('battery', 0)
        self.battery_progress.setValue(int(battery))
        
        # 根据电量调整颜色
        if battery > 50:
            color = "#4CAF50"  # 绿色
        elif battery > 20:
            color = "#FF9800"  # 橙色
        else:
            color = "#F44336"  # 红色
            
        self.battery_progress.setStyleSheet(f"""
            QProgressBar {{
                border: 2px solid #E0E0E0;
                border-radius: 5px;
                text-align: center;
                font-weight: bold;
            }}
            QProgressBar::chunk {{
                background-color: {color};
                border-radius: 3px;
            }}
        """)
        
        # 更新充电状态
        charging = self.robot_status.get('charging', False)
        if charging:
            self.charging_label.setText("充电中")
            self.charging_label.setStyleSheet("font-weight: bold; font-size: 13px; color: #4CAF50;")
        else:
            self.charging_label.setText("未充电")
            self.charging_label.setStyleSheet("font-weight: bold; font-size: 13px; color: #757575;")
    
    def update_details(self, status_data):
        """更新状态详情"""
        try:
            # 格式化状态数据为可读文本
            details_text = "最新状态数据:\n"
            details_text += json.dumps(status_data, indent=2, ensure_ascii=False)
            
            self.details_text.setText(details_text)
            
            # 滚动到底部
            scrollbar = self.details_text.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
            
        except Exception as e:
            print(f"[STATUS_PANEL] 更新详情失败: {e}")