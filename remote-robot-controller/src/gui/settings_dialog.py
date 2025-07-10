#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QLabel, QLineEdit, QSpinBox, QPushButton,
                             QGroupBox, QTabWidget, QWidget, QCheckBox,
                             QComboBox)
from PyQt5.QtCore import Qt

class SettingsDialog(QDialog):
    """设置对话框"""
    
    def __init__(self, config_manager, parent=None):
        super().__init__(parent)
        self.config_manager = config_manager
        self.config = config_manager.get_config()
        
        self.setWindowTitle("设置")
        self.setModal(True)
        self.resize(500, 400)
        
        self.init_ui()
        self.load_current_settings()
    
    def init_ui(self):
        """初始化用户界面"""
        layout = QVBoxLayout(self)
        
        # 创建标签页
        tab_widget = QTabWidget()
        layout.addWidget(tab_widget)
        
        # MQTT设置标签页
        mqtt_tab = self.create_mqtt_tab()
        tab_widget.addTab(mqtt_tab, "MQTT设置")
        
        # 机器人设置标签页
        robot_tab = self.create_robot_tab()
        tab_widget.addTab(robot_tab, "机器人设置")
        
        # 按钮布局
        button_layout = QHBoxLayout()
        
        self.save_btn = QPushButton("保存")
        self.save_btn.clicked.connect(self.save_settings)
        button_layout.addWidget(self.save_btn)
        
        self.cancel_btn = QPushButton("取消")
        self.cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(self.cancel_btn)
        
        layout.addLayout(button_layout)
    
    def create_mqtt_tab(self):
        """创建MQTT设置标签页"""
        widget = QWidget()
        layout = QGridLayout(widget)
        
        # MQTT服务器设置
        layout.addWidget(QLabel("服务器地址:"), 0, 0)
        self.host_edit = QLineEdit()
        layout.addWidget(self.host_edit, 0, 1)
        
        layout.addWidget(QLabel("端口:"), 1, 0)
        self.port_spin = QSpinBox()
        self.port_spin.setRange(1, 65535)
        layout.addWidget(self.port_spin, 1, 1)
        
        layout.addWidget(QLabel("用户名:"), 2, 0)
        self.username_edit = QLineEdit()
        layout.addWidget(self.username_edit, 2, 1)
        
        layout.addWidget(QLabel("密码:"), 3, 0)
        self.password_edit = QLineEdit()
        self.password_edit.setEchoMode(QLineEdit.Password)
        layout.addWidget(self.password_edit, 3, 1)
        
        return widget
    
    def create_robot_tab(self):
        """创建机器人设置标签页"""
        widget = QWidget()
        layout = QGridLayout(widget)
        
        # 机器人基本设置
        layout.addWidget(QLabel("机器人ID:"), 0, 0)
        self.robot_id_edit = QLineEdit()
        layout.addWidget(self.robot_id_edit, 0, 1)
        
        layout.addWidget(QLabel("机器人名称:"), 1, 0)
        self.robot_name_edit = QLineEdit()
        layout.addWidget(self.robot_name_edit, 1, 1)
        
        return widget
    
    def load_current_settings(self):
        """加载当前设置"""
        # MQTT设置
        mqtt_config = self.config.get('mqtt', {})
        self.host_edit.setText(mqtt_config.get('host', 'localhost'))
        self.port_spin.setValue(mqtt_config.get('port', 1883))
        self.username_edit.setText(mqtt_config.get('username', ''))
        self.password_edit.setText(mqtt_config.get('password', ''))
        
        # 机器人设置
        robot_config = self.config.get('robot', {})
        self.robot_id_edit.setText(robot_config.get('id', 'transbot_001'))
        self.robot_name_edit.setText(robot_config.get('name', 'TransBot'))
    
    def save_settings(self):
        """保存设置"""
        # 更新MQTT配置
        self.config['mqtt']['host'] = self.host_edit.text()
        self.config['mqtt']['port'] = self.port_spin.value()
        self.config['mqtt']['username'] = self.username_edit.text()
        self.config['mqtt']['password'] = self.password_edit.text()
        
        # 更新机器人配置
        self.config['robot']['id'] = self.robot_id_edit.text()
        self.config['robot']['name'] = self.robot_name_edit.text()
        
        # 保存配置
        if self.config_manager.save_config(self.config):
            self.accept()
        else:
            # 显示错误消息
            pass