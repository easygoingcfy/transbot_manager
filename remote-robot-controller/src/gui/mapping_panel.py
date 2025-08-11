#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
                             QPushButton, QLineEdit, QLabel, QTextEdit,
                             QProgressBar, QMessageBox, QInputDialog)
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont, QPalette

class MappingPanel(QWidget):
    """建图控制面板"""
    
    # 定义信号
    start_mapping_requested = pyqtSignal()
    save_mapping_requested = pyqtSignal(str)  # map_name
    set_map_name_requested = pyqtSignal(str)  # map_name
    
    def __init__(self):
        super().__init__()
        self.current_map_name = "new_map"
        self.mapping_status = "ready"
        self.init_ui()
        
    def init_ui(self):
        """初始化用户界面"""
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # 建图控制组
        control_group = self.create_control_group()
        layout.addWidget(control_group)
        
        # 地图名称组
        name_group = self.create_name_group()
        layout.addWidget(name_group)
        
        # 状态显示组
        status_group = self.create_status_group()
        layout.addWidget(status_group)
        
        layout.addStretch()
        
    def create_control_group(self):
        """创建建图控制组"""
        group = QGroupBox("建图控制")
        layout = QVBoxLayout(group)
        
        # 按钮布局
        button_layout = QHBoxLayout()
        
        # 开始建图按钮
        self.start_btn = QPushButton("开始建图")
        self.start_btn.setMinimumHeight(50)
        self.start_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
        """)
        self.start_btn.clicked.connect(self.on_start_mapping)
        button_layout.addWidget(self.start_btn)
        
        # 保存地图按钮
        self.save_btn = QPushButton("保存地图")
        self.save_btn.setMinimumHeight(50)
        self.save_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #1565C0;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
        """)
        self.save_btn.clicked.connect(self.on_save_mapping)
        self.save_btn.setEnabled(False)  # 初始状态禁用
        button_layout.addWidget(self.save_btn)
        
        layout.addLayout(button_layout)
        
        return group
    
    def create_name_group(self):
        """创建地图名称组"""
        group = QGroupBox("地图名称")
        layout = QVBoxLayout(group)
        
        # 名称输入布局
        name_layout = QHBoxLayout()
        
        # 地图名称输入框
        self.name_edit = QLineEdit()
        self.name_edit.setText(self.current_map_name)
        self.name_edit.setPlaceholderText("输入地图名称...")
        self.name_edit.setMinimumHeight(35)
        self.name_edit.textChanged.connect(self.on_name_changed)
        name_layout.addWidget(QLabel("名称:"))
        name_layout.addWidget(self.name_edit)
        
        # 设置名称按钮
        set_name_btn = QPushButton("设置名称")
        set_name_btn.setMinimumHeight(35)
        set_name_btn.clicked.connect(self.on_set_name)
        name_layout.addWidget(set_name_btn)
        
        layout.addLayout(name_layout)
        
        # 预设名称按钮
        preset_layout = QHBoxLayout()
        preset_names = ["map_office", "map_warehouse", "map_home", "map_floor1"]
        for name in preset_names:
            btn = QPushButton(name)
            btn.setMaximumHeight(30)
            btn.clicked.connect(lambda checked, n=name: self.set_preset_name(n))
            preset_layout.addWidget(btn)
        
        layout.addLayout(preset_layout)
        
        return group
    
    def create_status_group(self):
        """创建状态显示组"""
        group = QGroupBox("建图状态")
        layout = QVBoxLayout(group)
        
        # 状态显示
        self.status_label = QLabel("状态: 就绪")
        self.status_label.setStyleSheet("""
            QLabel {
                padding: 8px;
                border: 1px solid #ddd;
                border-radius: 4px;
                background-color: #f9f9f9;
                font-size: 12px;
            }
        """)
        layout.addWidget(self.status_label)
        
        # 进度条（用于保存地图时显示进度）
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)
        
        # 日志显示
        log_label = QLabel("操作日志:")
        layout.addWidget(log_label)
        
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(120)
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("""
            QTextEdit {
                font-family: 'Consolas', 'Monaco', monospace;
                font-size: 10px;
                background-color: #f5f5f5;
                border: 1px solid #ddd;
            }
        """)
        layout.addWidget(self.log_text)
        
        return group
    
    def on_start_mapping(self):
        """开始建图"""
        self.add_log("开始建图请求已发送")
        self.start_mapping_requested.emit()
        
    def on_save_mapping(self):
        """保存地图"""
        map_name = self.name_edit.text().strip()
        if not map_name:
            QMessageBox.warning(self, "警告", "请输入地图名称")
            return
            
        # 确认对话框
        reply = QMessageBox.question(
            self, 
            "确认保存", 
            f"确定要保存地图为 '{map_name}' 吗？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes
        )
        
        if reply == QMessageBox.Yes:
            self.add_log(f"保存地图请求已发送: {map_name}")
            self.save_mapping_requested.emit(map_name)
    
    def on_set_name(self):
        """设置地图名称"""
        map_name = self.name_edit.text().strip()
        if not map_name:
            QMessageBox.warning(self, "警告", "请输入地图名称")
            return
            
        self.current_map_name = map_name
        self.add_log(f"地图名称设置请求已发送: {map_name}")
        self.set_map_name_requested.emit(map_name)
    
    def on_name_changed(self, text):
        """名称输入框变化"""
        self.current_map_name = text.strip()
    
    def set_preset_name(self, name):
        """设置预设名称"""
        self.name_edit.setText(name)
        self.current_map_name = name
    
    @pyqtSlot(str)
    def update_mapping_status(self, status):
        """更新建图状态"""
        self.mapping_status = status
        
        status_text = {
            "ready": "就绪",
            "mapping": "建图中...",
            "saving": "保存中...",
            "saved": "已保存",
            "name_set": "名称已设置",
            "error": "错误"
        }.get(status, status)
        
        self.status_label.setText(f"状态: {status_text}")
        
        # 根据状态更新界面
        if status == "mapping":
            self.start_btn.setEnabled(False)
            self.save_btn.setEnabled(True)
            self.start_btn.setText("建图中...")
            self.add_log("建图已开始")
            
        elif status == "saving":
            self.save_btn.setEnabled(False)
            self.progress_bar.setVisible(True)
            self.progress_bar.setRange(0, 0)  # 无限进度条
            self.add_log("正在保存地图...")
            
        elif status == "saved":
            self.start_btn.setEnabled(True)
            self.save_btn.setEnabled(False)
            self.start_btn.setText("开始建图")
            self.progress_bar.setVisible(False)
            self.add_log("地图保存完成")
            
        elif status == "error":
            self.start_btn.setEnabled(True)
            self.save_btn.setEnabled(True if self.mapping_status == "mapping" else False)
            self.start_btn.setText("开始建图")
            self.progress_bar.setVisible(False)
            self.add_log("操作出现错误")
            
        elif status == "ready":
            self.start_btn.setEnabled(True)
            self.save_btn.setEnabled(False)
            self.start_btn.setText("开始建图")
            self.progress_bar.setVisible(False)
    
    @pyqtSlot(dict)
    def handle_mapping_response(self, response):
        """处理建图响应"""
        action = response.get('action', '')
        success = response.get('success', False)
        message = response.get('message', '')
        
        if success:
            self.add_log(f"✓ {action}: {message}")
        else:
            self.add_log(f"✗ {action}: {message}")
            QMessageBox.warning(self, "操作失败", f"{action} 失败:\n{message}")
    
    def add_log(self, message):
        """添加日志"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        
        # 自动滚动到底部
        cursor = self.log_text.textCursor()
        cursor.movePosition(cursor.End)
        self.log_text.setTextCursor(cursor)
    
    def set_connection_status(self, connected):
        """设置连接状态"""
        self.start_btn.setEnabled(connected)
        if connected:
            self.add_log("机器人已连接")
        else:
            self.add_log("机器人连接断开")
            self.update_mapping_status("ready")
