#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QLabel, QGroupBox, QPushButton, QComboBox,
                             QDoubleSpinBox, QLineEdit, QDialogButtonBox)
from PyQt5.QtCore import Qt

class TaskCreatorDialog(QDialog):
    """任务创建对话框"""
    
    def __init__(self, behavior_type, parent=None, edit_data=None):
        super().__init__(parent)
        self.behavior_type = behavior_type
        self.edit_data = edit_data  # 编辑现有任务数据
        self.task_data = None
        self.init_ui()
        
        # 如果是编辑模式，填充现有数据
        if self.edit_data:
            self.load_edit_data()
    
    def init_ui(self):
        """初始化UI"""
        title = f"编辑{self.behavior_type}任务" if self.edit_data else f"创建{self.behavior_type}任务"
        self.setWindowTitle(title)
        self.setModal(True)
        self.resize(400, 300)
        
        layout = QVBoxLayout(self)
        
        # 基本信息
        basic_group = QGroupBox("基本信息")
        basic_layout = QGridLayout(basic_group)
        
        basic_layout.addWidget(QLabel("任务名称(英文):"), 0, 0)
        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("task_name_example")
        basic_layout.addWidget(self.name_edit, 0, 1)
        
        layout.addWidget(basic_group)
        
        # 参数设置
        params_group = QGroupBox("参数设置")
        self.params_layout = QGridLayout(params_group)
        layout.addWidget(params_group)
        
        self.create_params_ui()
        
        # 按钮
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)
    
    def create_params_ui(self):
        """创建参数UI"""
        if self.behavior_type == "navigate":
            self.params_layout.addWidget(QLabel("目标X (m):"), 0, 0)
            self.x_spin = QDoubleSpinBox()
            self.x_spin.setRange(-10, 10)
            self.x_spin.setDecimals(2)
            self.x_spin.setValue(0.2)
            self.params_layout.addWidget(self.x_spin, 0, 1)
            
            self.params_layout.addWidget(QLabel("目标Y (m):"), 1, 0)
            self.y_spin = QDoubleSpinBox()
            self.y_spin.setRange(-10, 10)
            self.y_spin.setDecimals(2)
            self.y_spin.setValue(0.0)
            self.params_layout.addWidget(self.y_spin, 1, 1)
            
            self.params_layout.addWidget(QLabel("目标角度 (度):"), 2, 0)
            self.theta_spin = QDoubleSpinBox()
            self.theta_spin.setRange(-180, 180)
            self.theta_spin.setDecimals(1)
            self.theta_spin.setValue(0.0)
            self.params_layout.addWidget(self.theta_spin, 2, 1)
        
        elif self.behavior_type == "capture_image":
            self.params_layout.addWidget(QLabel("相机类型:"), 0, 0)
            self.camera_combo = QComboBox()
            self.camera_combo.addItems(["rgb", "depth", "ir"])
            self.params_layout.addWidget(self.camera_combo, 0, 1)
            
            self.params_layout.addWidget(QLabel("超时时间 (秒):"), 1, 0)
            self.timeout_spin = QDoubleSpinBox()
            self.timeout_spin.setRange(1.0, 10.0)
            self.timeout_spin.setValue(5.0)
            self.params_layout.addWidget(self.timeout_spin, 1, 1)
        
        elif self.behavior_type == "wait":
            self.params_layout.addWidget(QLabel("等待时间 (秒):"), 0, 0)
            self.duration_spin = QDoubleSpinBox()
            self.duration_spin.setRange(0.1, 60.0)
            self.duration_spin.setValue(3.0)
            self.params_layout.addWidget(self.duration_spin, 0, 1)
        
        elif self.behavior_type == "speak":
            self.params_layout.addWidget(QLabel("语音内容:"), 0, 0)
            self.text_edit = QLineEdit()
            self.text_edit.setText("Task completed")
            self.params_layout.addWidget(self.text_edit, 0, 1)
        
        elif self.behavior_type == "rotate":
            self.params_layout.addWidget(QLabel("旋转角度 (度):"), 0, 0)
            self.angle_spin = QDoubleSpinBox()
            self.angle_spin.setRange(-360, 360)
            self.angle_spin.setValue(90.0)
            self.params_layout.addWidget(self.angle_spin, 0, 1)
            
            self.params_layout.addWidget(QLabel("角速度 (度/秒):"), 1, 0)
            self.speed_spin = QDoubleSpinBox()
            self.speed_spin.setRange(5.0, 180.0)
            self.speed_spin.setValue(30.0)
            self.params_layout.addWidget(self.speed_spin, 1, 1)
    
    def load_edit_data(self):
        """加载编辑数据"""
        if not self.edit_data:
            return
        
        # 设置名称
        name = self.edit_data.get('name', '')
        self.name_edit.setText(name)
        
        # 设置参数
        params = self.edit_data.get('params', {})
        
        if self.behavior_type == "navigate":
            self.x_spin.setValue(params.get('x', 0.0))
            self.y_spin.setValue(params.get('y', 0.0))
            self.theta_spin.setValue(params.get('theta', 0.0))
        elif self.behavior_type == "capture_image":
            camera_type = params.get('camera_type', 'rgb')
            index = self.camera_combo.findText(camera_type)
            if index >= 0:
                self.camera_combo.setCurrentIndex(index)
            self.timeout_spin.setValue(params.get('timeout', 5.0))
        elif self.behavior_type == "wait":
            self.duration_spin.setValue(params.get('duration', 3.0))
        elif self.behavior_type == "speak":
            self.text_edit.setText(params.get('text', ''))
        elif self.behavior_type == "rotate":
            angle_deg = params.get('angle_delta', 1.57) * 180.0 / 3.14159
            speed_deg = params.get('angular_speed', 0.5) * 180.0 / 3.14159
            self.angle_spin.setValue(angle_deg)
            self.speed_spin.setValue(speed_deg)
    
    def accept(self):
        """确认创建/编辑"""
        name = self.name_edit.text().strip()
        if not name:
            name = f"{self.behavior_type}_task_{int(time.time())}"
        
        # 验证名称只包含英文、数字和下划线
        import re
        if not re.match(r'^[a-zA-Z0-9_]+$', name):
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.warning(self, "名称错误", "任务名称只能包含英文字母、数字和下划线")
            return
        
        task_data = {
            "task_id": self.edit_data.get('task_id', f"custom_{int(time.time())}") if self.edit_data else f"custom_{int(time.time())}",
            "type": "action",
            "name": name,
            "behavior": self.behavior_type,
            "params": {}
        }
        
        # 设置参数
        if self.behavior_type == "navigate":
            task_data["params"] = {
                "x": self.x_spin.value(),
                "y": self.y_spin.value(),
                "theta": self.theta_spin.value()
            }
        elif self.behavior_type == "capture_image":
            task_data["params"] = {
                "camera_type": self.camera_combo.currentText(),
                "timeout": self.timeout_spin.value()
            }
        elif self.behavior_type == "wait":
            task_data["params"] = {
                "duration": self.duration_spin.value()
            }
        elif self.behavior_type == "speak":
            task_data["params"] = {
                "text": self.text_edit.text()
            }
        elif self.behavior_type == "rotate":
            angle_rad = self.angle_spin.value() * 3.14159 / 180.0
            speed_rad = self.speed_spin.value() * 3.14159 / 180.0
            task_data["params"] = {
                "angle_delta": angle_rad,
                "angular_speed": speed_rad
            }
        
        self.task_data = task_data
        super().accept()
    
    def get_task_data(self):
        """获取任务数据"""
        return self.task_data