#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QLabel, QGroupBox, QPushButton, QListWidget,
                             QListWidgetItem, QTextEdit, QComboBox, QSpinBox,
                             QDoubleSpinBox, QCheckBox, QProgressBar)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QFont

class TaskPanel(QWidget):
    """任务面板 - 任务创建和管理"""
    
    # 定义信号
    task_created = pyqtSignal(dict)
    task_cancelled = pyqtSignal(str)
    task_paused = pyqtSignal(str)
    task_resumed = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.current_tasks = []
        self.task_counter = 0
        self.init_ui()
    
    def init_ui(self):
        """初始化用户界面"""
        layout = QVBoxLayout(self)
        
        # 任务创建组
        create_group = self.create_task_creation_group()
        layout.addWidget(create_group)
        
        # 任务列表组
        list_group = self.create_task_list_group()
        layout.addWidget(list_group)
        
        # 任务控制组
        control_group = self.create_task_control_group()
        layout.addWidget(control_group)
    
    def create_task_creation_group(self):
        """创建任务创建组"""
        group = QGroupBox("创建任务")
        layout = QGridLayout(group)
        
        # 任务类型选择
        layout.addWidget(QLabel("任务类型:"), 0, 0)
        self.task_type_combo = QComboBox()
        self.task_type_combo.addItems(["导航", "巡逻", "拍照", "配送", "检查", "自定义"])
        layout.addWidget(self.task_type_combo, 0, 1)
        
        # 任务名称
        layout.addWidget(QLabel("任务名称:"), 1, 0)
        self.task_name_edit = QTextEdit()
        self.task_name_edit.setMaximumHeight(30)
        layout.addWidget(self.task_name_edit, 1, 1)
        
        # 目标位置（用于导航任务）
        layout.addWidget(QLabel("目标X:"), 2, 0)
        self.target_x_spin = QDoubleSpinBox()
        self.target_x_spin.setRange(-100, 100)
        self.target_x_spin.setSuffix(" m")
        layout.addWidget(self.target_x_spin, 2, 1)
        
        layout.addWidget(QLabel("目标Y:"), 3, 0)
        self.target_y_spin = QDoubleSpinBox()
        self.target_y_spin.setRange(-100, 100)
        self.target_y_spin.setSuffix(" m")
        layout.addWidget(self.target_y_spin, 3, 1)
        
        # 优先级
        layout.addWidget(QLabel("优先级:"), 4, 0)
        self.priority_spin = QSpinBox()
        self.priority_spin.setRange(1, 10)
        self.priority_spin.setValue(5)
        layout.addWidget(self.priority_spin, 4, 1)
        
        # 创建任务按钮
        self.create_task_btn = QPushButton("创建任务")
        self.create_task_btn.clicked.connect(self.create_task)
        layout.addWidget(self.create_task_btn, 5, 0, 1, 2)
        
        return group
    
    def create_task_list_group(self):
        """创建任务列表组"""
        group = QGroupBox("任务列表")
        layout = QVBoxLayout(group)
        
        self.task_list = QListWidget()
        layout.addWidget(self.task_list)
        
        # 任务详情
        self.task_details = QTextEdit()
        self.task_details.setMaximumHeight(100)
        self.task_details.setReadOnly(True)
        layout.addWidget(self.task_details)
        
        # 连接信号
        self.task_list.itemClicked.connect(self.show_task_details)
        
        return group
    
    def create_task_control_group(self):
        """创建任务控制组"""
        group = QGroupBox("任务控制")
        layout = QHBoxLayout(group)
        
        # 控制按钮
        self.start_task_btn = QPushButton("开始任务")
        self.start_task_btn.clicked.connect(self.start_selected_task)
        layout.addWidget(self.start_task_btn)
        
        self.pause_task_btn = QPushButton("暂停任务")
        self.pause_task_btn.clicked.connect(self.pause_selected_task)
        layout.addWidget(self.pause_task_btn)
        
        self.cancel_task_btn = QPushButton("取消任务")
        self.cancel_task_btn.clicked.connect(self.cancel_selected_task)
        layout.addWidget(self.cancel_task_btn)
        
        layout.addStretch()
        
        # 清空列表按钮
        self.clear_list_btn = QPushButton("清空列表")
        self.clear_list_btn.clicked.connect(self.clear_task_list)
        layout.addWidget(self.clear_list_btn)
        
        return group
    
    def create_task(self):
        """创建新任务"""
        task_type = self.task_type_combo.currentText()
        task_name = self.task_name_edit.toPlainText().strip()
        
        if not task_name:
            task_name = f"{task_type}任务_{self.task_counter + 1}"
        
        # 创建任务数据
        task_data = {
            'id': f"task_{self.task_counter}",
            'name': task_name,
            'type': task_type.lower(),
            'priority': self.priority_spin.value(),
            'status': 'created',
            'target_x': self.target_x_spin.value(),
            'target_y': self.target_y_spin.value(),
            'created_time': QTimer().remainingTime()
        }
        
        # 添加到列表
        self.current_tasks.append(task_data)
        self.task_counter += 1
        
        # 更新UI
        self.update_task_list()
        
        # 清空输入
        self.task_name_edit.clear()
        
        # 发射信号
        self.task_created.emit(task_data)
    
    def update_task_list(self):
        """更新任务列表显示"""
        self.task_list.clear()
        
        for task in self.current_tasks:
            item_text = f"[{task['status'].upper()}] {task['name']} (优先级: {task['priority']})"
            item = QListWidgetItem(item_text)
            item.setData(Qt.UserRole, task)
            self.task_list.addItem(item)
    
    def show_task_details(self, item):
        """显示任务详情"""
        task = item.data(Qt.UserRole)
        if task:
            details = f"任务ID: {task['id']}\n"
            details += f"名称: {task['name']}\n"
            details += f"类型: {task['type']}\n"
            details += f"状态: {task['status']}\n"
            details += f"优先级: {task['priority']}\n"
            details += f"目标位置: ({task['target_x']:.2f}, {task['target_y']:.2f})"
            
            self.task_details.setText(details)
    
    def start_selected_task(self):
        """开始选中的任务"""
        current_item = self.task_list.currentItem()
        if current_item:
            task = current_item.data(Qt.UserRole)
            task['status'] = 'running'
            self.update_task_list()
            self.task_created.emit(task)  # 重新发送任务以开始执行
    
    def pause_selected_task(self):
        """暂停选中的任务"""
        current_item = self.task_list.currentItem()
        if current_item:
            task = current_item.data(Qt.UserRole)
            if task['status'] == 'running':
                task['status'] = 'paused'
                self.task_paused.emit(task['id'])
            elif task['status'] == 'paused':
                task['status'] = 'running'
                self.task_resumed.emit(task['id'])
            
            self.update_task_list()
    
    def cancel_selected_task(self):
        """取消选中的任务"""
        current_item = self.task_list.currentItem()
        if current_item:
            task = current_item.data(Qt.UserRole)
            task['status'] = 'cancelled'
            self.task_cancelled.emit(task['id'])
            self.update_task_list()
    
    def clear_task_list(self):
        """清空任务列表"""
        self.current_tasks.clear()
        self.task_list.clear()
        self.task_details.clear()
    
    def update_task_status(self, task_id, status):
        """更新任务状态"""
        for task in self.current_tasks:
            if task['id'] == task_id:
                task['status'] = status
                break
        self.update_task_list()
    
    def get_pending_tasks(self):
        """获取待执行的任务"""
        return [task for task in self.current_tasks if task['status'] in ['created', 'running']]