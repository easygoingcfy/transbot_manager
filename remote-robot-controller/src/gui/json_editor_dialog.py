#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QTextEdit,
                             QPushButton, QLabel, QDialogButtonBox, QMessageBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

class JsonEditorDialog(QDialog):
    """JSON编辑器对话框"""
    
    def __init__(self, task_data, parent=None):
        super().__init__(parent)
        self.original_data = task_data
        self.edited_data = None
        self.init_ui()
        self.load_data()
    
    def init_ui(self):
        """初始化UI"""
        self.setWindowTitle("任务JSON编辑器")
        self.setModal(True)
        self.resize(600, 500)
        
        layout = QVBoxLayout(self)
        
        # 说明标签
        info_label = QLabel("您可以在下方编辑任务的JSON数据。请确保JSON格式正确。")
        info_label.setStyleSheet("color: #666; margin: 5px;")
        layout.addWidget(info_label)
        
        # JSON编辑器
        self.json_editor = QTextEdit()
        self.json_editor.setFont(QFont("Consolas", 10))
        self.json_editor.textChanged.connect(self.on_text_changed)
        layout.addWidget(self.json_editor)
        
        # 状态标签
        self.status_label = QLabel("JSON格式: 正确")
        self.status_label.setStyleSheet("color: green; font-weight: bold;")
        layout.addWidget(self.status_label)
        
        # 按钮区域
        button_layout = QHBoxLayout()
        
        self.format_btn = QPushButton("格式化JSON")
        self.format_btn.clicked.connect(self.format_json)
        button_layout.addWidget(self.format_btn)
        
        self.validate_btn = QPushButton("验证JSON")
        self.validate_btn.clicked.connect(self.validate_json)
        button_layout.addWidget(self.validate_btn)
        
        self.reset_btn = QPushButton("重置")
        self.reset_btn.clicked.connect(self.reset_data)
        button_layout.addWidget(self.reset_btn)
        
        button_layout.addStretch()
        layout.addLayout(button_layout)
        
        # 对话框按钮
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)
        
        # 延迟验证定时器
        self.validation_timer = QTimer()
        self.validation_timer.setSingleShot(True)
        self.validation_timer.timeout.connect(self.validate_json_delayed)
    
    def load_data(self):
        """加载数据"""
        try:
            formatted_json = json.dumps(self.original_data, indent=2, ensure_ascii=False)
            self.json_editor.setText(formatted_json)
        except Exception as e:
            QMessageBox.critical(self, "数据加载失败", f"无法加载任务数据: {str(e)}")
    
    def on_text_changed(self):
        """文本变化时延迟验证"""
        self.validation_timer.start(500)  # 500ms后验证
    
    def validate_json_delayed(self):
        """延迟验证JSON"""
        self.validate_json(show_message=False)
    
    def validate_json(self, show_message=True):
        """验证JSON格式"""
        try:
            text = self.json_editor.toPlainText().strip()
            if not text:
                self.status_label.setText("JSON格式: 空内容")
                self.status_label.setStyleSheet("color: orange; font-weight: bold;")
                return False
            
            data = json.loads(text)
            
            # 基本字段验证
            required_fields = ["task_id", "type", "name"]
            missing_fields = [field for field in required_fields if field not in data]
            
            if missing_fields:
                self.status_label.setText(f"JSON格式: 缺少字段 {', '.join(missing_fields)}")
                self.status_label.setStyleSheet("color: red; font-weight: bold;")
                if show_message:
                    QMessageBox.warning(self, "验证失败", f"缺少必要字段: {', '.join(missing_fields)}")
                return False
            
            # 检查任务类型
            valid_types = ["action", "sequence", "parallel", "selector"]
            if data.get("type") not in valid_types:
                self.status_label.setText(f"JSON格式: 无效的类型 {data.get('type')}")
                self.status_label.setStyleSheet("color: red; font-weight: bold;")
                if show_message:
                    QMessageBox.warning(self, "验证失败", f"无效的任务类型: {data.get('type')}")
                return False
            
            # 如果是action类型，检查behavior字段
            if data.get("type") == "action" and "behavior" not in data:
                self.status_label.setText("JSON格式: action类型缺少behavior字段")
                self.status_label.setStyleSheet("color: red; font-weight: bold;")
                if show_message:
                    QMessageBox.warning(self, "验证失败", "action类型任务缺少behavior字段")
                return False
            
            self.status_label.setText("JSON格式: 正确")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            
            if show_message:
                QMessageBox.information(self, "验证成功", "JSON格式正确！")
            
            return True
            
        except json.JSONDecodeError as e:
            self.status_label.setText(f"JSON格式: 语法错误")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
            if show_message:
                QMessageBox.critical(self, "JSON格式错误", f"JSON语法错误: {str(e)}")
            return False
        except Exception as e:
            self.status_label.setText(f"JSON格式: 未知错误")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
            if show_message:
                QMessageBox.critical(self, "验证失败", f"验证时发生错误: {str(e)}")
            return False
    
    def format_json(self):
        """格式化JSON"""
        try:
            text = self.json_editor.toPlainText().strip()
            if not text:
                return
            
            data = json.loads(text)
            formatted = json.dumps(data, indent=2, ensure_ascii=False)
            self.json_editor.setText(formatted)
            
        except json.JSONDecodeError as e:
            QMessageBox.critical(self, "格式化失败", f"JSON格式错误，无法格式化: {str(e)}")
    
    def reset_data(self):
        """重置数据"""
        reply = QMessageBox.question(self, '确认重置', 
                                   '确定要重置到原始数据吗？所有修改将丢失。',
                                   QMessageBox.Yes | QMessageBox.No,
                                   QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.load_data()
    
    def accept(self):
        """确认修改"""
        if not self.validate_json():
            return
        
        try:
            text = self.json_editor.toPlainText().strip()
            self.edited_data = json.loads(text)
            super().accept()
        except Exception as e:
            QMessageBox.critical(self, "保存失败", f"无法保存数据: {str(e)}")
    
    def get_edited_data(self):
        """获取编辑后的数据"""
        return self.edited_data