#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
from datetime import datetime
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QPushButton, QGroupBox, QCheckBox, QSpinBox,
                             QFileDialog, QMessageBox)
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QPixmap, QImage

class CameraPanel(QWidget):
    """相机面板 - 图像显示和拍照控制"""
    
    # 定义信号
    capture_requested = pyqtSignal()
    stream_toggle = pyqtSignal(bool)
    
    def __init__(self):
        super().__init__()
        self.current_image = None
        self.save_directory = os.path.expanduser("~/robot_images")
        self.init_ui()
    
    def init_ui(self):
        """初始化用户界面"""
        layout = QVBoxLayout(self)
        
        # 图像显示区域
        image_group = self.create_image_group()
        layout.addWidget(image_group)
        
        # 控制按钮区域
        control_group = self.create_control_group()
        layout.addWidget(control_group)
        
        # 设置区域
        settings_group = self.create_settings_group()
        layout.addWidget(settings_group)
    
    def create_image_group(self):
        """创建图像显示组"""
        group = QGroupBox("相机图像")
        layout = QVBoxLayout(group)
        
        # 图像显示标签
        self.image_label = QLabel()
        self.image_label.setMinimumSize(400, 300)
        self.image_label.setMaximumSize(800, 600)
        self.image_label.setStyleSheet("""
            QLabel {
                border: 2px solid gray;
                background-color: black;
                color: white;
                text-align: center;
            }
        """)
        self.image_label.setText("无图像\n点击拍照获取图像")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setScaledContents(True)
        
        layout.addWidget(self.image_label)
        
        # 图像信息
        self.image_info_label = QLabel("图像信息: 无")
        layout.addWidget(self.image_info_label)
        
        return group
    
    def create_control_group(self):
        """创建控制按钮组"""
        group = QGroupBox("相机控制")
        layout = QVBoxLayout(group)
        
        # 第一行按钮
        button_layout1 = QHBoxLayout()
        
        self.capture_btn = QPushButton("拍照")
        self.capture_btn.clicked.connect(self.capture_image)
        button_layout1.addWidget(self.capture_btn)
        
        self.save_btn = QPushButton("保存图像")
        self.save_btn.clicked.connect(self.save_image)
        self.save_btn.setEnabled(False)
        button_layout1.addWidget(self.save_btn)
        
        self.clear_btn = QPushButton("清除图像")
        self.clear_btn.clicked.connect(self.clear_image)
        button_layout1.addWidget(self.clear_btn)
        
        layout.addLayout(button_layout1)
        
        # 第二行按钮
        button_layout2 = QHBoxLayout()
        
        self.stream_checkbox = QCheckBox("视频流")
        self.stream_checkbox.toggled.connect(self.toggle_stream)
        button_layout2.addWidget(self.stream_checkbox)
        
        button_layout2.addStretch()
        
        self.refresh_btn = QPushButton("刷新")
        self.refresh_btn.clicked.connect(self.capture_image)
        button_layout2.addWidget(self.refresh_btn)
        
        layout.addLayout(button_layout2)
        
        return group
    
    def create_settings_group(self):
        """创建设置组"""
        group = QGroupBox("设置")
        layout = QVBoxLayout(group)
        
        # 保存目录设置
        dir_layout = QHBoxLayout()
        dir_layout.addWidget(QLabel("保存目录:"))
        
        self.dir_label = QLabel(self.save_directory)
        self.dir_label.setStyleSheet("border: 1px solid gray; padding: 2px;")
        dir_layout.addWidget(self.dir_label)
        
        self.browse_btn = QPushButton("浏览")
        self.browse_btn.clicked.connect(self.browse_directory)
        dir_layout.addWidget(self.browse_btn)
        
        layout.addLayout(dir_layout)
        
        # 图像质量设置
        quality_layout = QHBoxLayout()
        quality_layout.addWidget(QLabel("图像质量:"))
        
        self.quality_spinbox = QSpinBox()
        self.quality_spinbox.setRange(1, 100)
        self.quality_spinbox.setValue(90)
        self.quality_spinbox.setSuffix("%")
        quality_layout.addWidget(self.quality_spinbox)
        
        quality_layout.addStretch()
        layout.addLayout(quality_layout)
        
        return group
    
    def capture_image(self):
        """拍照"""
        self.capture_requested.emit()
    
    def toggle_stream(self, enabled):
        """切换视频流"""
        self.stream_toggle.emit(enabled)
    
    @pyqtSlot(object)
    def display_image(self, image):
        """显示图像"""
        if isinstance(image, QImage):
            self.current_image = image
            
            # 缩放图像以适应显示区域
            pixmap = QPixmap.fromImage(image)
            scaled_pixmap = pixmap.scaled(
                self.image_label.size(), 
                Qt.KeepAspectRatio, 
                Qt.SmoothTransformation
            )
            
            self.image_label.setPixmap(scaled_pixmap)
            
            # 更新图像信息
            info = f"图像信息: {image.width()}x{image.height()}, {image.format()}"
            self.image_info_label.setText(info)
            
            # 启用保存按钮
            self.save_btn.setEnabled(True)
        
        elif isinstance(image, bytes):
            # 处理字节数据
            qimage = QImage()
            if qimage.loadFromData(image):
                self.display_image(qimage)
    
    def save_image(self):
        """保存图像"""
        if self.current_image is None:
            QMessageBox.warning(self, "警告", "没有图像可保存")
            return
        
        try:
            # 确保保存目录存在
            os.makedirs(self.save_directory, exist_ok=True)
            
            # 生成文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"robot_image_{timestamp}.png"
            filepath = os.path.join(self.save_directory, filename)
            
            # 保存图像
            quality = self.quality_spinbox.value()
            success = self.current_image.save(filepath, "PNG", quality)
            
            if success:
                QMessageBox.information(self, "成功", f"图像已保存到:\n{filepath}")
            else:
                QMessageBox.critical(self, "错误", "保存图像失败")
                
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存图像时发生错误:\n{e}")
    
    def clear_image(self):
        """清除图像"""
        self.current_image = None
        self.image_label.clear()
        self.image_label.setText("无图像\n点击拍照获取图像")
        self.image_info_label.setText("图像信息: 无")
        self.save_btn.setEnabled(False)
    
    def browse_directory(self):
        """浏览保存目录"""
        directory = QFileDialog.getExistingDirectory(
            self, "选择保存目录", self.save_directory
        )
        
        if directory:
            self.save_directory = directory
            self.dir_label.setText(directory)
    
    def get_save_directory(self):
        """获取保存目录"""
        return self.save_directory
    
    def set_save_directory(self, directory):
        """设置保存目录"""
        if os.path.isdir(directory):
            self.save_directory = directory
            self.dir_label.setText(directory)
    
    def get_image_quality(self):
        """获取图像质量设置"""
        return self.quality_spinbox.value()