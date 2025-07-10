#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QLabel, QGroupBox, QPushButton, QSlider, QSpinBox,
                             QDoubleSpinBox, QComboBox, QCheckBox, QButtonGroup,
                             QRadioButton, QFrame)
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QTimer
from PyQt5.QtGui import QFont, QPalette

from utils.constants import ROBOT_MODES, MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY

class ControlPanel(QWidget):
    """控制面板 - 机器人运动控制和模式切换"""
    
    # 定义信号
    mode_changed = pyqtSignal(str)
    velocity_changed = pyqtSignal(float, float)  # linear_x, angular_z
    navigation_requested = pyqtSignal(float, float, float)  # x, y, yaw
    elevator_changed = pyqtSignal(int)  # position
    emergency_stop_requested = pyqtSignal(bool)
    
    def __init__(self):
        super().__init__()
        
        # 当前状态
        self.current_mode = 'pause'
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.emergency_active = False
        self.connected = False
        
        # 控制定时器
        self.velocity_timer = QTimer()
        self.velocity_timer.timeout.connect(self.send_velocity)
        self.velocity_timer.start(100)  # 100ms发送一次速度命令
        
        self.init_ui()
    
    def init_ui(self):
        """初始化用户界面"""
        layout = QVBoxLayout(self)
        
        # 模式控制组
        mode_group = self.create_mode_group()
        layout.addWidget(mode_group)
        
        # 速度控制组
        velocity_group = self.create_velocity_group()
        layout.addWidget(velocity_group)
        
        # 方向控制组
        direction_group = self.create_direction_group()
        layout.addWidget(direction_group)
        
        # 导航控制组
        navigation_group = self.create_navigation_group()
        layout.addWidget(navigation_group)
        
        # 升降杆控制组
        elevator_group = self.create_elevator_group()
        layout.addWidget(elevator_group)
        
        # 紧急控制组
        emergency_group = self.create_emergency_group()
        layout.addWidget(emergency_group)
        
        layout.addStretch()
    
    def create_mode_group(self):
        """创建模式控制组"""
        group = QGroupBox("运行模式")
        layout = QVBoxLayout(group)
        
        # 模式选择按钮组
        self.mode_button_group = QButtonGroup()
        
        # 自动模式
        self.auto_radio = QRadioButton("自动模式")
        self.auto_radio.setToolTip("机器人自主执行任务")
        self.mode_button_group.addButton(self.auto_radio)
        layout.addWidget(self.auto_radio)
        
        # 手动模式
        self.manual_radio = QRadioButton("手动模式")
        self.manual_radio.setToolTip("手动控制机器人运动")
        self.mode_button_group.addButton(self.manual_radio)
        layout.addWidget(self.manual_radio)
        
        # 暂停模式
        self.pause_radio = QRadioButton("暂停模式")
        self.pause_radio.setToolTip("暂停所有运动")
        self.pause_radio.setChecked(True)  # 默认选中
        self.mode_button_group.addButton(self.pause_radio)
        layout.addWidget(self.pause_radio)
        
        # 连接信号
        self.auto_radio.toggled.connect(lambda checked: checked and self.set_mode('auto'))
        self.manual_radio.toggled.connect(lambda checked: checked and self.set_mode('manual'))
        self.pause_radio.toggled.connect(lambda checked: checked and self.set_mode('pause'))
        
        return group
    
    def create_velocity_group(self):
        """创建速度控制组"""
        group = QGroupBox("速度控制")
        layout = QGridLayout(group)
        
        # 线速度控制
        layout.addWidget(QLabel("线速度:"), 0, 0)
        
        self.linear_slider = QSlider(Qt.Horizontal)
        self.linear_slider.setRange(-100, 100)
        self.linear_slider.setValue(0)
        self.linear_slider.setTickPosition(QSlider.TicksBelow)
        self.linear_slider.setTickInterval(25)
        self.linear_slider.valueChanged.connect(self.on_linear_slider_changed)
        layout.addWidget(self.linear_slider, 0, 1)
        
        self.linear_spinbox = QDoubleSpinBox()
        self.linear_spinbox.setRange(-MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY)
        self.linear_spinbox.setSingleStep(0.1)
        self.linear_spinbox.setDecimals(2)
        self.linear_spinbox.setSuffix(" m/s")
        self.linear_spinbox.valueChanged.connect(self.on_linear_spinbox_changed)
        layout.addWidget(self.linear_spinbox, 0, 2)
        
        # 角速度控制
        layout.addWidget(QLabel("角速度:"), 1, 0)
        
        self.angular_slider = QSlider(Qt.Horizontal)
        self.angular_slider.setRange(-100, 100)
        self.angular_slider.setValue(0)
        self.angular_slider.setTickPosition(QSlider.TicksBelow)
        self.angular_slider.setTickInterval(25)
        self.angular_slider.valueChanged.connect(self.on_angular_slider_changed)
        layout.addWidget(self.angular_slider, 1, 1)
        
        self.angular_spinbox = QDoubleSpinBox()
        self.angular_spinbox.setRange(-MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)
        self.angular_spinbox.setSingleStep(0.1)
        self.angular_spinbox.setDecimals(2)
        self.angular_spinbox.setSuffix(" rad/s")
        self.angular_spinbox.valueChanged.connect(self.on_angular_spinbox_changed)
        layout.addWidget(self.angular_spinbox, 1, 2)
        
        # 停止按钮
        stop_btn = QPushButton("停止")
        stop_btn.clicked.connect(self.stop_movement)
        layout.addWidget(stop_btn, 2, 0, 1, 3)
        
        return group
    
    def create_direction_group(self):
        """创建方向控制组"""
        group = QGroupBox("方向控制")
        layout = QGridLayout(group)
        
        # 创建方向按钮
        self.forward_btn = QPushButton("前进")
        self.forward_btn.setMinimumHeight(40)
        self.forward_btn.pressed.connect(lambda: self.set_direction_velocity(0.3, 0))
        self.forward_btn.released.connect(self.stop_movement)
        layout.addWidget(self.forward_btn, 0, 1)
        
        self.left_btn = QPushButton("左转")
        self.left_btn.setMinimumHeight(40)
        self.left_btn.pressed.connect(lambda: self.set_direction_velocity(0, 0.5))
        self.left_btn.released.connect(self.stop_movement)
        layout.addWidget(self.left_btn, 1, 0)
        
        self.backward_btn = QPushButton("后退")
        self.backward_btn.setMinimumHeight(40)
        self.backward_btn.pressed.connect(lambda: self.set_direction_velocity(-0.3, 0))
        self.backward_btn.released.connect(self.stop_movement)
        layout.addWidget(self.backward_btn, 2, 1)
        
        self.right_btn = QPushButton("右转")
        self.right_btn.setMinimumHeight(40)
        self.right_btn.pressed.connect(lambda: self.set_direction_velocity(0, -0.5))
        self.right_btn.released.connect(self.stop_movement)
        layout.addWidget(self.right_btn, 1, 2)
        
        # 组合运动按钮
        self.forward_left_btn = QPushButton("前左")
        self.forward_left_btn.pressed.connect(lambda: self.set_direction_velocity(0.3, 0.3))
        self.forward_left_btn.released.connect(self.stop_movement)
        layout.addWidget(self.forward_left_btn, 0, 0)
        
        self.forward_right_btn = QPushButton("前右")
        self.forward_right_btn.pressed.connect(lambda: self.set_direction_velocity(0.3, -0.3))
        self.forward_right_btn.released.connect(self.stop_movement)
        layout.addWidget(self.forward_right_btn, 0, 2)
        
        self.backward_left_btn = QPushButton("后左")
        self.backward_left_btn.pressed.connect(lambda: self.set_direction_velocity(-0.3, 0.3))
        self.backward_left_btn.released.connect(self.stop_movement)
        layout.addWidget(self.backward_left_btn, 2, 0)
        
        self.backward_right_btn = QPushButton("后右")
        self.backward_right_btn.pressed.connect(lambda: self.set_direction_velocity(-0.3, -0.3))
        self.backward_right_btn.released.connect(self.stop_movement)
        layout.addWidget(self.backward_right_btn, 2, 2)
        
        return group
    
    def create_navigation_group(self):
        """创建导航控制组"""
        group = QGroupBox("导航控制")
        layout = QGridLayout(group)
        
        # 目标位置输入
        layout.addWidget(QLabel("目标X:"), 0, 0)
        self.target_x_spinbox = QDoubleSpinBox()
        self.target_x_spinbox.setRange(-100, 100)
        self.target_x_spinbox.setSingleStep(0.1)
        self.target_x_spinbox.setDecimals(2)
        self.target_x_spinbox.setSuffix(" m")
        layout.addWidget(self.target_x_spinbox, 0, 1)
        
        layout.addWidget(QLabel("目标Y:"), 1, 0)
        self.target_y_spinbox = QDoubleSpinBox()
        self.target_y_spinbox.setRange(-100, 100)
        self.target_y_spinbox.setSingleStep(0.1)
        self.target_y_spinbox.setDecimals(2)
        self.target_y_spinbox.setSuffix(" m")
        layout.addWidget(self.target_y_spinbox, 1, 1)
        
        layout.addWidget(QLabel("目标朝向:"), 2, 0)
        self.target_yaw_spinbox = QDoubleSpinBox()
        self.target_yaw_spinbox.setRange(-180, 180)
        self.target_yaw_spinbox.setSingleStep(1.0)
        self.target_yaw_spinbox.setDecimals(1)
        self.target_yaw_spinbox.setSuffix(" °")
        layout.addWidget(self.target_yaw_spinbox, 2, 1)
        
        # 导航按钮
        self.navigate_btn = QPushButton("开始导航")
        self.navigate_btn.clicked.connect(self.start_navigation)
        layout.addWidget(self.navigate_btn, 3, 0, 1, 2)
        
        # 取消导航按钮
        self.cancel_nav_btn = QPushButton("取消导航")
        self.cancel_nav_btn.clicked.connect(self.cancel_navigation)
        self.cancel_nav_btn.setEnabled(False)
        layout.addWidget(self.cancel_nav_btn, 4, 0, 1, 2)
        
        return group
    
    def create_elevator_group(self):
        """创建升降杆控制组"""
        group = QGroupBox("升降杆控制")
        layout = QVBoxLayout(group)
        
        # 位置选择
        position_layout = QHBoxLayout()
        position_layout.addWidget(QLabel("位置:"))
        
        self.elevator_slider = QSlider(Qt.Horizontal)
        self.elevator_slider.setRange(0, 100)
        self.elevator_slider.setValue(0)
        self.elevator_slider.setTickPosition(QSlider.TicksBelow)
        self.elevator_slider.setTickInterval(25)
        self.elevator_slider.valueChanged.connect(self.on_elevator_changed)
        position_layout.addWidget(self.elevator_slider)
        
        self.elevator_label = QLabel("0%")
        position_layout.addWidget(self.elevator_label)
        
        layout.addLayout(position_layout)
        
        # 预设位置按钮
        preset_layout = QHBoxLayout()
        
        bottom_btn = QPushButton("最低")
        bottom_btn.clicked.connect(lambda: self.set_elevator_position(0))
        preset_layout.addWidget(bottom_btn)
        
        middle_btn = QPushButton("中间")
        middle_btn.clicked.connect(lambda: self.set_elevator_position(50))
        preset_layout.addWidget(middle_btn)
        
        top_btn = QPushButton("最高")
        top_btn.clicked.connect(lambda: self.set_elevator_position(100))
        preset_layout.addWidget(top_btn)
        
        layout.addLayout(preset_layout)
        
        return group
    
    def create_emergency_group(self):
        """创建紧急控制组"""
        group = QGroupBox("紧急控制")
        layout = QVBoxLayout(group)
        
        # 紧急停止按钮
        self.emergency_btn = QPushButton("紧急停止")
        self.emergency_btn.setMinimumHeight(50)
        self.emergency_btn.setStyleSheet("""
            QPushButton {
                background-color: #ff4444;
                color: white;
                font-weight: bold;
                font-size: 14px;
                border-radius: 5px;
            }
            QPushButton:pressed {
                background-color: #cc0000;
            }
        """)
        self.emergency_btn.clicked.connect(self.toggle_emergency_stop)
        layout.addWidget(self.emergency_btn)
        
        # 解除紧急停止按钮
        self.release_emergency_btn = QPushButton("解除紧急停止")
        self.release_emergency_btn.setEnabled(False)
        self.release_emergency_btn.clicked.connect(self.release_emergency_stop)
        layout.addWidget(self.release_emergency_btn)
        
        return group
    
    # ================== 信号槽方法 ==================
    
    def on_linear_slider_changed(self, value):
        """线速度滑块变化"""
        # 将滑块值(-100到100)转换为实际速度值
        actual_value = (value / 100.0) * MAX_LINEAR_VELOCITY
        self.linear_spinbox.blockSignals(True)
        self.linear_spinbox.setValue(actual_value)
        self.linear_spinbox.blockSignals(False)
        self.linear_velocity = actual_value
    
    def on_linear_spinbox_changed(self, value):
        """线速度输入框变化"""
        # 将实际速度值转换为滑块值
        slider_value = int((value / MAX_LINEAR_VELOCITY) * 100)
        self.linear_slider.blockSignals(True)
        self.linear_slider.setValue(slider_value)
        self.linear_slider.blockSignals(False)
        self.linear_velocity = value
    
    def on_angular_slider_changed(self, value):
        """角速度滑块变化"""
        # 将滑块值(-100到100)转换为实际速度值
        actual_value = (value / 100.0) * MAX_ANGULAR_VELOCITY
        self.angular_spinbox.blockSignals(True)
        self.angular_spinbox.setValue(actual_value)
        self.angular_spinbox.blockSignals(False)
        self.angular_velocity = actual_value
    
    def on_angular_spinbox_changed(self, value):
        """角速度输入框变化"""
        # 将实际速度值转换为滑块值
        slider_value = int((value / MAX_ANGULAR_VELOCITY) * 100)
        self.angular_slider.blockSignals(True)
        self.angular_slider.setValue(slider_value)
        self.angular_slider.blockSignals(False)
        self.angular_velocity = value
    
    def on_elevator_changed(self, value):
        """升降杆位置变化"""
        self.elevator_label.setText(f"{value}%")
        self.elevator_changed.emit(value)
    
    # ================== 控制方法 ==================
    
    def set_mode(self, mode):
        """设置机器人模式"""
        if mode in ROBOT_MODES:
            self.current_mode = mode
            self.mode_changed.emit(mode)
            
            # 根据模式启用/禁用控件
            if mode == 'manual':
                self.enable_manual_controls(True)
            else:
                self.enable_manual_controls(False)
                if mode == 'pause':
                    self.stop_movement()
    
    def set_direction_velocity(self, linear, angular):
        """设置方向速度"""
        if self.current_mode == 'manual' and not self.emergency_active:
            self.linear_velocity = linear
            self.angular_velocity = angular
            self.update_velocity_controls()
    
    def stop_movement(self):
        """停止运动"""
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.update_velocity_controls()
    
    def send_velocity(self):
        """发送速度命令"""
        if self.connected and not self.emergency_active:
            self.velocity_changed.emit(self.linear_velocity, self.angular_velocity)
    
    def update_velocity_controls(self):
        """更新速度控制显示"""
        # 更新滑块
        linear_slider_value = int((self.linear_velocity / MAX_LINEAR_VELOCITY) * 100)
        angular_slider_value = int((self.angular_velocity / MAX_ANGULAR_VELOCITY) * 100)
        
        self.linear_slider.blockSignals(True)
        self.linear_slider.setValue(linear_slider_value)
        self.linear_slider.blockSignals(False)
        
        self.angular_slider.blockSignals(True)
        self.angular_slider.setValue(angular_slider_value)
        self.angular_slider.blockSignals(False)
        
        # 更新输入框
        self.linear_spinbox.blockSignals(True)
        self.linear_spinbox.setValue(self.linear_velocity)
        self.linear_spinbox.blockSignals(False)
        
        self.angular_spinbox.blockSignals(True)
        self.angular_spinbox.setValue(self.angular_velocity)
        self.angular_spinbox.blockSignals(False)
    
    def start_navigation(self):
        """开始导航"""
        x = self.target_x_spinbox.value()
        y = self.target_y_spinbox.value()
        yaw = self.target_yaw_spinbox.value()
        
        # 转换角度为弧度
        import math
        yaw_rad = math.radians(yaw)
        
        self.navigation_requested.emit(x, y, yaw_rad)
        
        # 更新按钮状态
        self.navigate_btn.setEnabled(False)
        self.cancel_nav_btn.setEnabled(True)
    
    def cancel_navigation(self):
        """取消导航"""
        # 发送停止命令
        self.stop_movement()
        
        # 更新按钮状态
        self.navigate_btn.setEnabled(True)
        self.cancel_nav_btn.setEnabled(False)
    
    def set_elevator_position(self, position):
        """设置升降杆位置"""
        self.elevator_slider.setValue(position)
    
    def toggle_emergency_stop(self):
        """切换紧急停止状态"""
        self.emergency_active = True
        self.emergency_stop_requested.emit(True)
        
        # 停止所有运动
        self.stop_movement()
        
        # 更新UI
        self.emergency_btn.setEnabled(False)
        self.release_emergency_btn.setEnabled(True)
        self.enable_all_controls(False)
    
    def release_emergency_stop(self):
        """解除紧急停止"""
        self.emergency_active = False
        self.emergency_stop_requested.emit(False)
        
        # 更新UI
        self.emergency_btn.setEnabled(True)
        self.release_emergency_btn.setEnabled(False)
        self.enable_all_controls(True)
    
    def enable_manual_controls(self, enabled):
        """启用/禁用手动控制"""
        # 速度控制
        self.linear_slider.setEnabled(enabled)
        self.linear_spinbox.setEnabled(enabled)
        self.angular_slider.setEnabled(enabled)
        self.angular_spinbox.setEnabled(enabled)
        
        # 方向控制
        direction_buttons = [
            self.forward_btn, self.backward_btn, self.left_btn, self.right_btn,
            self.forward_left_btn, self.forward_right_btn,
            self.backward_left_btn, self.backward_right_btn
        ]
        for btn in direction_buttons:
            btn.setEnabled(enabled)
    
    def enable_all_controls(self, enabled):
        """启用/禁用所有控制"""
        # 模式控制
        self.auto_radio.setEnabled(enabled)
        self.manual_radio.setEnabled(enabled)
        self.pause_radio.setEnabled(enabled)
        
        # 手动控制
        self.enable_manual_controls(enabled and self.current_mode == 'manual')
        
        # 导航控制
        self.target_x_spinbox.setEnabled(enabled)
        self.target_y_spinbox.setEnabled(enabled)
        self.target_yaw_spinbox.setEnabled(enabled)
        self.navigate_btn.setEnabled(enabled)
        
        # 升降杆控制
        self.elevator_slider.setEnabled(enabled)
    
    # ================== 外部接口方法 ==================
    
    @pyqtSlot(bool)
    def set_connection_status(self, connected):
        """设置连接状态"""
        self.connected = connected
        self.enable_all_controls(connected and not self.emergency_active)
    
    @pyqtSlot(str)
    def update_robot_mode(self, mode):
        """更新机器人模式显示"""
        if mode == 'auto':
            self.auto_radio.setChecked(True)
        elif mode == 'manual':
            self.manual_radio.setChecked(True)
        elif mode == 'pause':
            self.pause_radio.setChecked(True)
        
        self.current_mode = mode
        self.enable_manual_controls(mode == 'manual' and self.connected)
    
    def get_current_mode(self):
        """获取当前模式"""
        return self.current_mode
    
    def get_current_velocity(self):
        """获取当前速度"""
        return self.linear_velocity, self.angular_velocity
    
    def is_emergency_active(self):
        """检查是否处于紧急停止状态"""
        return self.emergency_active