#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import (QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
                             QSplitter, QMenuBar, QToolBar, QStatusBar,
                             QAction, QMessageBox, QTabWidget)
from PyQt5.QtCore import Qt, pyqtSlot, QTimer
from PyQt5.QtGui import QIcon, QKeySequence

from gui.control_panel import ControlPanel
from gui.status_panel import StatusPanel
from gui.camera_panel import CameraPanel
from gui.task_panel import TaskPanel
from gui.settings_dialog import SettingsDialog
from core.robot_controller import RobotController
from core.config_manager import ConfigManager
from core.logger import Logger
from utils.constants import APP_NAME, WINDOW_DEFAULT_WIDTH, WINDOW_DEFAULT_HEIGHT

class MainWindow(QMainWindow):
    """主窗口"""
    
    def __init__(self):
        super().__init__()
        self.logger = Logger.get_logger(__name__)
        
        # 初始化配置管理器
        self.config_manager = ConfigManager()
        self.config = self.config_manager.get_config()
        
        # 初始化机器人控制器
        self.robot_controller = RobotController(self.config)
        
        # 初始化UI
        self.init_ui()
        self.setup_connections()
        
        # 状态更新定时器
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(1000)  # 每秒更新一次
        
        self.logger.info("主窗口初始化完成")
    
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle(APP_NAME)
        self.setGeometry(100, 100, WINDOW_DEFAULT_WIDTH, WINDOW_DEFAULT_HEIGHT)
        
        # 创建菜单栏
        self.create_menu_bar()
        
        # 创建工具栏
        self.create_toolbar()
        
        # 创建中央窗口部件
        self.create_central_widget()
        
        # 创建状态栏
        self.create_status_bar()
    
    def create_menu_bar(self):
        """创建菜单栏"""
        menubar = self.menuBar()
        
        # 文件菜单
        file_menu = menubar.addMenu('文件(&F)')
        
        # 连接/断开连接
        self.connect_action = QAction('连接机器人(&C)', self)
        self.connect_action.setShortcut(QKeySequence('Ctrl+C'))
        self.connect_action.triggered.connect(self.toggle_connection)
        file_menu.addAction(self.connect_action)
        
        file_menu.addSeparator()
        
        # 设置
        settings_action = QAction('设置(&S)', self)
        settings_action.setShortcut(QKeySequence('Ctrl+S'))
        settings_action.triggered.connect(self.open_settings)
        file_menu.addAction(settings_action)
        
        file_menu.addSeparator()
        
        # 退出
        exit_action = QAction('退出(&Q)', self)
        exit_action.setShortcut(QKeySequence('Ctrl+Q'))
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # 控制菜单
        control_menu = menubar.addMenu('控制(&C)')
        
        # 紧急停止
        emergency_action = QAction('紧急停止(&E)', self)
        emergency_action.setShortcut(QKeySequence('Space'))
        emergency_action.triggered.connect(self.emergency_stop)
        control_menu.addAction(emergency_action)
        
        # 模式切换
        control_menu.addSeparator()
        auto_mode_action = QAction('自动模式(&A)', self)
        auto_mode_action.triggered.connect(lambda: self.set_mode('auto'))
        control_menu.addAction(auto_mode_action)
        
        manual_mode_action = QAction('手动模式(&M)', self)
        manual_mode_action.triggered.connect(lambda: self.set_mode('manual'))
        control_menu.addAction(manual_mode_action)
        
        pause_mode_action = QAction('暂停模式(&P)', self)
        pause_mode_action.triggered.connect(lambda: self.set_mode('pause'))
        control_menu.addAction(pause_mode_action)
        
        # 视图菜单
        view_menu = menubar.addMenu('视图(&V)')
        
        # 工具栏切换
        toolbar_action = QAction('工具栏(&T)', self)
        toolbar_action.setCheckable(True)
        toolbar_action.setChecked(True)
        toolbar_action.triggered.connect(self.toggle_toolbar)
        view_menu.addAction(toolbar_action)
        
        # 状态栏切换
        statusbar_action = QAction('状态栏(&S)', self)
        statusbar_action.setCheckable(True)
        statusbar_action.setChecked(True)
        statusbar_action.triggered.connect(self.toggle_statusbar)
        view_menu.addAction(statusbar_action)
        
        # 帮助菜单
        help_menu = menubar.addMenu('帮助(&H)')
        
        # 关于
        about_action = QAction('关于(&A)', self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)
    
    def create_toolbar(self):
        """创建工具栏"""
        self.toolbar = QToolBar()
        self.addToolBar(self.toolbar)
        
        # 连接按钮
        self.connect_btn = self.toolbar.addAction('连接')
        self.connect_btn.triggered.connect(self.toggle_connection)
        
        self.toolbar.addSeparator()
        
        # 紧急停止按钮
        emergency_btn = self.toolbar.addAction('紧急停止')
        emergency_btn.triggered.connect(self.emergency_stop)
        
        self.toolbar.addSeparator()
        
        # 模式按钮
        auto_btn = self.toolbar.addAction('自动')
        auto_btn.triggered.connect(lambda: self.set_mode('auto'))
        
        manual_btn = self.toolbar.addAction('手动')
        manual_btn.triggered.connect(lambda: self.set_mode('manual'))
        
        pause_btn = self.toolbar.addAction('暂停')
        pause_btn.triggered.connect(lambda: self.set_mode('pause'))
    
    def create_central_widget(self):
        """创建中央窗口部件"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QHBoxLayout(central_widget)
        
        # 左右分割器
        main_splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(main_splitter)
        
        # 左侧面板
        left_panel = self.create_left_panel()
        main_splitter.addWidget(left_panel)
        
        # 右侧面板
        right_panel = self.create_right_panel()
        main_splitter.addWidget(right_panel)
        
        # 设置分割器比例
        main_splitter.setStretchFactor(0, 1)
        main_splitter.setStretchFactor(1, 2)
    
    def create_left_panel(self):
        """创建左侧面板"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # 状态面板
        self.status_panel = StatusPanel()
        layout.addWidget(self.status_panel)
        
        # 控制面板
        self.control_panel = ControlPanel()
        layout.addWidget(self.control_panel)
        
        return widget
    
    def create_right_panel(self):
        """创建右侧面板"""
        # 使用标签页组织右侧内容
        tab_widget = QTabWidget()
        
        # 相机面板
        self.camera_panel = CameraPanel()
        tab_widget.addTab(self.camera_panel, "相机")
        
        # 任务面板
        self.task_panel = TaskPanel()
        tab_widget.addTab(self.task_panel, "任务")
        
        return tab_widget
    
    def create_status_bar(self):
        """创建状态栏"""
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        
        # 显示就绪状态
        self.status_bar.showMessage("就绪")
    
    def setup_connections(self):
        """设置信号连接"""
        # 机器人控制器信号
        self.robot_controller.status_updated.connect(self.status_panel.update_status)
        self.robot_controller.connection_changed.connect(self.on_connection_changed)
        self.robot_controller.image_received.connect(self.camera_panel.display_image)
        self.robot_controller.error_occurred.connect(self.show_error)
        
        # 控制面板信号
        self.control_panel.mode_changed.connect(self.robot_controller.set_mode)
        self.control_panel.velocity_changed.connect(self.robot_controller.send_velocity)
        self.control_panel.navigation_requested.connect(self.robot_controller.send_navigation)
        self.control_panel.elevator_changed.connect(self.robot_controller.send_elevator)
        self.control_panel.emergency_stop_requested.connect(self.robot_controller.emergency_stop)
        
        # 相机面板信号
        self.camera_panel.capture_requested.connect(self.robot_controller.send_camera)
        
        # 任务面板信号
        self.task_panel.task_created.connect(self.robot_controller.send_task)
    
    def toggle_connection(self):
        """切换连接状态"""
        if self.robot_controller.is_connected():
            self.robot_controller.disconnect()
        else:
            self.robot_controller.connect()
    
    def emergency_stop(self):
        """紧急停止"""
        self.robot_controller.emergency_stop(True)
        self.status_bar.showMessage("紧急停止已激活", 3000)
    
    def set_mode(self, mode):
        """设置机器人模式"""
        self.robot_controller.set_mode(mode)
        self.status_bar.showMessage(f"模式已设置为: {mode}", 2000)
    
    @pyqtSlot(bool)
    def on_connection_changed(self, connected):
        """连接状态变化处理"""
        if connected:
            self.connect_action.setText('断开连接(&D)')
            self.connect_btn.setText('断开')
            self.status_bar.showMessage("已连接到机器人")
        else:
            self.connect_action.setText('连接机器人(&C)')
            self.connect_btn.setText('连接')
            self.status_bar.showMessage("与机器人连接断开")
        
        # 更新状态面板
        self.status_panel.update_connection_status(connected)
    
    def update_status(self):
        """更新状态"""
        # 定期更新机器人状态
        if self.robot_controller.is_connected():
            self.robot_controller.update_status()
    
    def open_settings(self):
        """打开设置对话框"""
        dialog = SettingsDialog(self.config_manager, self)
        if dialog.exec_() == dialog.Accepted:
            # 重新加载配置
            self.config = self.config_manager.get_config()
            self.robot_controller.update_config(self.config)
            self.status_bar.showMessage("设置已更新", 2000)
    
    def show_error(self, error_message):
        """显示错误消息"""
        QMessageBox.critical(self, "错误", error_message)
        self.logger.error(f"显示错误消息: {error_message}")
    
    def toggle_toolbar(self, visible):
        """切换工具栏显示"""
        self.toolbar.setVisible(visible)
    
    def toggle_statusbar(self, visible):
        """切换状态栏显示"""
        self.status_bar.setVisible(visible)
    
    def show_about(self):
        """显示关于对话框"""
        QMessageBox.about(self, "关于", 
                         f"{APP_NAME}\n\n"
                         "基于PyQt5和MQTT的远程机器人控制器\n"
                         "版本: 1.0.0\n"
                         "作者: Robot Team")
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        reply = QMessageBox.question(self, '确认退出', 
                                   '确定要退出程序吗？',
                                   QMessageBox.Yes | QMessageBox.No,
                                   QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            # 断开机器人连接
            if self.robot_controller.is_connected():
                self.robot_controller.disconnect()
            
            self.logger.info("程序正常退出")
            event.accept()
        else:
            event.ignore()