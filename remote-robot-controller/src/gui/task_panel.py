#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import time
import uuid
from datetime import datetime
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QLabel, QGroupBox, QPushButton, QListWidget,
                             QListWidgetItem, QTextEdit, QComboBox, QSpinBox,
                             QDoubleSpinBox, QCheckBox, QProgressBar, QLineEdit,
                             QTabWidget, QMessageBox, QSplitter, QScrollArea,
                             QFrame, QToolButton, QMenu, QAction)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QMimeData, QPoint, QRect
from PyQt5.QtGui import QFont, QDrag, QPainter, QPen, QBrush, QColor, QPixmap, QIcon

# 本地导入
from .task_creator_dialog import TaskCreatorDialog
from .json_editor_dialog import JsonEditorDialog

class DraggableTaskItem(QFrame):
    """可拖拽的任务项目 - 大尺寸清晰版本"""
    
    task_selected = pyqtSignal(object)
    task_deleted = pyqtSignal(object)
    task_edited = pyqtSignal(object)
    
    def __init__(self, task_data, parent=None, compact_mode=False):
        super().__init__(parent)
        self.task_data = task_data
        self.compact_mode = compact_mode
        self.selected = False
        
        if compact_mode:
            # 紧凑模式 - 用于基本任务库，2倍大小
            self.setMinimumSize(280, 130)
            self.setMaximumSize(280, 130)
        else:
            # 标准模式 - 用于序列构建器
            self.setMinimumSize(180, 110)
            self.setMaximumSize(180, 110)
        
        self.setFrameStyle(QFrame.NoFrame)
        self.init_ui()
        
    def init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout(self)
        
        if self.compact_mode:
            layout.setContentsMargins(16, 12, 16, 12)
            layout.setSpacing(8)
        else:
            layout.setContentsMargins(12, 10, 12, 10)
            layout.setSpacing(6)
        
        # 任务类型标签
        behavior = self.task_data.get('behavior', 'unknown')
        type_text = self.get_behavior_display_name(behavior)
        
        type_label = QLabel(type_text)
        type_label.setAlignment(Qt.AlignCenter)
        if self.compact_mode:
            type_label.setStyleSheet("font-size: 20px; font-weight: bold; color: white;")
        else:
            type_label.setStyleSheet("font-size: 16px; font-weight: bold; color: white;")
        layout.addWidget(type_label)
        
        # 任务名称
        name = self.task_data.get('name', 'Unnamed')
        display_name = self.get_simplified_name(name)
        
        name_label = QLabel(display_name)
        name_label.setAlignment(Qt.AlignCenter)
        name_label.setWordWrap(True)
        if self.compact_mode:
            name_label.setStyleSheet("font-size: 16px; color: white; font-weight: normal;")
        else:
            name_label.setStyleSheet("font-size: 13px; color: white; font-weight: normal;")
        layout.addWidget(name_label)
        
        # 参数信息
        params = self.task_data.get('params', {})
        if params:
            param_text = self.get_param_text(behavior, params)
            if param_text:
                param_label = QLabel(param_text)
                param_label.setAlignment(Qt.AlignCenter)
                if self.compact_mode:
                    param_label.setStyleSheet("font-size: 14px; color: rgba(255,255,255,0.9);")
                else:
                    param_label.setStyleSheet("font-size: 11px; color: rgba(255,255,255,0.8);")
                layout.addWidget(param_label)
        
        self.update_style()
    
    def get_behavior_display_name(self, behavior):
        """获取行为显示名称"""
        display_map = {
            'navigate': '导航',
            'capture_image': '拍照',
            'wait': '等待',
            'speak': '语音',
            'rotate': '旋转'
        }
        return display_map.get(behavior, behavior.upper())
    
    def get_simplified_name(self, name):
        """获取简化名称"""
        name_map = {
            'move_forward': '前进',
            'move_left': '左移',
            'move_right': '右移',
            'move_back': '后退',
            'capture_rgb': 'RGB相机',
            'capture_depth': '深度相机',
            'wait_short': '短等待',
            'wait_long': '长等待',
            'speak_start': '开始提示',
            'speak_complete': '完成提示',
            'rotate_90': '旋转90°',
            'rotate_180': '旋转180°',
            # 基本模板名称
            'basic_navigate': '基础导航',
            'basic_capture': '基础拍照',
            'basic_wait': '基础等待',
            'basic_speak': '基础语音',
            'basic_rotate': '基础旋转'
        }
        
        simplified = name_map.get(name, name)
        # 紧凑模式可以显示更多字符
        max_len = 16 if self.compact_mode else 12
        return simplified[:max_len] + '...' if len(simplified) > max_len else simplified
    
    def get_param_text(self, behavior, params):
        """获取参数显示文本"""
        if behavior == "navigate":
            x, y = params.get('x', 0), params.get('y', 0)
            theta = params.get('theta', 0)
            if x != 0 or y != 0 or theta != 0:
                return f"X:{x:.1f}m Y:{y:.1f}m θ:{theta:.1f}°"
            return "可自定义坐标"
        elif behavior == "wait":
            duration = params.get('duration', 0)
            if duration > 0:
                return f"等待 {duration:.1f} 秒"
            return "可自定义时长"
        elif behavior == "rotate":
            angle = params.get('angle_delta', 0) * 180 / 3.14159
            if angle != 0:
                return f"旋转 {angle:.0f}°"
            return "可自定义角度"
        elif behavior == "speak":
            text = params.get('text', '')
            if text:
                max_len = 12 if self.compact_mode else 8
                return text[:max_len] + '..' if len(text) > max_len else text
            return "可自定义内容"
        elif behavior == "capture_image":
            camera_type = params.get('camera_type', 'rgb')
            return f"{camera_type.upper()} 相机"
        return "点击编辑参数"
    
    def update_style(self):
        """更新样式 - 简约风格"""
        behavior = self.task_data.get('behavior', 'unknown')
        
        # 简约的颜色方案
        color_map = {
            'navigate': '#5C6BC0',      # 靛蓝
            'capture_image': '#42A5F5',  # 浅蓝
            'wait': '#FFA726',          # 橙色
            'speak': '#AB47BC',         # 紫色
            'rotate': '#EF5350'         # 红色
        }
        
        base_color = color_map.get(behavior, '#78909C')
        
        if self.selected:
            border_style = f"border: 4px solid #FF9800;"
        else:
            border_style = f"border: 3px solid #E0E0E0;"
        
        border_radius = "12px" if self.compact_mode else "10px"
        
        self.setStyleSheet(f"""
            QFrame {{
                background-color: {base_color};
                {border_style}
                border-radius: {border_radius};
            }}
            QFrame:hover {{
                border: 4px solid #2196F3;
            }}
        """)
    
    def mousePressEvent(self, event):
        """鼠标按下事件"""
        if event.button() == Qt.LeftButton:
            self.selected = not self.selected
            self.update_style()
            self.task_selected.emit(self)
        elif event.button() == Qt.RightButton:
            # 右键菜单
            menu = QMenu(self)
            
            edit_action = QAction("编辑参数", self)
            edit_action.triggered.connect(lambda: self.task_edited.emit(self))
            menu.addAction(edit_action)
            
            delete_action = QAction("删除", self)
            delete_action.triggered.connect(lambda: self.task_deleted.emit(self))
            menu.addAction(delete_action)
            
            menu.exec_(event.globalPos())
        
        super().mousePressEvent(event)
    
    def mouseMoveEvent(self, event):
        """鼠标移动事件 - 开始拖拽"""
        if event.buttons() == Qt.LeftButton:
            drag = QDrag(self)
            mimeData = QMimeData()
            mimeData.setText(json.dumps(self.task_data))
            drag.setMimeData(mimeData)
            
            # 创建拖拽图标
            pixmap = QPixmap(self.size())
            self.render(pixmap)
            drag.setPixmap(pixmap)
            
            drag.exec_(Qt.CopyAction | Qt.MoveAction)

class TaskSequenceBuilder(QFrame):
    """任务序列构建器"""
    
    sequence_changed = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.task_sequence = []
        self.setAcceptDrops(True)
        self.setMinimumHeight(130)
        self.setFrameStyle(QFrame.Box)
        self.setStyleSheet("""
            QFrame {
                background-color: #FAFAFA;
                border: 3px dashed #BDBDBD;
                border-radius: 12px;
            }
        """)
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(12)
        
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.scroll_area.setStyleSheet("""
            QScrollArea {
                border: none;
                background: transparent;
            }
        """)
        
        self.sequence_widget = QWidget()
        self.sequence_layout = QHBoxLayout(self.sequence_widget)
        self.sequence_layout.setContentsMargins(8, 8, 8, 8)
        self.sequence_layout.setSpacing(15)
        
        self.scroll_area.setWidget(self.sequence_widget)
        layout.addWidget(self.scroll_area)
        
        self.update_display()
    
    def dragEnterEvent(self, event):
        """拖拽进入事件"""
        if event.mimeData().hasText():
            event.acceptProposedAction()
            self.setStyleSheet("""
                QFrame {
                    background-color: #E8F5E8;
                    border: 4px dashed #4CAF50;
                    border-radius: 12px;
                }
            """)
    
    def dragLeaveEvent(self, event):
        """拖拽离开事件"""
        self.setStyleSheet("""
            QFrame {
                background-color: #FAFAFA;
                border: 3px dashed #BDBDBD;
                border-radius: 12px;
            }
        """)
    
    def dropEvent(self, event):
        """放置事件"""
        self.dragLeaveEvent(event)
        
        try:
            task_data = json.loads(event.mimeData().text())
            
            # 计算插入位置
            drop_pos = event.pos().x()
            insert_index = self.calculate_insert_position(drop_pos)
            
            # 插入任务
            self.task_sequence.insert(insert_index, task_data.copy())
            self.update_display()
            self.sequence_changed.emit()
            
        except json.JSONDecodeError:
            pass
    
    def calculate_insert_position(self, x_pos):
        """计算插入位置"""
        if not self.task_sequence:
            return 0
        
        item_width = 200  # 任务项宽度 + 间距
        index = max(0, min(len(self.task_sequence), x_pos // item_width))
        return index
    
    def update_display(self):
        """更新显示"""
        # 清除现有项目
        for i in reversed(range(self.sequence_layout.count())):
            child = self.sequence_layout.itemAt(i).widget()
            if child:
                child.setParent(None)
        
        # 添加任务项目
        for i, task_data in enumerate(self.task_sequence):
            task_item = DraggableTaskItem(task_data, compact_mode=False)
            task_item.task_deleted.connect(lambda item, idx=i: self.remove_task(idx))
            task_item.task_edited.connect(lambda item, idx=i: self.edit_task(idx))
            self.sequence_layout.addWidget(task_item)
            
            # 添加箭头（除了最后一个）
            if i < len(self.task_sequence) - 1:
                arrow_label = QLabel("→")
                arrow_label.setAlignment(Qt.AlignCenter)
                arrow_label.setStyleSheet("""
                    font-size: 32px; 
                    color: #757575; 
                    font-weight: bold;
                    background: transparent;
                """)
                self.sequence_layout.addWidget(arrow_label)
        
        # 添加弹簧
        self.sequence_layout.addStretch()
        
        # 更新提示文本
        if not self.task_sequence:
            if self.sequence_layout.count() == 1:  # 只有弹簧
                hint_label = QLabel("拖拽左侧任务到这里组成执行序列")
                hint_label.setAlignment(Qt.AlignCenter)
                hint_label.setStyleSheet("""
                    color: #9E9E9E; 
                    font-style: italic; 
                    font-size: 18px;
                    background: transparent;
                """)
                self.sequence_layout.insertWidget(0, hint_label)
    
    def remove_task(self, index):
        """移除任务"""
        if 0 <= index < len(self.task_sequence):
            self.task_sequence.pop(index)
            self.update_display()
            self.sequence_changed.emit()
    
    def edit_task(self, index):
        """编辑任务"""
        if 0 <= index < len(self.task_sequence):
            task_data = self.task_sequence[index]
            behavior = task_data.get('behavior')
            
            dialog = TaskCreatorDialog(behavior, self.parent(), edit_data=task_data)
            if dialog.exec_() == dialog.Accepted:
                new_data = dialog.get_task_data()
                if new_data:
                    self.task_sequence[index] = new_data
                    self.update_display()
                    self.sequence_changed.emit()
    
    def clear_sequence(self):
        """清空序列"""
        self.task_sequence.clear()
        self.update_display()
        self.sequence_changed.emit()
    
    def get_sequence_data(self):
        """获取序列数据"""
        if not self.task_sequence:
            return None
        
        return {
            "task_id": f"sequence_{int(time.time())}",
            "type": "sequence",
            "name": f"custom_sequence_{len(self.task_sequence)}_tasks",
            "children": self.task_sequence.copy()
        }

class CompactTaskLibrary(QFrame):
    """基本任务库 - 大卡片版本"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.basic_tasks = []
        self.init_ui()
        self.load_basic_tasks()
    
    def init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)
        
        # 标题
        title_label = QLabel("基本任务库")
        title_label.setStyleSheet("""
            font-weight: bold; 
            font-size: 18px; 
            color: #333333;
            padding: 12px;
            background-color: #F5F5F5;
            border-radius: 6px;
        """)
        layout.addWidget(title_label)
        
        # 任务网格 - 改为单列布局
        self.tasks_widget = QWidget()
        self.tasks_layout = QVBoxLayout(self.tasks_widget)
        self.tasks_layout.setSpacing(12)
        self.tasks_layout.setContentsMargins(12, 12, 12, 12)
        
        # 滚动区域
        scroll_area = QScrollArea()
        scroll_area.setWidget(self.tasks_widget)
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll_area.setStyleSheet("""
            QScrollArea {
                border: 2px solid #E0E0E0;
                border-radius: 8px;
                background-color: white;
            }
            QScrollBar:vertical {
                background: #F8F8F8;
                width: 14px;
                border-radius: 7px;
            }
            QScrollBar::handle:vertical {
                background: #CCCCCC;
                border-radius: 7px;
                min-height: 40px;
            }
            QScrollBar::handle:vertical:hover {
                background: #AAAAAA;
            }
        """)
        
        layout.addWidget(scroll_area, 1)
        
        # 快速创建按钮组
        quick_group = QGroupBox("快速创建")
        quick_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 16px;
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
        quick_layout = QGridLayout(quick_group)
        quick_layout.setSpacing(8)
        quick_layout.setContentsMargins(12, 16, 12, 12)
        
        # 按钮样式 - 更大的按钮
        button_style = """
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                padding: 12px 16px;
                border-radius: 8px;
                font-weight: bold;
                font-size: 14px;
                min-height: 40px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #1565C0;
            }
        """
        
        # 创建按钮
        buttons = [
            ("导航", 'navigate', 0, 0),
            ("拍照", 'capture_image', 0, 1),
            ("等待", 'wait', 1, 0),
            ("语音", 'speak', 1, 1),
            ("旋转", 'rotate', 2, 0)
        ]
        
        for text, behavior, row, col in buttons:
            btn = QPushButton(text)
            btn.setStyleSheet(button_style)
            btn.clicked.connect(lambda checked, b=behavior: self.create_quick_task(b))
            quick_layout.addWidget(btn, row, col)
        
        layout.addWidget(quick_group)
    
    def load_basic_tasks(self):
        """加载基本任务 - 每种类型只有一个"""
        templates = [
            {
                "task_id": "template_basic_navigate",
                "type": "action",
                "name": "basic_navigate",
                "behavior": "navigate",
                "params": {"x": 0.5, "y": 0.0, "theta": 0.0}
            },
            {
                "task_id": "template_basic_capture",
                "type": "action",
                "name": "basic_capture",
                "behavior": "capture_image",
                "params": {"camera_type": "rgb", "timeout": 5.0}
            },
            {
                "task_id": "template_basic_wait",
                "type": "action",
                "name": "basic_wait",
                "behavior": "wait",
                "params": {"duration": 3.0}
            },
            {
                "task_id": "template_basic_speak",
                "type": "action",
                "name": "basic_speak",
                "behavior": "speak",
                "params": {"text": "Hello"}
            },
            {
                "task_id": "template_basic_rotate",
                "type": "action",
                "name": "basic_rotate",
                "behavior": "rotate",
                "params": {"angle_delta": 1.57, "angular_speed": 0.5}
            }
        ]
        
        self.basic_tasks = templates
        self.update_library_display()
    
    def update_library_display(self):
        """更新任务库显示 - 单列大卡片"""
        # 清除现有项目
        for i in reversed(range(self.tasks_layout.count())):
            child = self.tasks_layout.itemAt(i).widget()
            if child:
                child.setParent(None)
        
        # 按类型分组
        task_groups = [
            ('navigate', [t for t in self.basic_tasks if t.get('behavior') == 'navigate']),
            ('capture_image', [t for t in self.basic_tasks if t.get('behavior') == 'capture_image']),
            ('wait', [t for t in self.basic_tasks if t.get('behavior') == 'wait']),
            ('speak', [t for t in self.basic_tasks if t.get('behavior') == 'speak']),
            ('rotate', [t for t in self.basic_tasks if t.get('behavior') == 'rotate']),
            ('custom', [t for t in self.basic_tasks if not t.get('task_id', '').startswith('template_')])
        ]
        
        group_names = {
            'navigate': '🧭 导航任务',
            'capture_image': '📷 拍照任务',
            'wait': '⏰ 等待任务',
            'speak': '🔊 语音任务',
            'rotate': '🔄 旋转任务',
            'custom': '⚙️ 自定义任务'
        }
        
        for group_type, tasks in task_groups:
            if not tasks:
                continue
                
            # 添加分组标题
            group_label = QLabel(group_names[group_type])
            group_label.setStyleSheet(f"""
                font-size: 14px; 
                font-weight: bold; 
                color: #555555; 
                padding: 8px 4px;
                background-color: #F8F9FA;
                border-radius: 4px;
                margin: 4px 0;
            """)
            self.tasks_layout.addWidget(group_label)
            
            # 添加任务项目 - 每个占满宽度
            for task in tasks:
                task_item = DraggableTaskItem(task, compact_mode=True)
                task_item.task_deleted.connect(lambda item: self.remove_from_library(item))
                task_item.task_edited.connect(lambda item: self.edit_library_task(item))
                self.tasks_layout.addWidget(task_item)
        
        # 添加弹簧
        self.tasks_layout.addStretch()
    
    def create_quick_task(self, behavior_type):
        """快速创建任务"""
        dialog = TaskCreatorDialog(behavior_type, self)
        if dialog.exec_() == dialog.Accepted:
            task_data = dialog.get_task_data()
            if task_data:
                self.basic_tasks.append(task_data)
                self.update_library_display()
    
    def edit_library_task(self, task_item):
        """编辑库中的任务"""
        task_data = task_item.task_data
        behavior = task_data.get('behavior')
        
        dialog = TaskCreatorDialog(behavior, self, edit_data=task_data)
        if dialog.exec_() == dialog.Accepted:
            new_data = dialog.get_task_data()
            if new_data:
                for i, task in enumerate(self.basic_tasks):
                    if task.get('task_id') == task_data.get('task_id'):
                        self.basic_tasks[i] = new_data
                        break
                self.update_library_display()
    
    def remove_from_library(self, task_item):
        """从库中移除任务"""
        task_data = task_item.task_data
        if task_data in self.basic_tasks:
            self.basic_tasks.remove(task_data)
            self.update_library_display()

class TaskPanel(QWidget):
    """任务面板 - 大卡片最终版本"""
    
    # 定义信号
    task_created = pyqtSignal(dict)
    task_cancelled = pyqtSignal(str)
    task_paused = pyqtSignal(str)
    task_resumed = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.current_tasks = []
        self.init_ui()
    
    def init_ui(self):
        """初始化用户界面"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        
        # 主分割器
        main_splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(main_splitter)
        
        # 左侧：任务库 - 稍微增加宽度以适应大卡片
        self.task_library = CompactTaskLibrary()
        self.task_library.setMaximumWidth(320)
        self.task_library.setMinimumWidth(300)
        main_splitter.addWidget(self.task_library)
        
        # 右侧：序列构建和控制
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        
        # 序列构建器
        builder_group = QGroupBox("任务序列构建")
        builder_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 16px;
                border: 3px solid #E0E0E0;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 16px;
                padding: 0 8px 0 8px;
            }
        """)
        builder_layout = QVBoxLayout(builder_group)
        
        self.sequence_builder = TaskSequenceBuilder()
        self.sequence_builder.sequence_changed.connect(self.on_sequence_changed)
        builder_layout.addWidget(self.sequence_builder)
        
        # 序列控制按钮
        control_layout = QHBoxLayout()
        
        button_style = """
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                padding: 16px 24px;
                border-radius: 8px;
                font-weight: bold;
                font-size: 16px;
            }
            QPushButton:hover {
                background-color: #45A049;
            }
            QPushButton:pressed {
                background-color: #3D8B40;
            }
            QPushButton:disabled {
                background-color: #CCCCCC;
            }
        """
        
        self.create_sequence_btn = QPushButton("创建序列任务")
        self.create_sequence_btn.setStyleSheet(button_style)
        self.create_sequence_btn.clicked.connect(self.create_sequence_task)
        self.create_sequence_btn.setEnabled(False)
        control_layout.addWidget(self.create_sequence_btn)
        
        clear_button_style = button_style.replace('#4CAF50', '#F44336').replace('#45A049', '#E53935').replace('#3D8B40', '#C62828')
        self.clear_sequence_btn = QPushButton("清空序列")
        self.clear_sequence_btn.setStyleSheet(clear_button_style)
        self.clear_sequence_btn.clicked.connect(self.sequence_builder.clear_sequence)
        control_layout.addWidget(self.clear_sequence_btn)
        
        control_layout.addStretch()
        
        builder_layout.addLayout(control_layout)
        right_layout.addWidget(builder_group)
        
        # 任务队列
        queue_group = QGroupBox("任务队列")
        queue_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 16px;
                border: 3px solid #E0E0E0;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 16px;
                padding: 0 8px 0 8px;
            }
        """)
        queue_layout = QVBoxLayout(queue_group)
        
        self.task_list = QListWidget()
        self.task_list.setStyleSheet("""
            QListWidget {
                border: 2px solid #E0E0E0;
                border-radius: 8px;
                background-color: white;
                font-size: 15px;
            }
            QListWidget::item {
                padding: 12px;
                border-bottom: 1px solid #F5F5F5;
            }
            QListWidget::item:selected {
                background-color: #2196F3;
                color: white;
            }
            QListWidget::item:hover {
                background-color: #E3F2FD;
            }
        """)
        self.task_list.itemClicked.connect(self.show_task_details)
        queue_layout.addWidget(self.task_list)
        
        # 任务详情
        self.task_details = QTextEdit()
        self.task_details.setMaximumHeight(140)
        self.task_details.setReadOnly(True)
        self.task_details.setStyleSheet("""
            QTextEdit {
                border: 2px solid #E0E0E0;
                border-radius: 8px;
                background-color: #FAFAFA;
                font-family: 'Consolas', monospace;
                font-size: 14px;
                padding: 12px;
            }
        """)
        queue_layout.addWidget(self.task_details)
        
        # 任务控制
        task_control_layout = QHBoxLayout()
        
        execute_style = """
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                padding: 12px 20px;
                border-radius: 8px;
                font-weight: bold;
                font-size: 15px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
        """
        
        self.execute_task_btn = QPushButton("执行选中任务")
        self.execute_task_btn.setStyleSheet(execute_style)
        self.execute_task_btn.clicked.connect(self.execute_selected_task)
        task_control_layout.addWidget(self.execute_task_btn)
        
        self.cancel_task_btn = QPushButton("取消任务")
        self.cancel_task_btn.setStyleSheet(execute_style.replace('#2196F3', '#FF9800').replace('#1976D2', '#F57C00'))
        self.cancel_task_btn.clicked.connect(self.cancel_selected_task)
        task_control_layout.addWidget(self.cancel_task_btn)
        
        # JSON编辑按钮
        self.edit_json_btn = QPushButton("编辑JSON")
        self.edit_json_btn.setStyleSheet(execute_style.replace('#2196F3', '#9C27B0').replace('#1976D2', '#7B1FA2'))
        self.edit_json_btn.clicked.connect(self.edit_task_json)
        task_control_layout.addWidget(self.edit_json_btn)
        
        task_control_layout.addStretch()
        
        self.clear_queue_btn = QPushButton("清空队列")
        self.clear_queue_btn.setStyleSheet(execute_style.replace('#2196F3', '#F44336').replace('#1976D2', '#D32F2F'))
        self.clear_queue_btn.clicked.connect(self.clear_task_queue)
        task_control_layout.addWidget(self.clear_queue_btn)
        
        queue_layout.addLayout(task_control_layout)
        right_layout.addWidget(queue_group)
        
        main_splitter.addWidget(right_widget)
        
        # 设置分割器比例
        main_splitter.setSizes([320, 700])
    
    def on_sequence_changed(self):
        """序列变化处理"""
        has_tasks = len(self.sequence_builder.task_sequence) > 0
        self.create_sequence_btn.setEnabled(has_tasks)
    
    def create_sequence_task(self):
        """创建序列任务"""
        sequence_data = self.sequence_builder.get_sequence_data()
        if sequence_data:
            self.add_task_to_queue(sequence_data)
            self.sequence_builder.clear_sequence()
            
            QMessageBox.information(self, "任务创建成功", 
                                   f"已创建包含 {len(sequence_data['children'])} 个步骤的序列任务")
    
    def add_task_to_queue(self, task_data):
        """添加任务到队列"""
        self.current_tasks.append(task_data)
        self.update_task_queue()
    
    def update_task_queue(self):
        """更新任务队列显示"""
        self.task_list.clear()
        
        for task in self.current_tasks:
            status = task.get('status', 'created')
            task_type = task.get('type', 'unknown')
            task_name = task.get('name', 'Unnamed')
            
            if task_type == 'sequence' and 'children' in task:
                item_text = f"[{status.upper()}] {task_name} ({len(task['children'])} 步骤)"
            else:
                item_text = f"[{status.upper()}] {task_name} ({task_type})"
            
            item = QListWidgetItem(item_text)
            item.setData(Qt.UserRole, task)
            self.task_list.addItem(item)
    
    def show_task_details(self, item):
        """显示任务详情"""
        task = item.data(Qt.UserRole)
        if task:
            details = self.format_task_details(task)
            self.task_details.setText(details)
    
    def format_task_details(self, task):
        """格式化任务详情"""
        details = f"任务名称: {task.get('name', 'Unnamed')}\n"
        details += f"任务类型: {task.get('type', 'unknown')}\n"
        details += f"任务ID: {task.get('task_id', 'unknown')}\n"
        
        if task.get('type') == 'sequence' and 'children' in task:
            details += f"步骤数量: {len(task['children'])}\n"
            for i, child in enumerate(task['children'], 1):
                behavior = child.get('behavior', 'unknown')
                name = child.get('name', f'步骤 {i}')
                details += f"  {i}. {name} ({behavior})\n"
        elif task.get('type') == 'action':
            details += f"行为类型: {task.get('behavior', 'unknown')}\n"
            params = task.get('params', {})
            if params:
                details += "参数: "
                param_str = ", ".join([f"{k}={v}" for k, v in params.items()])
                details += param_str
        
        return details
    
    def edit_task_json(self):
        """编辑任务JSON"""
        current_item = self.task_list.currentItem()
        if current_item:
            task = current_item.data(Qt.UserRole)
            
            dialog = JsonEditorDialog(task, self)
            if dialog.exec_() == dialog.Accepted:
                edited_data = dialog.get_edited_data()
                if edited_data:
                    for i, current_task in enumerate(self.current_tasks):
                        if current_task.get('task_id') == task.get('task_id'):
                            self.current_tasks[i] = edited_data
                            break
                    
                    self.update_task_queue()
                    QMessageBox.information(self, "编辑成功", "任务JSON已更新")
        else:
            QMessageBox.warning(self, "编辑失败", "请先选择要编辑的任务")
    
    def execute_selected_task(self):
        """执行选中的任务"""
        current_item = self.task_list.currentItem()
        if current_item:
            task = current_item.data(Qt.UserRole)
            task['status'] = 'executing'
            self.update_task_queue()
            
            print(f"[TASK_PANEL] 执行任务: {task['name']}")
            self.task_created.emit(task)
        else:
            QMessageBox.warning(self, "执行失败", "请先选择要执行的任务")
    
    def cancel_selected_task(self):
        """取消选中的任务"""
        current_item = self.task_list.currentItem()
        if current_item:
            task = current_item.data(Qt.UserRole)
            task['status'] = 'cancelled'
            self.task_cancelled.emit(task['task_id'])
            self.update_task_queue()
            print(f"[TASK_PANEL] 取消任务: {task['name']}")
        else:
            QMessageBox.warning(self, "取消失败", "请先选择要取消的任务")
    
    def clear_task_queue(self):
        """清空任务队列"""
        reply = QMessageBox.question(self, '确认清空', 
                                   '确定要清空所有任务吗？',
                                   QMessageBox.Yes | QMessageBox.No,
                                   QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.current_tasks.clear()
            self.task_list.clear()
            self.task_details.clear()
            print("[TASK_PANEL] 任务队列已清空")
    
    def update_task_status(self, task_id, status):
        """更新任务状态"""
        for task in self.current_tasks:
            if task.get('task_id') == task_id:
                task['status'] = status
                break
        self.update_task_queue()
        print(f"[TASK_PANEL] 更新任务状态: {task_id} -> {status}")
    
    def get_pending_tasks(self):
        """获取待执行的任务"""
        return [task for task in self.current_tasks 
                if task.get('status', 'created') in ['created', 'executing']]