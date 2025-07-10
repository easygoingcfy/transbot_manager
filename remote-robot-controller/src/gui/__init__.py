"""GUI模块"""

# 使用 try-except 来处理可能的导入错误
try:
    from .main_window import MainWindow
except ImportError:
    MainWindow = None

try:
    from .control_panel import ControlPanel
except ImportError:
    ControlPanel = None

try:
    from .status_panel import StatusPanel
except ImportError:
    StatusPanel = None

try:
    from .camera_panel import CameraPanel
except ImportError:
    CameraPanel = None

try:
    from .task_panel import TaskPanel
except ImportError:
    TaskPanel = None

try:
    from .settings_dialog import SettingsDialog
except ImportError:
    SettingsDialog = None

__all__ = ['MainWindow', 'ControlPanel', 'StatusPanel', 'CameraPanel', 'TaskPanel', 'SettingsDialog']