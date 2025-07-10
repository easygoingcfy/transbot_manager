#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""常量定义"""

# 应用信息
APP_NAME = "远程机器人控制器"
APP_VERSION = "1.0.0"
APP_AUTHOR = "Robot Team"

# 默认值
DEFAULT_MQTT_HOST = "localhost"
DEFAULT_MQTT_PORT = 1883
DEFAULT_ROBOT_ID = "transbot_001"

# 控制限制
MAX_LINEAR_VELOCITY = 1.0  # m/s
MAX_ANGULAR_VELOCITY = 2.0  # rad/s
MIN_VELOCITY = 0.0

# 网络超时
MQTT_CONNECT_TIMEOUT = 10  # 秒
MQTT_KEEPALIVE = 60  # 秒
REQUEST_TIMEOUT = 5  # 秒

# 更新频率
STATUS_UPDATE_INTERVAL = 1000  # 毫秒
TELEMETRY_UPDATE_INTERVAL = 500  # 毫秒
IMAGE_UPDATE_INTERVAL = 200  # 毫秒

# 文件路径
CONFIG_DIR = "config"
LOG_DIR = "logs"
IMAGE_SAVE_DIR = "robot_images"

# UI尺寸
WINDOW_MIN_WIDTH = 800
WINDOW_MIN_HEIGHT = 600
WINDOW_DEFAULT_WIDTH = 1200
WINDOW_DEFAULT_HEIGHT = 800

# 机器人模式
ROBOT_MODES = ["auto", "manual", "pause"]

# 任务类型
TASK_TYPES = ["navigation", "patrol", "delivery", "inspection", "custom"]

# 日志级别
LOG_LEVELS = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]

# 主题样式
THEMES = ["默认", "深色", "浅色"]

# 语言选项
LANGUAGES = ["中文", "English"]

# 图像格式
SUPPORTED_IMAGE_FORMATS = ["PNG", "JPEG", "BMP", "TIFF"]

# 控制键映射
CONTROL_KEYS = {
    'W': 'forward',
    'S': 'backward', 
    'A': 'left',
    'D': 'right',
    'SPACE': 'emergency_stop',
    'C': 'capture'
}

# 错误代码
ERROR_CODES = {
    'MQTT_CONNECTION_FAILED': 1001,
    'ROBOT_NOT_RESPONDING': 1002,
    'INVALID_COMMAND': 1003,
    'TASK_EXECUTION_FAILED': 1004,
    'CONFIG_LOAD_FAILED': 1005
}

# 状态代码
STATUS_CODES = {
    'NORMAL': 'normal',
    'WARNING': 'warning', 
    'ERROR': 'error',
    'OFFLINE': 'offline'
}