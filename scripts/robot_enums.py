#!/usr/bin/env python
from enum import Enum

class TaskResult(Enum):
    SUCCESS = "success"
    FAILURE = "failure" 
    TIMEOUT = "timeout"
    CANCELLED = "cancelled"

class TaskStatus(Enum):
    IDLE = "idle"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"

class ControlMode(Enum):
    AUTO = "auto"        # 自动导航模式
    MANUAL = "manual"    # 手动遥控模式
    TASK = "task"        # 任务执行模式
    PAUSE = "pause"      # 暂停模式
    STOP = "stop"        # 停止模式

class SystemStatus(Enum):
    NORMAL = "normal"    # 正常状态
    WARNING = "warning"  # 警告状态
    ERROR = "error"      # 错误状态

class DeviceType(Enum):
    ELEVATOR = "elevator"
    CAMERA = "camera"
    LED = "led"
    BUZZER = "buzzer"
