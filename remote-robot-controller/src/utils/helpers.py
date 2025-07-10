#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time
import json
from datetime import datetime
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QByteArray, QBuffer, QIODevice

def format_timestamp(timestamp=None):
    """格式化时间戳"""
    if timestamp is None:
        timestamp = time.time()
    
    dt = datetime.fromtimestamp(timestamp)
    return dt.strftime("%Y-%m-%d %H:%M:%S")

def degrees_to_radians(degrees):
    """角度转弧度"""
    return math.radians(degrees)

def radians_to_degrees(radians):
    """弧度转角度"""
    return math.degrees(radians)

def euler_to_quaternion(roll, pitch, yaw):
    """欧拉角转四元数"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw

def quaternion_to_euler(qx, qy, qz, qw):
    """四元数转欧拉角"""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def distance_2d(x1, y1, x2, y2):
    """计算二维距离"""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def clamp(value, min_value, max_value):
    """限制数值范围"""
    return max(min_value, min(value, max_value))

def safe_float(value, default=0.0):
    """安全转换为浮点数"""
    try:
        return float(value)
    except (ValueError, TypeError):
        return default

def safe_int(value, default=0):
    """安全转换为整数"""
    try:
        return int(value)
    except (ValueError, TypeError):
        return default

def safe_json_loads(json_str, default=None):
    """安全解析JSON"""
    try:
        return json.loads(json_str)
    except (json.JSONDecodeError, TypeError):
        return default if default is not None else {}

def qimage_to_bytes(qimage, format='PNG', quality=90):
    """QImage转字节数据"""
    byte_array = QByteArray()
    buffer = QBuffer(byte_array)
    buffer.open(QIODevice.WriteOnly)
    qimage.save(buffer, format, quality)
    return byte_array.data()

def bytes_to_qimage(data):
    """字节数据转QImage"""
    qimage = QImage()
    qimage.loadFromData(data)
    return qimage

def validate_ip_address(ip):
    """验证IP地址"""
    try:
        parts = ip.split('.')
        return len(parts) == 4 and all(0 <= int(part) <= 255 for part in parts)
    except ValueError:
        return False

def validate_port(port):
    """验证端口号"""
    try:
        port_int = int(port)
        return 1 <= port_int <= 65535
    except ValueError:
        return False

def format_bytes(bytes_value):
    """格式化字节数"""
    for unit in ['B', 'KB', 'MB', 'GB']:
        if bytes_value < 1024.0:
            return f"{bytes_value:.1f} {unit}"
        bytes_value /= 1024.0
    return f"{bytes_value:.1f} TB"

def format_duration(seconds):
    """格式化时间间隔"""
    if seconds < 60:
        return f"{seconds:.1f} 秒"
    elif seconds < 3600:
        return f"{seconds/60:.1f} 分钟"
    else:
        hours = seconds // 3600
        minutes = (seconds % 3600) // 60
        return f"{hours:.0f} 小时 {minutes:.0f} 分钟"