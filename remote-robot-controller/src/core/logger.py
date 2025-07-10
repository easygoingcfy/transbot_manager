#!/usr/bin/env python
# -*- coding: utf-8 -*-

import logging
import logging.handlers
import os
from pathlib import Path
from datetime import datetime

class Logger:
    """日志管理器"""
    
    _loggers = {}
    _initialized = False
    
    @classmethod
    def initialize(cls, log_level='INFO', log_to_file=False, log_dir=None):
        """初始化日志系统"""
        if cls._initialized:
            return
        
        # 设置日志级别
        level = getattr(logging, log_level.upper(), logging.INFO)
        
        # 配置根日志器
        root_logger = logging.getLogger()
        root_logger.setLevel(level)
        
        # 清除现有处理器
        for handler in root_logger.handlers[:]:
            root_logger.removeHandler(handler)
        
        # 创建格式器
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        # 控制台处理器
        console_handler = logging.StreamHandler()
        console_handler.setLevel(level)
        console_handler.setFormatter(formatter)
        root_logger.addHandler(console_handler)
        
        # 文件处理器
        if log_to_file:
            if log_dir is None:
                log_dir = Path.home() / ".robot_controller" / "logs"
            else:
                log_dir = Path(log_dir)
            
            log_dir.mkdir(parents=True, exist_ok=True)
            
            # 创建按日期滚动的文件处理器
            log_file = log_dir / f"robot_controller_{datetime.now().strftime('%Y%m%d')}.log"
            
            file_handler = logging.handlers.RotatingFileHandler(
                log_file, maxBytes=10*1024*1024, backupCount=5, encoding='utf-8'
            )
            file_handler.setLevel(level)
            file_handler.setFormatter(formatter)
            root_logger.addHandler(file_handler)
        
        cls._initialized = True
    
    @classmethod
    def get_logger(cls, name):
        """获取日志器"""
        if not cls._initialized:
            cls.initialize()
        
        if name not in cls._loggers:
            logger = logging.getLogger(name)
            cls._loggers[name] = logger
        
        return cls._loggers[name]
    
    @classmethod
    def set_level(cls, level):
        """设置日志级别"""
        log_level = getattr(logging, level.upper(), logging.INFO)
        
        root_logger = logging.getLogger()
        root_logger.setLevel(log_level)
        
        for handler in root_logger.handlers:
            handler.setLevel(log_level)
    
    @classmethod
    def add_file_handler(cls, log_file, level='INFO'):
        """添加文件处理器"""
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        file_handler = logging.FileHandler(log_file, encoding='utf-8')
        file_handler.setLevel(getattr(logging, level.upper(), logging.INFO))
        file_handler.setFormatter(formatter)
        
        root_logger = logging.getLogger()
        root_logger.addHandler(file_handler)