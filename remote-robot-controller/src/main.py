#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import signal
from PyQt5.QtWidgets import QApplication, QMessageBox
from PyQt5.QtCore import Qt

# 修复导入路径 - 使用绝对导入
try:
    from gui.main_window import MainWindow
    from core.logger import Logger
    from core.config_manager import ConfigManager
    from utils.constants import APP_NAME, APP_VERSION
except ImportError:
    # 如果绝对导入失败，尝试添加当前目录到路径
    import os
    current_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, current_dir)
    
    from gui.main_window import MainWindow
    from core.logger import Logger
    from core.config_manager import ConfigManager
    from utils.constants import APP_NAME, APP_VERSION

def signal_handler(sig, frame):
    """信号处理器"""
    try:
        logger = Logger.get_logger(__name__)
        logger.info("收到中断信号，正在关闭应用...")
    except:
        print("收到中断信号，正在关闭应用...")
    QApplication.quit()

def main():
    """主函数"""
    # 创建应用
    app = QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    app.setApplicationVersion(APP_VERSION)
    
    # 设置信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # 初始化配置管理器
        config_manager = ConfigManager()
        
        # 初始化日志系统
        ui_config = config_manager.get_ui_config()
        log_config = ui_config.get('logging', {})
        Logger.initialize(
            log_level=log_config.get('level', 'INFO'),
            log_to_file=log_config.get('to_file', False)
        )
        
        logger = Logger.get_logger(__name__)
        logger.info(f"启动 {APP_NAME} v{APP_VERSION}")
        
        # 验证配置
        is_valid, errors = config_manager.validate_config()
        if not is_valid:
            error_msg = "配置验证失败:\n" + "\n".join(errors)
            logger.error(error_msg)
            QMessageBox.critical(None, "配置错误", error_msg)
            return 1
        
        # 创建主窗口
        main_window = MainWindow()
        main_window.show()
        
        logger.info("应用启动完成")
        
        # 运行应用
        return app.exec_()
        
    except Exception as e:
        error_msg = f"应用启动失败: {e}"
        print(error_msg)
        try:
            logger = Logger.get_logger(__name__)
            logger.critical(error_msg)
        except:
            pass
        
        QMessageBox.critical(None, "启动错误", error_msg)
        return 1

if __name__ == '__main__':
    sys.exit(main())