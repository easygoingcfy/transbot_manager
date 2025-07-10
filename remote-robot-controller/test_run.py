#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
简化的测试启动脚本
用于调试导入问题
"""

import sys
import os

# 添加src目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.join(current_dir, 'src')
sys.path.insert(0, src_dir)

print(f"当前目录: {current_dir}")
print(f"源代码目录: {src_dir}")
print(f"Python路径: {sys.path[:3]}")

try:
    print("测试导入 PyQt5...")
    from PyQt5.QtWidgets import QApplication
    print("✓ PyQt5 导入成功")
    
    print("测试导入 utils.constants...")
    from utils.constants import APP_NAME
    print(f"✓ constants 导入成功, APP_NAME: {APP_NAME}")
    
    print("测试导入 core.logger...")
    from core.logger import Logger
    print("✓ Logger 导入成功")
    
    print("测试导入 core.config_manager...")
    from core.config_manager import ConfigManager
    print("✓ ConfigManager 导入成功")
    
    print("测试导入 gui.main_window...")
    from gui.main_window import MainWindow
    print("✓ MainWindow 导入成功")
    
    print("\n所有关键模块导入成功！")
    print("现在尝试启动应用...")
    
    from main import main
    sys.exit(main())
    
except ImportError as e:
    print(f"✗ 导入失败: {e}")
    import traceback
    traceback.print_exc()
    
    # 显示详细的文件结构信息
    print(f"\n检查文件结构:")
    for root, dirs, files in os.walk(src_dir):
        level = root.replace(src_dir, '').count(os.sep)
        indent = ' ' * 2 * level
        print(f"{indent}{os.path.basename(root)}/")
        subindent = ' ' * 2 * (level + 1)
        for file in files:
            print(f"{subindent}{file}")
        if level > 2:  # 限制显示深度
            break

except Exception as e:
    print(f"✗ 其他错误: {e}")
    import traceback
    traceback.print_exc()