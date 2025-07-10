#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
远程机器人控制器启动脚本
直接运行此文件启动应用程序
"""

import sys
import os

# 添加src目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.join(current_dir, 'src')
sys.path.insert(0, src_dir)

# 设置环境变量以避免相对导入问题
os.environ['PYTHONPATH'] = src_dir

def check_dependencies():
    """检查依赖包是否已安装"""
    required_packages = ['PyQt5', 'paho-mqtt', 'yaml']
    missing_packages = []
    
    for package in required_packages:
        try:
            if package == 'yaml':
                import yaml
            elif package == 'paho-mqtt':
                import paho.mqtt.client
            elif package == 'PyQt5':
                from PyQt5.QtWidgets import QApplication
        except ImportError:
            missing_packages.append(package)
    
    if missing_packages:
        print(f"缺少依赖包: {', '.join(missing_packages)}")
        print("请运行: pip install -r requirements.txt")
        return False
    
    return True

# 导入并运行主程序
if __name__ == '__main__':
    try:
        # 检查依赖
        if not check_dependencies():
            sys.exit(1)
        
        # 导入主程序
        from main import main
        sys.exit(main())
        
    except ImportError as e:
        print(f"导入错误: {e}")
        print("请检查以下事项:")
        print("1. 确保所有依赖都已正确安装: pip install -r requirements.txt")
        print("2. 确保当前目录为项目根目录")
        print("3. 检查Python版本是否为3.7+")
        sys.exit(1)
    except Exception as e:
        print(f"启动失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)