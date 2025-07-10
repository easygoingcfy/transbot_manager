#!/bin/bash
# filepath: /home/jetson/transbot_ws/src/transbot_manager/scripts/fix_issues.sh

echo "=== 修复系统问题 ==="

cd ~/transbot_ws

# 清理旧的编译文件
echo "清理旧的编译文件..."
rm -rf build/ devel/

# 重新编译
echo "重新编译..."
catkin_make

# 重新source环境
source devel/setup.bash

# 检查话题类型一致性
echo "检查话题定义..."
echo "控制器发布的话题:"
grep -n "Publisher.*robot_status" src/transbot_manager/scripts/enhanced_motion_controller.py
echo "协调器订阅的话题:"
grep -n "Subscriber.*robot_status" src/transbot_manager/scripts/system_coordinator.py

echo "修复完成，可以重新启动系统"