#!/bin/bash
# filepath: /home/jetson/transbot_ws/src/transbot_manager/scripts/run_tests.sh

echo "=== TransBot 系统测试脚本 ==="

# 设置环境
source /opt/ros/melodic/setup.bash
source ~/transbot_ws/devel/setup.bash

# 检查必要的文件
echo "检查文件完整性..."
FILES=(
    "enhanced_motion_controller.py"
    "system_coordinator.py" 
    "cloud_task_receiver.py"
    "task_status_manager.py"
    "system_tester.py"
    "mock_sensors.py"
)

for file in "${FILES[@]}"; do
    if [ ! -f "$(rospack find transbot_manager)/scripts/$file" ]; then
        echo "错误: 找不到文件 $file"
        exit 1
    fi
done

echo "文件检查完成 ✓"

# 单独测试每个节点
echo "=== 单节点测试 ==="

test_node() {
    local node_name=$1
    local timeout=10
    
    echo "测试节点: $node_name"
    
    # 启动节点
    rosrun transbot_manager $node_name &
    local pid=$!
    
    # 等待节点启动
    sleep 3
    
    # 检查节点是否运行
    if rosnode list | grep -q $node_name; then
        echo "$node_name 启动成功 ✓"
        # 停止节点
        kill $pid 2>/dev/null
        sleep 1
    else
        echo "$node_name 启动失败 ✗"
        kill $pid 2>/dev/null
        return 1
    fi
    
    return 0
}

# 测试各个节点
test_node "enhanced_motion_controller.py"
test_node "system_coordinator.py"
test_node "mock_sensors.py"

echo "=== 集成测试 ==="

# 启动完整系统测试
echo "启动完整系统测试..."
roslaunch transbot_manager system_test.launch &
LAUNCH_PID=$!

# 等待测试完成
sleep 30

# 停止测试
kill $LAUNCH_PID 2>/dev/null

echo "=== 测试完成 ==="