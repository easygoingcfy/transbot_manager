#!/bin/bash

echo "=== 地图管理工具 ==="

MAP_DIR="$HOME/transbot_ws/src/transbot_nav/maps"

# 自动安装功能
install_tool() {
    echo "正在安装map_tools到系统..."
    
    # 创建bin目录
    mkdir -p ~/bin
    
    # 创建链接
    ln -sf "$(realpath $0)" ~/bin/map_tools
    
    # 添加到PATH
    if ! echo $PATH | grep -q "$HOME/bin"; then
        echo 'export PATH="$HOME/bin:$PATH"' >> ~/.bashrc
        echo "已添加~/bin到PATH"
        echo "请运行: source ~/.bashrc 或重新打开终端"
    fi
    
    echo "安装完成！现在可以直接使用: map_tools <command>"
}

show_maps() {
    echo "可用地图:"
    if [ ! -d "$MAP_DIR" ]; then
        echo "  错误: 地图目录不存在 $MAP_DIR"
        return 1
    fi
    
    ls $MAP_DIR/*.yaml 2>/dev/null | while read map_file; do
        map_name=$(basename "$map_file" .yaml)
        echo "  - $map_name"
    done
}

view_map() {
    local map_name=$1
    if [ -z "$map_name" ]; then
        echo "错误: 请指定地图名称"
        echo "用法: map_tools view <map_name>"
        return 1
    fi
    
    if [ ! -f "$MAP_DIR/$map_name.yaml" ]; then
        echo "错误: 地图文件不存在: $MAP_DIR/$map_name.yaml"
        return 1
    fi
    
    echo "启动地图查看器: $map_name"
    
    # 检查ROS环境
    if [ -z "$ROS_MASTER_URI" ]; then
        echo "设置ROS环境..."
        source /opt/ros/melodic/setup.bash
        source ~/transbot_ws/devel/setup.bash
    fi
    
    # 启动roscore（如果没运行）
    if ! pgrep rosmaster > /dev/null; then
        echo "启动roscore..."
        roscore &
        ROSCORE_PID=$!
        sleep 3
    fi
    
    # 启动地图服务器
    rosrun map_server map_server "$MAP_DIR/$map_name.yaml" &
    MAP_PID=$!
    
    # 启动rviz
    rviz &
    RVIZ_PID=$!
    
    echo "地图查看器已启动"
    echo "在rviz中添加Map显示，Topic选择/map"
    echo "按Enter关闭查看器..."
    read
    
    kill $MAP_PID $RVIZ_PID 2>/dev/null
    [ ! -z "$ROSCORE_PID" ] && kill $ROSCORE_PID 2>/dev/null
}

edit_map() {
    local map_name=$1
    if [ -z "$map_name" ]; then
        echo "错误: 请指定地图名称"
        echo "用法: map_tools edit <map_name>"
        return 1
    fi
    
    echo "启动地图编辑器: $map_name"

    EDITOR_PATH="$HOME/transbot_ws/src/transbot_manager/map/map_editor.py"
    
    # 检查是否有编辑器
    if [ -f "$EDITOR_PATH" ]; then
        python "$EDITOR_PATH" "$MAP_DIR/$map_name.yaml"
    else
        echo "地图编辑器未找到，使用GIMP打开图像文件..."
        eog "$MAP_DIR/$map_name.pgm" 2>/dev/null &
    fi
}

case "$1" in
    "list"|"ls")
        show_maps
        ;;
    "view"|"show")
        view_map "$2"
        ;;
    "edit")
        edit_map "$2"
        ;;
    "install")
        install_tool
        ;;
    *)
        echo "TransBot地图管理工具"
        echo ""
        echo "用法: $0 {list|view|edit|install} [map_name]"
        echo ""
        echo "命令:"
        echo "  list                   - 列出所有地图"
        echo "  view <name>           - 查看指定地图"
        echo "  edit <name>           - 编辑指定地图"
        echo "  install               - 安装为系统命令"
        echo ""
        echo "示例:"
        echo "  $0 list               # 列出所有地图"
        echo "  $0 view house         # 查看house地图"
        echo "  $0 edit house         # 编辑house地图"
        echo "  $0 install            # 安装为系统命令"
        echo ""
        echo "安装后可直接使用: map_tools list"
        ;;
esac