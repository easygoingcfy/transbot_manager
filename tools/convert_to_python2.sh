#!/bin/bash
# filepath: /home/jetson/transbot_ws/src/transbot_manager/scripts/convert_to_python2.sh

echo "=== 转换为Python 2兼容 ==="

SCRIPTS_DIR="$(rospack find transbot_manager)/scripts"

# 需要修复的文件
FILES=(
    "task_composer.py"
    "task_parser.py"
    "cloud_task_receiver.py"
    "behavior_factory.py"
    "elevator_behavior.py"
    "enhanced_motion_controller.py"
    "system_coordinator.py"
    "task_status_manager.py"
    "atomic_behaviors.py"
    "robot_enums.py"
)

for file in "${FILES[@]}"; do
    if [ -f "$SCRIPTS_DIR/$file" ]; then
        echo "转换文件: $file"
        
        # 1. 修改shebang行
        sed -i '1s|#!/usr/bin/env python3|#!/usr/bin/env python|' "$SCRIPTS_DIR/$file"
        
        # 2. 添加编码声明（如果没有）
        if ! head -n 3 "$SCRIPTS_DIR/$file" | grep -q "coding.*utf-8"; then
            temp_file=$(mktemp)
            head -n 1 "$SCRIPTS_DIR/$file" > "$temp_file"
            echo "# -*- coding: utf-8 -*-" >> "$temp_file"
            tail -n +2 "$SCRIPTS_DIR/$file" >> "$temp_file"
            mv "$temp_file" "$SCRIPTS_DIR/$file"
        fi
        
        # 3. 替换f-string为.format()
        # f"text {var}" -> "text {}".format(var)
        sed -i 's/f"\([^"]*\){\([^}]*\)}\([^"]*\)"/"\1{}\3".format(\2)/g' "$SCRIPTS_DIR/$file"
        sed -i "s/f'\([^']*\){\([^}]*\)}\([^']*\)'/'\1{}\3'.format(\2)/g" "$SCRIPTS_DIR/$file"
        
        # 4. 替换super()为super(ClassName, self)
        if [ "$file" = "task_composer.py" ]; then
            sed -i 's/super().__init__(name)/super(ActionNode, self).__init__(name)/' "$SCRIPTS_DIR/$file"
            sed -i 's/super().cancel()/super(ActionNode, self).cancel()/' "$SCRIPTS_DIR/$file"
        fi
        
        # 5. 确保执行权限
        chmod +x "$SCRIPTS_DIR/$file"
        
        echo "  ✓ 转换完成"
    else
        echo "  ✗ 文件不存在: $file"
    fi
done

echo "=== 转换完成 ==="
echo "所有文件已转换为Python 2兼容格式"