#!/bin/bash
# filepath: /home/jetson/transbot_ws/src/transbot_manager/scripts/check_python.sh

echo "=== 检查Python环境 ==="

# 检查默认Python版本
echo "默认Python版本:"
python --version

# 检查ROS Python环境
echo "ROS Python环境:"
which python
python -c "import rospy; print('ROS Python 2 可用')"

# 检查具体的Python模块
echo "检查关键模块:"
python -c "
try:
    import rospy
    print('✓ rospy')
except ImportError as e:
    print('✗ rospy: {}'.format(e))

try:
    from tf.transformations import quaternion_from_euler
    print('✓ tf.transformations')
except ImportError as e:
    print('✗ tf.transformations: {}'.format(e))

try:
    from geometry_msgs.msg import Twist
    print('✓ geometry_msgs')
except ImportError as e:
    print('✗ geometry_msgs: {}'.format(e))
"

echo "=== 检查完成 ==="