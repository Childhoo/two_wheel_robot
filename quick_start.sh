#!/bin/bash
# 快速开始脚本

echo "Two-Wheel Robot Package 快速开始"
echo "================================"

# 检查文件是否存在
if [ ! -f "package.xml" ]; then
    echo "❌ 错误: package.xml 文件未找到"
    echo "请先按照 FILE_PLACEMENT_GUIDE.txt 放置所有文件"
    exit 1
fi

# 进入工作空间根目录
cd "/root/ros2_ws"

# Source ROS2环境
source /opt/ros/humble/setup.bash

echo "✅ 正在安装依赖..."
cd src/two_wheel_robot
chmod +x install_dependencies.sh
./install_dependencies.sh

echo "✅ 正在构建包..."
cd "/root/ros2_ws"
colcon build --packages-select two_wheel_robot

echo "✅ 正在source工作空间..."
source install/setup.bash

echo "✅ 启动仿真..."
ros2 launch two_wheel_robot robot_simulation.launch.py
