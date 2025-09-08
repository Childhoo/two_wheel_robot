#!/bin/bash

# 安装两轮机器人仿真package所需的所有依赖

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查ROS2分布
if [ -z "$ROS_DISTRO" ]; then
    print_error "ROS_DISTRO环境变量未设置。请先source ROS2 setup脚本。"
    echo "例如: source /opt/ros/humble/setup.bash"
    exit 1
fi

print_info "检测到ROS2分布: $ROS_DISTRO"

# 更新包管理器
print_info "更新包管理器..."
sudo apt update

# 安装基本依赖
print_info "安装ROS2基本依赖..."
sudo apt install -y \
    ros-$ROS_DISTRO-rclcpp \
    ros-$ROS_DISTRO-rclpy \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-nav-msgs

# 安装TF2依赖
print_info "安装TF2依赖..."
sudo apt install -y \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-tf2-tools

# 安装机器人相关包
print_info "安装机器人状态发布器和关节状态发布器..."
sudo apt install -y \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui

# 安装Gazebo和相关插件
print_info "安装Gazebo仿真环境..."
sudo apt install -y \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-gazebo-plugins

# 安装URDF和Xacro
print_info "安装URDF和Xacro工具..."
sudo apt install -y \
    ros-$ROS_DISTRO-urdf \
    ros-$ROS_DISTRO-xacro

# 安装RViz2
print_info "安装RViz2可视化工具..."
sudo apt install -y \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-rviz-common \
    ros-$ROS_DISTRO-rviz-default-plugins

# 安装遥控工具
print_info "安装键盘遥控工具..."
sudo apt install -y \
    ros-$ROS_DISTRO-teleop-twist-keyboard

# 安装图像处理和传输
print_info "安装图像处理依赖..."
sudo apt install -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-image-common \
    ros-$ROS_DISTRO-camera-info-manager

# 安装RQT工具
print_info "安装RQT调试工具..."
sudo apt install -y \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-rqt-graph \
    ros-$ROS_DISTRO-rqt-image-view \
    ros-$ROS_DISTRO-rqt-topic \
    ros-$ROS_DISTRO-rqt-console

# 安装构建工具
print_info "安装构建工具..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep

# 安装Python GUI依赖 (可选)
print_info "安装Python GUI依赖..."
sudo apt install -y \
    python3-tk \
    python3-pil \
    python3-pil.imagetk

# 初始化rosdep (如果还没有初始化)
if [ ! -d "/etc/ros/rosdep/sources.list.d" ]; then
    print_info "初始化rosdep..."
    sudo rosdep init
fi

print_info "更新rosdep数据库..."
rosdep update

print_success "所有依赖安装完成！"

# 验证关键组件
print_info "验证关键组件..."

# 检查Gazebo
if command -v gazebo >/dev/null 2>&1; then
    print_success "Gazebo: 已安装"
else
    print_error "Gazebo: 未找到"
fi

# 检查RViz2
if ros2 pkg list | grep -q rviz2; then
    print_success "RViz2: 已安装"
else
    print_error "RViz2: 未找到"
fi

# 检查tf2工具
if ros2 pkg list | grep -q tf2_tools; then
    print_success "TF2工具: 已安装"
else
    print_error "TF2工具: 未找到"
fi

echo
print_info "安装总结:"
echo "✅ ROS2基础消息类型"
echo "✅ TF2变换库"
echo "✅ Gazebo仿真环境"
echo "✅ RViz2可视化工具"
echo "✅ 机器人状态发布器"
echo "✅ 遥控工具"
echo "✅ 图像处理库"
echo "✅ RQT调试工具"
echo "✅ 构建工具"

echo
print_success "现在您可以构建和运行机器人仿真了！"
echo "下一步:"
echo "1. cd 到您的工作空间"
echo "2. 运行: colcon build --packages-select two_wheel_robot"
echo "3. 运行: source install/setup.bash"
echo "4. 运行: ros2 launch two_wheel_robot robot_simulation.launch.py"