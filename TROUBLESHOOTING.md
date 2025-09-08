# 故障排除指南

## 🔧 编译错误

### 1. tf2_geometry_msgs 头文件找不到

**错误信息:**
```
fatal error: tf2_geometry_msgs/tf2_geometry_msgs.hpp: No such file or directory
```

**解决方案:**
```bash
# 安装缺失的依赖
sudo apt install ros-$ROS_DISTRO-tf2-geometry-msgs

# 或者运行完整的依赖安装脚本
./install_dependencies.sh
```

### 2. 包找不到错误

**错误信息:**
```
Package 'package_name' not found
```

**解决方案:**
```bash
# 检查ROS2环境是否正确设置
echo $ROS_DISTRO
source /opt/ros/$ROS_DISTRO/setup.bash

# 安装缺失的包
sudo apt install ros-$ROS_DISTRO-package-name

# 更新rosdep并安装所有依赖
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 构建失败

**解决方案:**
```bash
# 清理构建缓存
rm -rf build/ install/ log/

# 重新构建
colcon build --packages-select two_wheel_robot --cmake-clean-cache

# 如果还是失败，尝试详细输出
colcon build --packages-select two_wheel_robot --event-handlers console_direct+
```

## 🎮 运行时错误

### 1. Gazebo启动失败

**错误信息:**
```
[gazebo-1] process has died [pid xxx, exit code 1]
```

**解决方案:**
```bash
# 检查Gazebo是否正确安装
gazebo --version

# 重新安装Gazebo
sudo apt remove gazebo*
sudo apt install gazebo11 ros-$ROS_DISTRO-gazebo-ros-pkgs

# 清理Gazebo配置
rm -rf ~/.gazebo
```

### 2. 机器人模型不显示

**问题:** RViz或Gazebo中看不到机器人

**解决方案:**
```bash
# 检查robot_description话题
ros2 topic list | grep robot_description
ros2 topic echo /robot_description --once

# 手动测试URDF
check_urdf install/two_wheel_robot/share/two_wheel_robot/urdf/two_wheel_robot.urdf.xacro

# 检查TF变换
ros2 run tf2_tools view_frames
```

### 3. 相机图像不显示

**解决方案:**
```bash
# 检查相机话题
ros2 topic list | grep camera
ros2 topic hz /robot_camera/image_raw

# 手动启动图像查看器
ros2 run rqt_image_view rqt_image_view

# 检查Gazebo相机插件
ros2 topic echo /robot_camera/camera_info --once
```

### 4. 键盘控制无效

**解决方案:**
```bash
# 确保teleop窗口获得焦点
# 点击teleop终端窗口

# 手动测试cmd_vel话题
ros2 topic echo /cmd_vel

# 手动发送运动命令
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.1}}"

# 检查teleop_twist_keyboard是否安装
ros2 pkg list | grep teleop
```

### 5. TF变换错误

**错误信息:**
```
Could not transform from 'base_link' to 'odom'
```

**解决方案:**
```bash
# 检查TF树
ros2 run tf2_tools view_frames

# 检查机器人状态发布器
ros2 node list | grep robot_state_publisher

# 重启机器人状态发布器
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(ros2 topic echo /robot_description --once)"
```

## 🔍 调试工具

### 1. 话题监控
```bash
# 列出所有话题
ros2 topic list

# 检查话题类型
ros2 topic type /topic_name

# 实时监控话题
ros2 topic echo /topic_name

# 检查发布频率
ros2 topic hz /topic_name
```

### 2. 节点调试
```bash
# 列出所有节点
ros2 node list

# 检查节点信息
ros2 node info /node_name

# 检查节点间连接
ros2 run rqt_graph rqt_graph
```

### 3. 服务和参数
```bash
# 列出服务
ros2 service list

# 检查参数
ros2 param list
ros2 param get /node_name parameter_name
```

## 📊 性能问题

### 1. 仿真运行缓慢

**解决方案:**
```bash
# 降低Gazebo实时因子
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
gazebo --verbose

# 禁用不必要的可视化
ros2 launch two_wheel_robot robot_simulation.launch.py use_rviz:=false

# 使用轻量级物理引擎设置
# 编辑world文件中的physics参数
```

### 2. 高CPU使用率

**解决方案:**
```bash
# 检查系统资源
htop

# 降低更新频率
# 修改launch文件中的timer频率

# 关闭不需要的节点
ros2 node list
ros2 lifecycle set /node_name shutdown
```

## 🐛 常见配置问题

### 1. Python路径问题

**错误信息:**
```
ModuleNotFoundError: No module named 'rclpy'
```

**解决方案:**
```bash
# 确保ROS2 Python环境正确
source /opt/ros/$ROS_DISTRO/setup.bash
pip3 install setuptools==58.2.0

# 检查Python路径
python3 -c "import rclpy; print('ROS2 Python OK')"
```

### 2. 权限问题

**错误信息:**
```
Permission denied
```

**解决方案:**
```bash
# 给脚本执行权限
chmod +x setup_and_run.sh
chmod +x install_dependencies.sh

# 检查文件权限
ls -la *.sh
```

### 3. 网络显示问题

**错误信息:**
```
cannot connect to X server
```

**解决方案:**
```bash
# 设置显示
export DISPLAY=:0

# 如果使用SSH
ssh -X username@hostname

# Docker环境
docker run --net=host --env="DISPLAY" --volume="$HOME/.Xauth:/root/.Xauth:rw"
```

## 📞 获取帮助

如果以上方法都不能解决问题:

1. **检查日志输出**: 仔细阅读错误信息
2. **查看GitHub Issues**: 搜索类似问题
3. **ROS2官方文档**: https://docs.ros.org/en/humble/
4. **ROS Answers**: https://answers.ros.org/
5. **Gazebo论坛**: http://answers.gazebosim.org/

## 🔄 重新安装

如果所有方法都失败，可以完全重新安装:

```bash
# 清理工作空间
rm -rf build/ install/ log/

# 重新安装依赖
./install_dependencies.sh

# 重新构建
colcon build --packages-select two_wheel_robot

# 重新source环境
source install/setup.bash
```