# Two-Wheel Robot Simulation Package

这是一个完整的ROS2 package，用于仿真一个带相机的两轮机器人。该package包含了机器人模型、仿真环境、控制程序以及可视化工具。

## 功能特性

- 🤖 **机器人仿真**: 完整的两轮差动驱动机器人模型
- 📷 **相机集成**: 机器人配备前向相机，可实时输出图像
- 🎮 **手动控制**: 支持键盘遥控操作
- 🧠 **自动导航**: 内置多种自动移动模式
- 📊 **实时监控**: 显示机器人状态和运动参数
- 🔍 **图形可视化**: RViz可视化和计算图显示

## 系统要求

- ROS2 (Humble或更新版本)
- Gazebo Classic
- Python 3.8+
- 以下ROS2包:
  - `gazebo_ros_pkgs`
  - `robot_state_publisher`
  - `teleop_twist_keyboard`
  - `rviz2`
  - `rqt_*` 工具包

## 安装步骤

1. **创建工作空间**:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. **复制文件结构**:
```
two_wheel_robot/
├── CMakeLists.txt
├── package.xml
├── README.md
├── src/
│   ├── robot_monitor.cpp
│   └── autonomous_navigator.cpp
├── launch/
│   ├── robot_simulation.launch.py
│   └── visualize_graph.launch.py
├── urdf/
│   └── two_wheel_robot.urdf.xacro
├── config/
│   └── robot_view.rviz
└── worlds/
    └── simple_world.world
```

3. **编译包**:
```bash
cd ~/ros2_ws
colcon build --packages-select two_wheel_robot
source install/setup.bash
```

## 使用方法

### 基本仿真启动

启动完整的机器人仿真环境：

```bash
ros2 launch two_wheel_robot robot_simulation.launch.py
```

这将启动：
- Gazebo仿真环境
- 机器人模型
- RViz可视化
- 相机图像查看器
- 键盘控制终端
- 机器人状态监控器

### 仅启动自动导航

如果你想要机器人自动移动：

```bash
ros2 launch two_wheel_robot robot_simulation.launch.py use_autonomous:=true use_teleop:=false
```

### 手动键盘控制

在键盘控制终端中使用以下按键：
- `i`: 前进
- `k`: 停止
- `j`: 左转
- `l`: 右转
- `u`: 前进左转
- `o`: 前进右转
- `m`: 后退左转
- `.`: 后退右转
- `,`: 后退
- `q/z`: 增加/减少线速度
- `w/x`: 增加/减少角速度

### 可视化计算图

查看ROS2节点和话题的关系图：

```bash
ros2 launch two_wheel_robot visualize_graph.launch.py
```

## 主要组件说明

### 1. 机器人模型 (URDF)
- **文件**: `urdf/two_wheel_robot.urdf.xacro`
- **功能**: 定义机器人的物理结构，包括底盘、轮子和相机
- **传感器**: 配置了差动驱动控制器和相机插件

### 2. 仿真环境 (Gazebo World)
- **文件**: `worlds/simple_world.world`
- **内容**: 包含地面平面和几个障碍物供导航测试

### 3. 状态监控器
- **文件**: `src/robot_monitor.cpp`
- **功能**: 实时显示机器人的运动参数、位置信息和相机状态
- **输出**: 终端中的格式化状态信息

### 4. 自动导航器
- **文件**: `src/autonomous_navigator.cpp`
- **模式**: 支持多种自动移动模式：
  - 直线前进
  - 圆形运动
  - 8字形轨迹
  - 随机游走
  - 螺旋运动

## 话题和服务

### 发布的话题
- `/cmd_vel` (geometry_msgs/Twist): 速度控制指令
- `/odom` (nav_msgs/Odometry): 里程计信息
- `/robot_camera/image_raw` (sensor_msgs/Image): 相机图像
- `/robot_camera/camera_info` (sensor_msgs/CameraInfo): 相机参数

### 订阅的话题
- `/cmd_vel`: 接收控制指令
- TF变换: 机器人各部件的坐标变换

## RViz可视化

RViz配置文件包含以下显示项：
- 机器人3D模型
- TF坐标系
- 里程计轨迹
- 相机图像
- 网格参考

## 自定义和扩展

### 修改机器人参数
编辑 `urdf/two_wheel_robot.urdf.xacro` 中的xacro属性：
```xml
<xacro:property name="wheel_radius" value="0.1"/>
<xacro:property name="wheel_separation" value="0.35"/>
<xacro:property name="chassis_length" value="0.4"/>
```

### 添加新的导航模式
在 `src/autonomous_navigator.cpp` 中添加新的NavigationPattern枚举值和对应的执行函数。

### 环境定制
修改 `worlds/simple_world.world` 文件来添加新的障碍物或改变环境布局。

## 故障排除

### 常见问题

1. **Gazebo启动失败**:
```bash
# 检查gazebo安装
gazebo --version
# 重新安装gazebo_ros_pkgs
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs
```

2. **无法看到机器人模型**:
```bash
# 检查URDF语法
check_urdf ~/ros2_ws/install/two_wheel_robot/share/two_wheel_robot/urdf/two_wheel_robot.urdf.xacro
```

3. **键盘控制无响应**:
   - 确保键盘控制终端窗口处于激活状态
   - 检查 `/cmd_vel` 话题是否正常发布：
   ```bash
   ros2 topic echo /cmd_vel
   ```

4. **相机图像不显示**:
   - 检查相机话题：
   ```bash
   ros2 topic list | grep camera
   ros2 topic hz /robot_camera/image_raw
   ```

5. **TF变换错误**:
   - 查看TF树：
   ```bash
   ros2 run tf2_tools view_frames.py
   ```

### 性能优化

- 如果仿真运行缓慢，可以在启动时禁用某些可视化组件
- 调整Gazebo的物理引擎参数以获得更好的性能

## 扩展功能建议

### 1. 传感器集成
可以添加更多传感器：
- 激光雷达 (LiDAR)
- IMU (惯性测量单元)
- 超声波传感器

### 2. 导航算法
集成更高级的导航功能：
- SLAM (同时定位与建图)
- 路径规划算法
- 障碍物避免

### 3. 机器学习集成
- 使用深度学习进行视觉导航
- 强化学习控制策略

## 参考资料

- [ROS2官方文档](https://docs.ros.org/en/humble/)
- [Gazebo经典版教程](http://gazebosim.org/tutorials)
- [URDF教程](http://wiki.ros.org/urdf/Tutorials)
- [RViz2用户指南](https://github.com/ros2/rviz)

## 许可证

本项目采用MIT许可证。详见LICENSE文件。

## 贡献

欢迎提交Issue和Pull Request来改进这个项目。

## 联系方式

如有问题，请通过GitHub Issues联系。

---

**注意**: 首次运行可能需要下载Gazebo模型，这可能需要一些时间。请确保网络连接正常。