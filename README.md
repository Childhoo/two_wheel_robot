# Two-Wheel Robot Simulation Package

This is a complete ROS2 package for simulating a two-wheel robot with camera. The package includes robot model, simulation environment, control programs, and visualization tools.

## Features

- 🤖 **Robot Simulation**: Complete two-wheel differential drive robot model
- 📷 **Camera Integration**: Robot equipped with forward-facing camera for real-time image output
- 🎮 **Manual Control**: Keyboard teleoperation support
- 🧠 **Autonomous Navigation**: Built-in multiple automatic movement patterns
- 📊 **Real-time Monitoring**: Display robot status and motion parameters
- 🔍 **Graphical Visualization**: RViz visualization and computation graph display

## System Requirements

- ROS2 (Humble or newer)
- Gazebo Classic
- Python 3.8+
- Following ROS2 packages:
  - `gazebo_ros_pkgs`
  - `robot_state_publisher`
  - `teleop_twist_keyboard`
  - `rviz2`
  - `rqt_*` toolkits

## Installation

1. **Create workspace**:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. **Copy file structure**:
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

3. **Build package**:
```bash
cd ~/ros2_ws
colcon build --packages-select two_wheel_robot
source install/setup.bash
```

## Usage

### Basic Simulation Launch

Launch complete robot simulation environment:

```bash
ros2 launch two_wheel_robot robot_simulation.launch.py
```

This will start:
- Gazebo simulation environment
- Robot model
- RViz visualization
- Camera image viewer
- Keyboard control terminal
- Robot status monitor

### Launch Autonomous Navigation Only

If you want the robot to move automatically:

```bash
ros2 launch two_wheel_robot robot_simulation.launch.py use_autonomous:=true use_teleop:=false
```

### Manual Keyboard Control

Use the following keys in the keyboard control terminal:
- `i`: Move forward
- `k`: Stop
- `j`: Turn left
- `l`: Turn right
- `u`: Forward + left turn
- `o`: Forward + right turn
- `m`: Backward + left turn
- `.`: Backward + right turn
- `,`: Move backward
- `q/z`: Increase/decrease linear velocity
- `w/x`: Increase/decrease angular velocity

### Visualize Computation Graph

View ROS2 nodes and topics relationship graph:

```bash
ros2 launch two_wheel_robot visualize_graph.launch.py
```

## Main Components

### 1. Robot Model (URDF)
- **File**: `urdf/two_wheel_robot.urdf.xacro`
- **Function**: Defines robot's physical structure including chassis, wheels, and camera
- **Sensors**: Configured with differential drive controller and camera plugin

### 2. Simulation Environment (Gazebo World)
- **File**: `worlds/simple_world.world`
- **Content**: Contains ground plane and several obstacles for navigation testing

### 3. Status Monitor
- **File**: `src/robot_monitor.cpp`
- **Function**: Real-time display of robot motion parameters, position information, and camera status
- **Output**: Formatted status information in terminal

### 4. Autonomous Navigator
- **File**: `src/autonomous_navigator.cpp`
- **Patterns**: Supports multiple automatic movement patterns:
  - Straight line forward
  - Circular motion
  - Figure-8 trajectory
  - Random walk
  - Spiral motion

## Topics and Services

### Published Topics
- `/cmd_vel` (geometry_msgs/Twist): Velocity control commands
- `/odom` (nav_msgs/Odometry): Odometry information
- `/robot_camera/image_raw` (sensor_msgs/Image): Camera images
- `/robot_camera/camera_info` (sensor_msgs/CameraInfo): Camera parameters

### Subscribed Topics
- `/cmd_vel`: Receive control commands
- TF transforms: Coordinate transformations between robot components

## RViz Visualization

RViz configuration file includes the following display items:
- Robot 3D model
- TF coordinate frames
- Odometry trajectory
- Camera images
- Grid reference

## Customization and Extension

### Modify Robot Parameters
Edit xacro properties in `urdf/two_wheel_robot.urdf.xacro`:
```xml
<xacro:property name="wheel_radius" value="0.1"/>
<xacro:property name="wheel_separation" value="0.35"/>
<xacro:property name="chassis_length" value="0.4"/>
```

### Add New Navigation Patterns
Add new NavigationPattern enum values and corresponding execution functions in `src/autonomous_navigator.cpp`.

### Environment Customization
Modify `worlds/simple_world.world` file to add new obstacles or change environment layout.

## Troubleshooting

### Common Issues

1. **Gazebo fails to start**:
```bash
# Check gazebo installation
gazebo --version
# Reinstall gazebo_ros_pkgs
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs
```

2. **Cannot see robot model**:
```bash
# Check URDF syntax
check_urdf ~/ros2_ws/install/two_wheel_robot/share/two_wheel_robot/urdf/two_wheel_robot.urdf.xacro
```

3. **Keyboard control unresponsive**:
   - Ensure keyboard control terminal window is active
   - Check if `/cmd_vel` topic is publishing normally:
   ```bash
   ros2 topic echo /cmd_vel
   ```

4. **Camera image not displaying**:
   - Check camera topics:
   ```bash
   ros2 topic list | grep camera
   ros2 topic hz /robot_camera/image_raw
   ```

5. **TF transform errors**:
   - View TF tree:
   ```bash
   ros2 run tf2_tools view_frames.py
   ```

### Performance Optimization

- If simulation runs slowly, disable certain visualization components at startup
- Adjust Gazebo physics engine parameters for better performance

## Extended Functionality Suggestions

### 1. Sensor Integration
Additional sensors can be added:
- LiDAR (Light Detection and Ranging)
- IMU (Inertial Measurement Unit)
- Ultrasonic sensors

### 2. Navigation Algorithms
Integrate more advanced navigation features:
- SLAM (Simultaneous Localization and Mapping)
- Path planning algorithms
- Obstacle avoidance

### 3. Machine Learning Integration
- Use deep learning for visual navigation
- Reinforcement learning control strategies

## References

- [ROS2 Official Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Classic Tutorials](http://gazebosim.org/tutorials)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [RViz2 User Guide](https://github.com/ros2/rviz)

## License

This project is licensed under the MIT License. See LICENSE file for details.

## Contributing

Issues and Pull Requests are welcome to improve this project.

## Contact

For questions, please contact through GitHub Issues.

---

**Note**: First run may require downloading Gazebo models, which may take some time. Please ensure stable network connection.