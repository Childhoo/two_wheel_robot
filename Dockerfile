# 简化版 ROS2 机器人仿真 Dockerfile
FROM osrf/ros:humble-desktop-full

# 安装必需的包
RUN apt-get update && apt-get install -y \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# 创建工作空间
WORKDIR /root/ros2_ws
RUN mkdir -p src

# 复制项目文件
COPY . /root/ros2_ws/src/two_wheel_robot/

# 构建项目
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select two_wheel_robot"

# 创建简单启动脚本
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /root/ros2_ws/install/setup.bash\n\
echo "🤖 启动机器人仿真..."\n\
echo "模式: ${1:-auto}"\n\
case "${1:-auto}" in\n\
  auto)\n\
    ros2 launch two_wheel_robot robot_simulation.launch.py use_rviz:=false use_teleop:=false use_autonomous:=true\n\
    ;;\n\
  monitor)\n\
    ros2 run two_wheel_robot robot_monitor\n\
    ;;\n\
  *)\n\
    echo "可用模式: auto (默认), monitor"\n\
    ;;\n\
esac\n\
' > /root/start.sh && chmod +x /root/start.sh

CMD ["/root/start.sh"]