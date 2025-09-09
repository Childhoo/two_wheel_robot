# 直接进入容器版 Dockerfile
FROM osrf/ros:humble-desktop-full

# 安装必需包
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

# 设置环境变量，自动source
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "echo '🤖 ROS2环境已准备就绪'" >> /root/.bashrc && \
    echo "echo '运行: ros2 launch two_wheel_robot robot_simulation.launch.py use_rviz:=false use_autonomous:=true'" >> /root/.bashrc

# 直接进入bash
CMD ["/bin/bash"]