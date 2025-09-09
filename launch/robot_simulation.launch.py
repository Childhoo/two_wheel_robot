#!/usr/bin/env python3
"""
Launch file for two-wheel robot simulation with all visualization components
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Package directory
    pkg_share = FindPackageShare(package='two_wheel_robot').find('two_wheel_robot')
    
    # Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'two_wheel_robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'config', 'robot_view.rviz')
    world_file = os.path.join(pkg_share, 'worlds', 'simple_world.world')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_teleop = LaunchConfiguration('use_teleop')
    use_autonomous = LaunchConfiguration('use_autonomous')
    
    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    declare_use_teleop = DeclareLaunchArgument(
        'use_teleop',
        default_value='true',
        description='Whether to launch teleop keyboard control'
    )
    
    declare_use_autonomous = DeclareLaunchArgument(
        'use_autonomous',
        default_value='false',
        description='Whether to launch autonomous navigation'
    )
    
    # Robot description
    robot_description_content = Command([
        'xacro ', urdf_file
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros').find('gazebo_ros'), 
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'false'
        }.items()
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', 
                  '-entity', 'two_wheel_robot',
                  '-x', '0.0',
                  '-y', '0.0', 
                  '-z', '0.0'],
        output='screen'
    )
    
    # Robot monitor (for displaying robot status)
    robot_monitor_node = Node(
        package='two_wheel_robot',
        executable='robot_monitor',
        name='robot_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    # Image view node for camera display
    image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/robot_camera/image_raw'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_image_view)
    )    

    # Autonomous navigator (optional)
    autonomous_navigator_node = Node(
        package='two_wheel_robot',
        executable='autonomous_navigator',
        name='autonomous_navigator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_autonomous)
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )
    
    # Teleop node  
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        condition=IfCondition(use_teleop),
        prefix='xterm -e'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_use_teleop,
        declare_use_autonomous,
        
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_launch,
        spawn_robot,
        rviz_node,
        teleop_node,
        robot_monitor_node,
        autonomous_navigator_node,
        image_view_node
    ])