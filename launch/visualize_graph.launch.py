#!/usr/bin/env python3
"""
Launch file to visualize the ROS2 computation graph using rqt_graph
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # RQT Graph for visualizing computation graph
    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph',
        output='screen'
    )
    
    # RQT Console for log messages
    rqt_console_node = Node(
        package='rqt_console',
        executable='rqt_console',
        name='rqt_console',
        output='screen'
    )
    
    # RQT Topic monitor
    rqt_topic_node = Node(
        package='rqt_topic',
        executable='rqt_topic',
        name='rqt_topic',
        output='screen'
    )
    
    return LaunchDescription([
        rqt_graph_node,
        rqt_console_node,
        rqt_topic_node
    ])