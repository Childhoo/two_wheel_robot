from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # ROSBridge WebSocket服务器 (端口9090)
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0'
        }],
        output='screen'
    )
    
    # Web视频服务器 (端口8080)
    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{
            'port': 8080,
            'address': '0.0.0.0'
        }],
        output='screen'
    )
    
    return LaunchDescription([
        rosbridge_server,
        web_video_server
    ])