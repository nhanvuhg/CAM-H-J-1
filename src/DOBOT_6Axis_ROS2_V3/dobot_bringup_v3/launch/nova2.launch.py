from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dobot_bringup_v3',
            executable='dobot_bringup',
            namespace='nova2',
            name='dobot_bringup',
            parameters=[
                {'robot_ip': '192.168.27.8'},
                {'robot_type': 'nova2'},
            ]
        ),
        Node(
            package='dobot_bringup_v3',
            executable='feedback',
            namespace='nova2',
            name='feedback',
            parameters=[
                {'robot_ip': '192.168.27.8'},
            ]
        ),
    ])