from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # --- Robot Nova 2 ---
        Node(
        #     package='dobot_bringup_v3',
        #     executable='dobot_bringup',
        #     name='dobot_bringup',
        #     namespace='nova2',
        #     parameters=[{'robot_ip': '192.168.27.8'}],
        #     output='screen'
        # ),
        # Node(
        #     package='dobot_bringup_v3',
        #     executable='feedback',
        #     name='feedback',
        #     namespace='nova2',
        #     parameters=[{'robot_ip': '192.168.27.8'}],
        #     output='screen'
        # ),

        # --- Robot Nova 5 ---
        Node(
            package='dobot_bringup_v3',
            executable='dobot_bringup',
            name='dobot_bringup',
            namespace='nova5',
            parameters=[{'robot_ip': '192.168.27.8'}],
            output='screen'
        ),
        Node(
            package='dobot_bringup_v3',
            executable='feedback',
            name='feedback',
            namespace='nova5',
            parameters=[{'robot_ip': '192.168.27.8'}],
            output='screen'
        ),
    ])