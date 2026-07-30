#!/usr/bin/env python3
"""
Launch file for robot control with Festo gripper integration
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Parameters for gripper node
    cpx_ip_arg = DeclareLaunchArgument(
        'cpx_ip',
        default_value='172.16.11.37',
        description='IP address of Festo CPX-AP module'
    )
    
    cpx_module_arg = DeclareLaunchArgument(
        'cpx_module',
        default_value='3',
        description='CPX module index (3 for VABX valve terminal)'
    )
    
    # Robot logic node (C++)
    robot_logic_node = Node(
        package='robot_control_main',
        executable='robot_logic_node',
        name='robot_logic_node',
        parameters=[
            'src/robot_control_main/config/joint_pose_params.yaml'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        cpx_ip_arg,
        cpx_module_arg,
        robot_logic_node
    ])
