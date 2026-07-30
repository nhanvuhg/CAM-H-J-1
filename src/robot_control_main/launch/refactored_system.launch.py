import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_control_main')
    params_file = os.path.join(pkg_dir, 'config', 'joint_pose_params.yaml')
    
    return LaunchDescription([
        # 1. Vision Node (Handles YOLO & ROI)
        Node(
            package='robot_control_main',
            executable='vision_decision_node',
            name='vision_decision_node',
            output='screen'
        ),
        
        # 2. Motion Executor (Handles Dobot Service Calls)
        Node(
            package='robot_control_main',
            executable='motion_executor',
            name='motion_executor',
            output='screen',
            parameters=[params_file]
        ),
        
        # 3. Robot Logic Node (State Machine Coordinator)
        Node(
            package='robot_control_main',
            executable='robot_logic_node',
            name='robot_logic_nova5',
            output='screen',
            parameters=[
                params_file,
                {'robot_ip': '172.16.11.34'}
            ]
        )
    ])
