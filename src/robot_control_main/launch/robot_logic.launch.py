import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# ──────────────────────────────────────────────────────────────────────
# Robot logic bringup — motion_executor + robot_logic_node.
#
# All parameters (joint poses, robot_ip, safe_pose, motion sequence) are
# loaded here from the installed share/config so start_all.sh never has
# to pass --params-file.
#
# vision_decision_node is intentionally NOT started here; it is launched
# by dual_camera_system.launch.py alongside the camera pipeline.
# ──────────────────────────────────────────────────────────────────────

ROBOT_IP_DEFAULT = '172.16.11.34'


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_control_main')
    params_file = os.path.join(pkg_share, 'config', 'joint_pose_params.yaml')

    return LaunchDescription([
        Node(
            package='robot_control_main',
            executable='motion_executor',
            name='motion_executor',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='robot_control_main',
            executable='robot_logic_node',
            name='robot_logic_nova5',
            output='screen',
            parameters=[
                params_file,
                {'robot_ip': ROBOT_IP_DEFAULT},
            ],
        ),
    ])
