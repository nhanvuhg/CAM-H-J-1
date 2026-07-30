from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ──────────────────────────────────────────────────────────────────────
# Nova5 driver bringup — single source of truth for robot connection.
#
# Connection protocol (fixed, do NOT override from start scripts):
#   - Dashboard/command : TCP port 29999 (request/response)
#   - Feedback streaming: TCP port 30004 (~100 Hz, raw 1440-byte frames)
#
# Both nodes share the SAME robot_ip via a single launch argument so the
# command channel and feedback channel can never drift apart.
# ──────────────────────────────────────────────────────────────────────

ROBOT_IP_DEFAULT = '172.16.11.34'
ROBOT_NAMESPACE = 'nova5'
ROBOT_TYPE = 'nova5'


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value=ROBOT_IP_DEFAULT,
        description='Nova5 controller IP (shared by dobot_bringup + feedback)',
    )
    robot_ip = LaunchConfiguration('robot_ip')

    return LaunchDescription([
        robot_ip_arg,
        Node(
            package='dobot_bringup_v3',
            executable='dobot_bringup',
            namespace=ROBOT_NAMESPACE,
            name='dobot_bringup',
            output='screen',
            parameters=[
                {'robot_ip': robot_ip},
                {'robot_type': ROBOT_TYPE},
            ],
        ),
        Node(
            package='dobot_bringup_v3',
            executable='feedback',
            namespace=ROBOT_NAMESPACE,
            name='feedback',
            output='screen',
            parameters=[
                {'robot_ip': robot_ip},
            ],
        ),
    ])
