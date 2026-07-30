"""Launch cartridge system node + GUI for testing."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node
import os


def generate_launch_description():
    gui_script = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'scripts', 'cartridge_gui.py'
    )

    return LaunchDescription([
        LogInfo(msg='========================================'),
        LogInfo(msg='  Starting Cartridge System + GUI'),
        LogInfo(msg='  GUI: http://localhost:8080'),
        LogInfo(msg='========================================'),

        # Cartridge providesystem Python node
        Node(
            package='system_feed_cartridge',
            executable='cartridge_providesystem_py',
            name='cartridge_providesystem',
            output='screen',
        ),

        # GUI web server (not a ROS node, run as process)
        ExecuteProcess(
            cmd=['python3', gui_script],
            name='cartridge_gui',
            output='screen',
        ),
    ])
