from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        # 1. Cartridge Feeding Control Node (Backend)
        Node(
            package='system_feed_cartridge',
            executable='cartridge_providesystem_py',
            name='cartridge_providesystem',
            output='screen',
            respawn=True,           # <--- TỰ BẬT LẠI NẾU SẬP
            respawn_delay=2.0,      # Chờ 2s rồi mới bật lại
            parameters=[{
                'use_sim_time': False
            }]
        ),

        # 2. Unified Control GUI (Frontend)
        Node(
            package='unified_control_gui',
            executable='unified_control_gui',
            name='gui_main',
            output='screen',
            respawn=True,           # <--- TỰ BẬT LẠI NẾU LỠ TAY TẮT
            respawn_delay=5.0,      # Chờ 5s rồi mới bật lại
            env={
                'QT_QPA_PLATFORM': 'xcb',  # Đảm bảo hiển thị trên X11/Desktop
                'DISPLAY': ':0'
            }
        ),
    ])
