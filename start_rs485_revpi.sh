#!/bin/bash
# RS485 Bus Node Startup — RevPi A
# Rule: start script CHỈ chạy node. Tất cả params (port, baud, slave_id,
# ref_hz) đã set sẵn trong rs485_bus_node.cpp. ROS env (DOMAIN_ID, RMW,
# FastDDS) set trong /home/pi/ros2_env.sh — file này cũng source từ
# ~/.bashrc nên mọi terminal SSH có sẵn env, source ở đây chỉ guard
# trường hợp launch ngoài bashrc (tmux/cron).
[ -f /home/pi/ros2_env.sh ] && source /home/pi/ros2_env.sh
source /home/pi/ros2_jazzy/install/setup.bash 2>/dev/null

killall -9 rs485_bus_node 2>/dev/null
sleep 1

exec /home/pi/ros2_jazzy/install/com_rs485/lib/com_rs485/rs485_bus_node
