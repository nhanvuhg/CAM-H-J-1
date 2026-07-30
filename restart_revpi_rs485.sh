#!/bin/bash
# Restart rs485_bus_node on RevPi with correct DDS config
# IP RevPi A đọc từ $REVPI_A_HOST (single source of truth: ros2_env.sh).
source /home/pi/ros2_ws/ros2_env.sh
REVPI="${REVPI_A_HOST:-172.16.11.31}"

# Step 1: kill old
ssh -o ConnectTimeout=5 pi@$REVPI 'kill -9 $(pgrep -f rs485_bus_node) 2>/dev/null; echo step1_done'

sleep 1

# Step 2: check domain ID and env on RevPi
ssh -o ConnectTimeout=5 pi@$REVPI 'echo DOMAIN=$ROS_DOMAIN_ID; echo RMW=$RMW_IMPLEMENTATION; echo FASTRTPS=$FASTRTPS_DEFAULT_PROFILES_FILE; cat /etc/environment 2>/dev/null | grep ROS'

# Step 3: start node with correct env
ssh -o ConnectTimeout=5 pi@$REVPI 'export ROS_DOMAIN_ID=22 && export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastdds_no_shm.xml && source /home/pi/ros2_jazzy/install/setup.bash 2>/dev/null && nohup /home/pi/ros2_jazzy/install/com_rs485/lib/com_rs485/rs485_bus_node --ros-args -p port:=/dev/ttyRS485 -p baudrate:=9600 -p slave_id:=2 -p ref_hz:=-30.0 > /tmp/rs485_bus_node.log 2>&1 & echo PID=$!'

sleep 3

# Step 4: check log
ssh -o ConnectTimeout=5 pi@$REVPI 'cat /tmp/rs485_bus_node.log'

# Step 5: check from Pi5 side
echo "=== Pi5 check ==="
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
ros2 node list 2>/dev/null | grep rs485 && echo "rs485 node VISIBLE" || echo "rs485 node NOT visible"
ros2 topic info /vfd/cmd_run 2>/dev/null | grep -i sub
