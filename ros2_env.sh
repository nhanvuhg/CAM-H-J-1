# ROS 2 + FastDDS env — single source of truth.
# Source từ ~/.bashrc để mọi shell/process tự động có đúng env, không phụ
# thuộc vào start_all.sh hay start_rs485.sh.
#
# Jetson path: source ~/ros2_ws/ros2_env.sh
# RevPi A path remains /home/pi/ros2_env.sh (deploy_revpi.sh copy sang).
#
# Cài đặt 1 lần (idempotent):
#   grep -qxF 'source /home/pi/ros2_ws/ros2_env.sh' ~/.bashrc \
#     || echo 'source /home/pi/ros2_ws/ros2_env.sh' >> ~/.bashrc

export ROS_DOMAIN_ID=22
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Host của RevPi A — single source of truth cho IP RevPi A.
# Dùng cho SSH start rs485_bus_node + loadcell_node, và iframe web GUI loadcell.
# Đổi IP bằng /home/pi/Desktop/update_revpi_ip.sh để tự đồng bộ file này,
# fastdds_peers.xml, Start All và config trên RevPi A.
# Lưu ý: fastdds_peers.xml là XML static, FastDDS không support env-var trong
# <address>, nên updater sẽ sửa các locator tương ứng. Sanity check:
#   bash scripts/check_revpi_ip_sync.sh
export REVPI_A_HOST="172.16.11.31"

# Cross-host discovery: unicast peers (xem fastdds_peers.xml).
ROS2_WS_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$ROS2_WS_ENV_DIR/fastdds_peers.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$ROS2_WS_ENV_DIR/fastdds_peers.xml"
elif [ -f /home/pi/fastdds_peers.xml ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastdds_peers.xml
fi
unset ROS2_WS_ENV_DIR
