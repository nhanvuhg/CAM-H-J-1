#!/bin/bash
set -e

# Setup ROS 2 environment
source "/opt/ros/humble/setup.bash"

# Setup workspace environment if it exists
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source "/ros2_ws/install/setup.bash"
fi

# Execute the command passed to the container
exec "$@"
