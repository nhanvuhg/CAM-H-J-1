#!/bin/bash
echo "🧹 Cleaning up ROS 2 and Hailo processes..."

# Kill ROS nodes
pkill -9 -f component_container
pkill -9 -f yolo_ros_hailort
pkill -9 -f csi_camera_node
pkill -9 -f ros2

# Kill only Hailo-related Python processes (not all python3)
pkill -9 -f "yolo_ros_hailort" 2>/dev/null || true
pkill -9 -f "csi_camera" 2>/dev/null || true

# Release /dev/hailo0 device lock
fuser -k /dev/hailo0 >/dev/null 2>&1

echo "✅ Cleanup complete. You can now relaunch."
