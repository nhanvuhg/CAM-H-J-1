# Jetson TensorRT YOLO ROS 2 node

This package runs the two production YOLOv8 engines on the Jetson Orin GPU.
It accepts `sensor_msgs/Image` in BGR8, applies 640x640 letterboxing, runs
TensorRT, performs class-aware NMS, maps boxes back to the 640x360 camera
coordinates, and publishes `vision_msgs/Detection2DArray`.

Both CSI nodes capture and publish at 30 FPS. AI subscriptions use
SensorDataQoS depth=1, so inference always processes a recent frame rather
than accumulating latency if the GPU is temporarily busy.

Production models:

| Camera | Engine | Classes | ROS image | ROS detections |
| --- | --- | --- | --- | --- |
| Input tray | `/home/nhan/models/data_input_hp_final_2_fp16.engine` | `tray`, `cartridge` | `/cam0HP/image_raw` | `/cam0HP/yolo/bounding_boxes` |
| Output tray | `/home/nhan/models/data_output_hp_final_fp16.engine` | `tray`, `cartridge`, `cartridgefall` | `/cam1HP/image_raw` | `/cam1HP/yolo/bounding_boxes` |

Build:

```bash
cd /home/nhan/ros2_ws
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --executor sequential \
  --packages-select yolo_tensorrt_ros2 csi_camera bbox_drawer_cpp
```

Run the production camera and AI stack:

```bash
source /home/nhan/ros2_ws/install/setup.bash
ros2 launch csi_camera dual_camera_system.launch.py
```

Set `enable_inference:=false` only for camera-only diagnostics.
