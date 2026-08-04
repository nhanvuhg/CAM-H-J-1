# YOLO-HailoRT-ROS2

> **LEGACY REFERENCE ONLY:** this directory is excluded by `COLCON_IGNORE`.
> The Jetson production system uses `yolo_tensorrt_ros2` with the two
> `/home/nhan/models/*.engine` files. Do not launch the `.hef` examples on
> this Jetson.
ref: https://github.com/hailo-ai/Hailo-Application-Code-Examples/tree/main/runtime/cpp/object_detection/general_detection_inference

YOLOX-ROS : https://github.com/Ar-Ray-code/YOLOX-ROS

## Requirements

- HailoRT 4.17.0
- ROS-Jazzy

## Usage

```bash
ros2 launch yolo_ros_hailort_cpp yolox_hailort.launch.py video_device:=/dev/video4 model_path:=./yolox_tiny.hef
```
