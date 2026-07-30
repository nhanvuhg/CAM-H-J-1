# Hướng dẫn Setup Hệ thống Camera Dual CSI với libcamera

## Yêu cầu phần cứng
- Raspberry Pi 5 (4GB+ RAM khuyến nghị)
- 2x Camera CSI (IMX219, IMX477, hoặc tương thích)
- 2x Cáp ribbon 15-pin (22-pin to 15-pin nếu dùng adapter)
- Hailo-8L AI Accelerator (cho YOLO inference, optional)

## Yêu cầu phần mềm
- Raspberry Pi OS (Bookworm 64-bit)
- ROS 2 Jazzy
- libcamera development libraries
- OpenCV 4.x

---

## Bước 1: Cài đặt dependencies

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install libcamera development
sudo apt install -y libcamera-dev libcamera-ipa libcamera-tools

# Install OpenCV
sudo apt install -y libopencv-dev

# Verify libcamera works
libcamera-hello --list-cameras
# Expected output: 2 cameras detected
```

---

## Bước 2: Cấu hình /boot/firmware/config.txt

```bash
sudo nano /boot/firmware/config.txt
```

Thêm/sửa các dòng sau:
```ini
# Disable auto-detect to manually control cameras
camera_auto_detect=0

# Enable both CSI ports
dtoverlay=imx219,cam0
dtoverlay=imx219,cam1

# For IMX477 cameras, use:
# dtoverlay=imx477,cam0
# dtoverlay=imx477,cam1
```

Sau đó reboot:
```bash
sudo reboot
```

---

## Bước 3: Tạo ROS 2 Package

### 3.1 Tạo package
```bash
cd ~/ros2_ws/src
ros2 pkg create csi_camera \
  --build-type ament_cmake \
  --dependencies rclcpp sensor_msgs std_msgs cv_bridge OpenCV
```

### 3.2 Copy code vào package
```bash
# Copy từ backup
cp /path/to/backup/libcamera_dual_node.cpp ~/ros2_ws/src/csi_camera/src/
cp /path/to/backup/CMakeLists.txt ~/ros2_ws/src/csi_camera/
cp /path/to/backup/package.xml ~/ros2_ws/src/csi_camera/
```

### 3.3 CMakeLists.txt quan trọng

**File: `src/csi_camera/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(csi_camera)

# C++ 17 required for libcamera
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Find libcamera using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA libcamera)

if(LIBCAMERA_FOUND)
  message(STATUS "libcamera found: ${LIBCAMERA_VERSION}")
  
  # libcamera dual node
  add_executable(libcamera_dual_node src/libcamera_dual_node.cpp)
  
  target_include_directories(libcamera_dual_node PRIVATE
    ${LIBCAMERA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  )
  
  target_link_directories(libcamera_dual_node PRIVATE
    ${LIBCAMERA_LIBRARY_DIRS}
  )
  
  ament_target_dependencies(libcamera_dual_node
    rclcpp sensor_msgs std_msgs
  )
  
  target_link_libraries(libcamera_dual_node
    ${LIBCAMERA_LIBRARIES}
    ${OpenCV_LIBS}
  )
  
  install(TARGETS libcamera_dual_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "libcamera not found, skipping libcamera_dual_node")
endif()

ament_package()
```

---

## Bước 4: Build và Test

### 4.1 Build
```bash
cd ~/ros2_ws
colcon build --packages-select csi_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 4.2 Test camera node
```bash
# Kill any existing camera processes first
pkill -9 libcamera 2>/dev/null || true
pkill -9 rpicam 2>/dev/null || true

# Run camera node
ros2 run csi_camera libcamera_dual_node --ros-args -p fps:=30 -p width:=640 -p height:=480
```

### 4.3 Kiểm tra topics
```bash
# List camera topics
ros2 topic list | grep cam

# Check FPS
ros2 topic hz /cam0HP/image_raw
ros2 topic hz /cam1HP/image_raw

# Check status
ros2 topic echo /camera/status
```

---

## Bước 5: Tạo Script khởi động

### 5.1 run_all_three_v2.sh
Copy file `run_all_three_v2.sh` từ backup và chỉnh sửa theo nhu cầu:

```bash
cp /path/to/backup/run_all_three_v2.sh ~/ros2_ws/
chmod +x ~/ros2_ws/run_all_three_v2.sh
```

### 5.2 stop_all.sh
```bash
cp /path/to/backup/stop_all.sh ~/ros2_ws/
chmod +x ~/ros2_ws/stop_all.sh
```

---

## Bước 6: Các điểm quan trọng cần nhớ

### 6.1 Color Conversion (QUAN TRỌNG!)
libcamera trả về format `BGR888` nhưng thực tế là **RGB order**. Luôn convert:
```cpp
cv::cvtColor(frame, frame_safe, cv::COLOR_RGB2BGR);
```

### 6.2 Signal Disconnect trong Destructor
Phải disconnect signals **TRƯỚC** khi cleanup để tránh crash:
```cpp
~LibcameraDualNode() {
    running_ = false;
    // Disconnect signals FIRST
    camera0_->requestCompleted.disconnect(this, &processRequest);
    camera1_->requestCompleted.disconnect(this, &processRequest);
    // Then cleanup...
}
```

### 6.3 Mutex cho mmap
Dùng mutex để tránh race condition:
```cpp
std::mutex process_mutex_;

void processRequest(...) {
    std::lock_guard<std::mutex> lock(process_mutex_);
    // mmap operations here...
}
```

### 6.4 Cleanup khi dừng
Luôn kill tất cả processes khi dừng:
```bash
pkill -9 libcamera_dual
pkill -9 rpicam
killall -9 libcamera_dual_node
```

---

## Bước 7: Troubleshooting

| Lỗi | Nguyên nhân | Giải pháp |
|-----|-------------|-----------|
| "Pipeline handler in use" | Process cũ còn chạy | `pkill -9 libcamera`; reboot nếu cần |
| "Camera frontend has timed out" | Cáp lỏng/hỏng | Tắt Pi, kiểm tra cáp ribbon |
| "No cameras found" | config.txt sai | Kiểm tra dtoverlay settings |
| Color sai (xanh→vàng) | Không convert RGB→BGR | Thêm `cv::cvtColor(COLOR_RGB2BGR)` |
| Crash khi exit | Không disconnect signals | Disconnect signals trong destructor |

---

## Bước 8: Performance Tuning

### 8.1 Giảm FPS nếu cần
```bash
ros2 run csi_camera libcamera_dual_node --ros-args -p fps:=15
```

### 8.2 Thay đổi resolution
```bash
ros2 run csi_camera libcamera_dual_node --ros-args \
  -p width:=1280 -p height:=720 -p fps:=15
```

### 8.3 Release build (quan trọng cho performance)
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## Tham khảo

- [libcamera Documentation](https://libcamera.org/docs/)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Raspberry Pi Camera Documentation](https://www.raspberrypi.com/documentation/accessories/camera.html)
