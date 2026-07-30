# Dual CSI Camera System - libcamera API (High Performance)

## 📊 Performance
| Metric | Value |
|--------|-------|
| FPS | ~30+ per camera |
| Latency | ~16ms |
| CPU Usage | ~8% |

## 📁 Contents

```
libcamera_dual_camera_system/
├── ARCHITECTURE.md      # Kiến trúc và cách hoạt động
├── SETUP_GUIDE.md       # Hướng dẫn setup chi tiết
├── README.md            # File này
└── backup/
    ├── libcamera_dual_node.cpp    # Camera node chính (libcamera C++)
    ├── dual_csi_camera_node.cpp   # Node cũ (rpicam-still) - reference
    ├── CMakeLists.txt             # Build configuration
    ├── package.xml                # ROS 2 package manifest
    ├── dual_camera_system.launch.py
    ├── system_csi_dual_model.launch.py  # YOLO launch
    ├── run_all_three_v2.sh        # Script khởi động hệ thống
    └── stop_all.sh                # Script dừng hệ thống
```

## 🚀 Quick Start

```bash
# 1. Copy files vào ROS 2 workspace
cp backup/libcamera_dual_node.cpp ~/ros2_ws/src/csi_camera/src/
cp backup/CMakeLists.txt ~/ros2_ws/src/csi_camera/

# 2. Build
cd ~/ros2_ws
colcon build --packages-select csi_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 3. Run
ros2 run csi_camera libcamera_dual_node --ros-args -p fps:=30
```

## ⚠️ Các điểm quan trọng

1. **Color conversion bắt buộc**: libcamera BGR888 = RGB → phải convert
2. **Disconnect signals trong destructor**: Tránh crash khi exit
3. **Mutex khi mmap**: Thread safety
4. **Kill processes trước khi chạy lại**: Tránh "Pipeline in use"

## 📖 Đọc thêm
- [ARCHITECTURE.md](ARCHITECTURE.md) - Chi tiết kiến trúc
- [SETUP_GUIDE.md](SETUP_GUIDE.md) - Hướng dẫn từng bước
