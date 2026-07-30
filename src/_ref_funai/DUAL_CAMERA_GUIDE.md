# Dual Camera System - Quick Reference

> **Hardware**: Raspberry Pi 5 + 2× IMX477 (12.3MP) HQ Camera
> **Sensor Mode**: 2028×1520 @ 30fps (SRGGB12_CSI2P)
> **Output**: 1280×720 YUV420 → BGR8 via OpenCV
> **Last Updated**: 2026-04-21

## 🚀 Quick Start

### Run Dual Parallel Camera Mode (NEW)
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch csi_camera dual_camera.launch.py
```

### Run Switch-Based Camera Mode (BACKUP)
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run csi_camera csi_camera_node
```

## 📊 Topics

### Dual Camera Mode
```bash
# Camera streams
/cam0Funai/image_raw  # Input Tray (CAM0)
/cam1Funai/image_raw  # Output Tray (CAM1)

# YOLO detections
/cam0/detections      # Input Tray detections
/cam1/detections      # Output Tray detections

# Check rates
ros2 topic hz /cam0Funai/image_raw
ros2 topic hz /cam1Funai/image_raw
```

### Switch Mode (Backup)
```bash
# Camera streams
/cam0Funai/image_raw  # Currently active camera
/cam1Funai/image_raw  # (published to active stream)

# Control
ros2 topic pub -1 /robot/camera_select std_msgs/Int32 "{data: 0}"  # CAM0
ros2 topic pub -1 /robot/camera_select std_msgs/Int32 "{data: 1}"  # CAM1
```

## 📷 Camera Sensor Parameters (IMX477)

### rpicam-vid Command
```bash
rpicam-vid --camera {id} -t 0 --nopreview --codec yuv420 \
  --width 1280 --height 720 --framerate 30 \
  --mode 2028:1520:12:P \
  --denoise cdn_off --flush \
  --shutter 30000 --analoggain 8.0 --awbgains 1.5,1.8 \
  -o -
```

| Parameter | Value | Lý do |
|-----------|-------|--------|
| `--mode` | `2028:1520:12:P` | Native mode, tránh ISP scale overhead |
| `--shutter` | `30000` (30ms) | Fixed exposure, **KHÔNG dùng 33333** (xem bug bên dưới) |
| `--analoggain` | `8.0` | Fixed analog gain |
| `--awbgains` | `1.5,1.8` | Fixed white balance (Red=1.5, Blue=1.8, indoor) |
| `--denoise` | `cdn_off` | Tắt denoise để giảm latency |
| `--flush` | | Flush mỗi frame qua pipe ngay lập tức |

### IMX477 Timing Constraints (QUAN TRỌNG)
```
Sensor mode: 2028×1520 @ 30fps
Line duration: 15.17 μs
Total frame: 2197 lines = 33.32 ms
Max safe shutter: 2175 lines = 32.99 ms (~30600 μs)
VBLANK minimum: 128 lines
```

> ⚠️ **KHÔNG BAO GIỜ** đặt `--shutter` > 30600. Xem mục Bug Fix bên dưới.

---

## 🐛 Bug Fix: Camera Freeze khi vật thể lướt qua (2026-04-21)

### Triệu chứng
- Khi tay hoặc vật thể đi vào gần camera (dù chỉ 1 thoáng), camera **đơ ngay lập tức**
- Sau đó V4L2 bị treo hoàn toàn, phải reboot

### Nguyên nhân gốc: `--shutter 33333` vượt frame capacity

```
--shutter 33333 (33.333ms) → cần 2198 sensor lines
Frame capacity @ 30fps     → chỉ có 2197 lines
Chênh lệch                 → -1 line! VBLANK bị âm!
```

**Chuỗi sự kiện:**
1. Shutter 33333μs vượt frame period → driver tự tăng VBLANK → fps thực tế < 30
2. CFE hardware vẫn expect 30fps → timing luôn **trên bờ vực**
3. Vật thể lướt qua → ISP processing spike thêm vài μs → **CFE timeout ngay lập tức**
4. Code cũ: reconnect → reload kernel driver → V4L2 chết vĩnh viễn

### 4 Fix đã áp dụng

| # | Fix | Chi tiết |
|---|-----|----------|
| 1 | **Giảm shutter 33333→30000** | 30ms << max 30.6ms, cho ~129 lines VBLANK margin |
| 2 | **Tăng timeout 4s→8s** | Cho phép sensor recover từ occlusion ngắn |
| 3 | **Không reload kernel driver khi reconnect** | Chỉ reload sau ≥3 lần reconnect thất bại |
| 4 | **Fixed AWB `--awbgains 1.5,1.8`** | Tránh AWB hunting trên frame tối |

### Hành vi sau fix

| Tình huống | Kết quả |
|---|---|
| Vật thể lướt qua nhanh | ✅ Không ảnh hưởng (VBLANK đủ margin) |
| Che camera < 8s | ✅ Log warning, tự recover khi bỏ tay |
| Che camera > 8s | ✅ Reconnect nhẹ (chỉ restart rpicam-vid) |
| Reconnect thất bại 3+ lần | ✅ Reload kernel driver (biện pháp cuối) |

---

## 🔧 Troubleshooting

### No cameras detected
```bash
# Check hardware
rpicam-hello --list-cameras

# Should show:
# Available cameras:
# 0 : imx477 [4056x3040] (/base/...)
# 1 : imx477 [4056x3040] (/base/...)
```

If no cameras:
1. Check physical connections (ribbon cable)
2. Check `/boot/firmware/config.txt`
3. Reboot: `sudo reboot`

### Camera đơ / V4L2 freeze
```bash
# Check rpicam error logs
cat /tmp/rpicam_cam0.log
cat /tmp/rpicam_cam1.log

# Check kernel errors
dmesg | grep -i -E "cfe|imx477|dequeue|timeout"

# Kill and restart
pkill -9 rpicam-vid
pkill -9 csi_dual_camera_node
```

### Build issues
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select csi_camera --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install
source install/setup.bash
```

### Chỉnh white balance
Nếu màu bị lệch, chỉnh `--awbgains R,B` trong `csi_dual_camera_node.cpp`:
- Ám vàng/xanh → tăng Blue (vd: `1.5,2.0`)
- Ám đỏ → giảm Red (vd: `1.3,1.8`)
- Ám xanh dương → giảm Blue (vd: `1.5,1.5`)

## 📁 Files

### New Files (Dual Camera)
- `/home/pi/ros2_ws/src/csi_camera/src/csi_dual_camera_node.cpp`
- `/home/pi/ros2_ws/src/csi_camera/launch/dual_camera.launch.py`

### Backup Files (Switch Camera)
- `/home/pi/ros2_ws/src/csi_camera/src/csi_camera_node.cpp`
- `/home/pi/ros2_ws/src/csi_camera/launch/full_camera_system.launch.py`

## 🧪 Testing

Run verification script:
```bash
cd ~/ros2_ws
./test_dual_camera.sh
```

## 🔄 Switching Between Modes

Just use different launch files:
```bash
# Dual mode
ros2 launch csi_camera dual_camera.launch.py

# Single mode (backup)
ros2 launch csi_camera full_camera_system.launch.py
```

Both executables are installed and ready to use!
