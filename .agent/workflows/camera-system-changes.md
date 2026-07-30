---
description: Quy tắc khi sửa đổi hệ thống camera (dual CSI + YOLO)
---

# Camera System Change Policy

> ⚠️ **QUAN TRỌNG**: Hệ thống camera hiện tại ĐANG HOẠT ĐỘNG. Phải cẩn thận khi thay đổi!

## Trước khi sửa bất kỳ file nào

1. **Backup tất cả các file liên quan:**
   ```bash
   BACKUP_DIR=~/ros2_ws/backups/$(date +%Y%m%d_%H%M%S)
   mkdir -p $BACKUP_DIR
   cp ~/ros2_ws/src/csi_camera/src/dual_csi_camera_node.cpp $BACKUP_DIR/
   cp ~/ros2_ws/src/csi_camera/launch/dual_camera_system.launch.py $BACKUP_DIR/
   cp ~/ros2_ws/run_all_three.sh $BACKUP_DIR/
   cp ~/ros2_ws/stop_all.sh $BACKUP_DIR/
   echo "Backed up to: $BACKUP_DIR"
   ```

2. **Cảnh báo user nếu thay đổi có thể ảnh hưởng đến:**
   - Cách capture frames (rpicam-still vs rpicam-vid)
   - Topic names (/cam0HP/*, /cam1HP/*)
   - Launch file structure
   - I2C commands hoặc camera switching logic

## Kiến trúc hiện tại (WORKING)

```
dual_csi_camera_node.cpp:
  - Dùng rpicam-still (KHÔNG DÙNG rpicam-vid)
  - Không có I2C switching
  - 2 threads độc lập cho 2 cameras
  - Output: /cam0HP/image_raw, /cam1HP/image_raw

dual_camera_system.launch.py:
  - dual_csi_camera_node
  - yolo_container (2 YOLO nodes)
  - overlay_bboxes_node
```

## Nếu cần rollback

```bash
# Restore từ backup
cp ~/ros2_ws/backups/YYYYMMDD_HHMMSS/* ~/ros2_ws/src/csi_camera/src/
# Rebuild
cd ~/ros2_ws && colcon build --packages-select csi_camera
```
