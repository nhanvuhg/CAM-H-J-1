# Camera Tools — HP Robot

ROI calibration + capture helpers cho vision_decision_node.

## Files

| File | Mô tả |
|---|---|
| `camera_roi.py` | PyQt5 GUI chụp ảnh CSI ở **1280×720** (production scale) → `~/Pictures/roi/cam_roi_*.png` |
| `roi_picker.py` | Multi-ROI desktop picker (cv2 imshow). Click 24 corner (TRAY + R1..R5) → in C++ paste-ready coords |
| `roi_picker_web.py` | Flask web version (port 8090) cho SSH-only / IDE workflow |
| `camera_roi.sh` | Launcher wrapper cho `camera_roi.py` |
| `camera_roi.desktop` | Desktop shortcut |

## Workflow ROI

```bash
# 1. Chụp ảnh production scale
python3 camera_roi.py
# → ~/Pictures/roi/cam_roi_<ts>.png (1280×720)

# 2. Pick 4 corners × 6 ROIs (TRAY + ROW1..ROW5)
python3 roi_picker.py            # desktop (cần DISPLAY)
python3 roi_picker_web.py        # web (http://<pi-ip>:8090)
# → in toạ độ paste-ready vào vision_decision_node.cpp
```

## Install (deploy về `/home/pi/`)

Đặt symlink hoặc copy:
```bash
ln -sf $(pwd)/camera_roi.py     /home/pi/camera_roi.py
ln -sf $(pwd)/roi_picker.py     /home/pi/roi_picker.py
ln -sf $(pwd)/roi_picker_web.py /home/pi/roi_picker_web.py
ln -sf $(pwd)/camera_roi.sh     /home/pi/Desktop/camera_roi.sh
ln -sf $(pwd)/camera_roi.desktop /home/pi/Desktop/camera_roi.desktop
```

## Coordinate space

Toạ độ pick = **1280×720** (production camera publish space) =
`cam0HP/yolo/bounding_boxes` Detection2D bbox space. Dùng trực tiếp trong
`vision_decision_node.cpp` không cần scale.
