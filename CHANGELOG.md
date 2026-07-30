# 📋 CHANGELOG — Cartridge Filling Machine System

> File này ghi lại mọi thay đổi theo thời gian.
> **Vị trí lưu:**
> - Git: `ros2_ws/CHANGELOG.md` (branch: `refactor/vision_split_stage1`)
> - Pi local: `/home/pi/ros2_ws/CHANGELOG.md` + symlink `/home/pi/CHANGELOG.md`

---

## [2026-03-10] — Session: GUI Optimization + State 2 Fix

### 🎛️ GUI — `CartridgePage.qml`
- **Mode Selection**: Bỏ nút IDLE, chỉ giữ 2 nút **AUTO** và **MANUAL** (chia đôi bằng nhau)
- **Idle Guard**: Khi khởi động (chưa chọn mode), header hiện `⚠ SELECT MODE` nhấp nháy màu vàng
- **Disable Controls**: SYSTEM CONTROL và STATE NAVIGATION bị mờ/disabled cho đến khi chọn mode
- **2 nút bằng nhau**: Dùng `w = Math.floor((parent.width - 4) / 2)` thay vì `Layout.fillWidth`

### 🔧 Backend — `cartridge_providesystem_py_node.py`
| Thay đổi | Chi tiết |
|---|---|
| **State 2 flow** | Bỏ `S2_CONFIRM_SAFE` (pop-up cũ). Nhấn STATE 2 → tự kích `change_tray_input_callback()` → `S2_WAIT_TRIGGER` |
| **Topic thống nhất** | Gộp `/robot/change_tray` + `/vision/change_tray_input` → **`/change_tray_input`** (1 topic) |
| **Fix duplicate subscriptions** | `/system/start_button` và `/system/stop_button` bị subscribe 2 lần → **fix: chỉ 1 lần** |
| **Timer 50Hz → 20Hz** | `create_timer(0.02)` → `create_timer(0.05)` — giảm ~50% CPU |
| **Idle guard positions** | `_publish_positions` không đọc Modbus khi `operation_mode='idle'` + state IDLE |
| **_publish_config fix** | Bổ sung `inx_output_stack`, `iny_safe_zone`, `outx_target3`, `outy_safe_zone`, `servo3_target1` |
| **IO Module crash fix** | `CpxAp(cycle_time=0.5)` + `threading.excepthook` bắt `ConnectionAbortedError` |

### 🚀 Launcher — `start_all.sh`
- Web GUI (port 8080) **mặc định TẮT** để tiết kiệm ~70MB RAM
- Bật bằng flag: `bash start_all.sh --web`

### 🏗️ Kiến trúc tối ưu
- **CPU sau tối ưu**: GUI 25% + Cartridge 4% + Robot ~8% + AI Camera ~15% = **~52%** / 400% (RPi5)
- **RAM**: ~790MB / 8GB
- **Load average**: 0.78 → ✅ ổn định

---

## Template cho lần cập nhật tiếp theo

```
## [YYYY-MM-DD] — Session: <mô tả ngắn>

### 📁 File thay đổi
- `tên_file.py`: mô tả thay đổi

### 🔧 Chi tiết
| Thay đổi | Chi tiết |
|---|---|
| ... | ... |

### Git commit
- Branch: `...`
- Commit: `...`
```

---

*Cập nhật lần cuối: 2026-03-10 12:17 (GMT+7)*
