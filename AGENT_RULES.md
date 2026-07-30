# 🤖 AGENT RULES — Cartridge System (Raspberry Pi 5)
> Đọc file này TRƯỚC KHI làm bất kỳ thay đổi nào trong project này.

---

## ⚡ Quy tắc bắt buộc

### 1. Sau khi sửa Python node, PHẢI build lại
Node chạy từ thư mục **install**, không phải source. Chỉnh sửa source chưa đủ.

```bash
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select system_feed_cartridge --symlink-install
```

> ⚠️ **Bắt buộc mỗi lần** chỉnh sửa `cartridge_providesystem_py_node.py`!

### 2. Sau khi sửa QML, PHẢI build lại
```bash
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select unified_control_gui
```

### 3. "GUI" = QML, không phải HTML
- **Giao diện chính** = QML (`/unified_control_gui/qml/CartridgePage.qml`) — hiển thị trên màn hình HDMI
- **Giao diện phụ** = HTML (`cartridge_gui.py`, port 8080) — chỉ dùng debug từ xa
- Khi user nói "cập nhật giao diện" → **sửa QML**, không phải HTML.

### 4. Khởi động hệ thống
```bash
bash ~/ros2_ws/src/unified_control_gui/scripts/start_all.sh
```
Script tự kill process cũ và bật mới. Chạy lại để restart.

### 5. Không dùng sim_auto_test nữa
File `sim_auto_test.py` đã bị bỏ. Test trực tiếp trên hardware hoặc qua GUI.

---

## 📐 Thông số kỹ thuật quan trọng

### Servo / Axis
| Servo | Tên | IP |
|-------|-----|----|
| 1 | InX | 172.16.11.35 |
| 2 | InY | 172.16.11.36 |
| 3 | PutTray | 172.16.11.38 |
| 4 | OutX | 172.16.11.39 |
| 5 | OutY | 172.16.11.40 |
| IO 1 | IO Module | 172.16.11.37 |
| IO 2 | IO Module | 172.16.11.41 |

### Vị trí safe zone
- **InX home** = `20.0 mm` (Target 1, gần home)
- **InY home** = `10.0 mm` (Target 1, safe zone)
- **InY Target 2** = `200.0 mm` (vị trí đặt khay cho robot)
- **InX Target 2** = `500.0 mm` (cuối băng tải)

### Sensor Cylinder
| Sensor | Ý nghĩa |
|--------|---------|
| S10 | Cyl1**-** = Cylinder 1 **RETRACT** (đã nhả) |
| S11 | Cyl1**+** = Cylinder 1 **EXTEND** (đang gắp) |
| S12 | Cyl2**-** = Cylinder 2 **RETRACT** |
| S13 | Cyl2**+** = Cylinder 2 **EXTEND** |

### Sensor Stack / Conveyor
| Sensor | Ý nghĩa |
|--------|---------|
| S1 | Đầu băng tải (có khay) |
| S3 | Cuối băng tải (khay đã tới) |
| S4 | Phát hiện chồng khay custom (Pos 1) |

---

## 🔄 STATE1 — Logic trình tự chính xác

```
[S1 ON] → 2s delay → InX đến Target2 (500mm)
→ [InX dừng] → [S3 ON] → InY jog xuống (tăng mm)
→ [S4 ON @ Xmm] → Snap lên row TRÊN (row có pos ≥ X) → 2s delay → InY đến row đó
→ Extend Cylinder1 → 2s delay → [S11 ON] → 2s delay → InY về safe (10mm)
→ 2s delay → InY đến Target2 (200mm) → Retract Cylinder1 → 2s delay → [S10 ON]
→ 2s delay → InY về safe (10mm) → 2s delay → InX về safe (20mm)
→ STATE1_COMPLETE (chờ robot ACK)
```

**Quan trọng:**
- Khi S4 ON tại vị trí X mm, InY KHÔNG snap về nearest row mà snap lên **row đầu tiên có position ≥ X** (đếm lên, không quay về).
- Sau S11 ON, INY **BẮT BUỘC** về safe 10mm TRƯỚC khi đi Target2 = 200mm.
- Có **delay 2 giây** (`MOTION_DELAY` state) giữa mỗi bước motion.

---

## 🔒 Interlock an toàn

- **InX chỉ được di chuyển khi InY < 50mm** (iny_safe_zone)
- **InY không được jog xuống (tăng mm) khi S11 ON** (Cyl1 đang extend)
- **Cylinder timeout đã bị tắt** — chờ vô thời hạn đến khi sensor xác nhận

---

## 📁 File quan trọng

| File | Mục đích |
|------|----------|
| `src/system_feed_cartridge/scripts/cartridge_providesystem_py_node.py` | Node điều khiển chính — state machine |
| `src/system_feed_cartridge/config/cartridge_config.yaml` | Cấu hình vị trí servo, timeout |
| `src/unified_control_gui/qml/CartridgePage.qml` | Giao diện QML chính |
| `src/unified_control_gui/scripts/start_all.sh` | Script khởi động toàn bộ |
| `logs/cartridge_node.log` | Log realtime của node |
