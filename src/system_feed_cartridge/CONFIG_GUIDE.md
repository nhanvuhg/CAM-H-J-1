# 📋 Hướng Dẫn Cấu Hình Hệ Thống Cartridge

## 🎯 Mục Đích
Hướng dẫn điều chỉnh các thông số vị trí cho 5 servo motors trong hệ thống nạp cartridge.

---

## 📁 Files Quan Trọng

| File | Mô Tả |
|------|-------|
| `config/cartridge_config.yaml` | **File cấu hình chính** - Chứa tất cả thông số vị trí |
| `read_servo_positions.py` | **Tool đọc vị trí** - Đọc vị trí hiện tại của servo |
| `scripts/cartridge_system_py_node.py` | **ROS 2 Node** - Chương trình điều khiển chính |

---

## 🔧 Quy Trình Cấu Hình

### **Bước 1: Home Tất Cả Servos**

Trước khi đo vị trí, cần home tất cả servos:

```bash
cd ~/
python3 test_festo_servos.py
# Nhấn Enter để home tất cả servos
# Sau khi home xong, nhấn Ctrl+C để thoát
```

### **Bước 2: Đo Vị Trí Các Servo**

Sử dụng tool `read_servo_positions.py`:

```bash
python3 read_servo_positions.py
```

**2 chế độ:**
1. **Đọc tất cả (chế độ 1)**: Đọc vị trí hiện tại của tất cả 5 servos một lần
2. **Chế độ tương tác (chế độ 2)**: Theo dõi vị trí của 1 servo liên tục

#### **Ví dụ: Đo vị trí InX Target2**

1. Chạy tool ở chế độ tương tác (chọn 2)
2. Chọn Servo 1 (InX)
3. **Di chuyển servo thủ công** đến vị trí mà bạn muốn đo
4. Đọc giá trị vị trí hiện tại trên màn hình
5. Ghi lại giá trị (mm)
6. Nhấn Ctrl+C để dừng

### **Bước 3: Cập Nhật File Config YAML**

Mở file `config/cartridge_config.yaml` và cập nhật giá trị:

```yaml
# Ví dụ: Cập nhật vị trí InX Target2
inx_target2: 485.5  # mm - Vị trí đã đo được từ tool
```

### **Bước 4: Test Lại**

Sau khi cập nhật config, test lại bằng cách chạy ROS 2 node:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run system_feed_cartridge cartridge_system_py
```

---

## 📊 Danh Sách Vị Trí Cần Đo

### **Servo 1: InX (Trục X Đầu Vào)**
- [ ] `inx_home`: 0.0 mm (mặc định)
- [ ] `inx_target2`: Vị trí lấy khay đầu vào

### **Servo 2: InY (Trục Y Đầu Vào)**
- [ ] `iny_home`: 0.0 mm (mặc định)
- [ ] `iny_target2`: Vị trí an toàn để InX di chuyển
- [ ] `row_positions[1-8]`: Vị trí 8 hàng trên khay
  - [ ] Row 1 (thấp nhất)
  - [ ] Row 2
  - [ ] Row 3
  - [ ] Row 4
  - [ ] Row 5
  - [ ] Row 6
  - [ ] Row 7
  - [ ] Row 8 (cao nhất)

### **Servo 3: Put Tray (Đẩy Khay)**
- [ ] `servo3_home`: 0.0 mm (mặc định)

### **Servo 4: OutX (Trục X Đầu Ra)**
- [ ] `outx_home`: 0.0 mm (mặc định)
- [ ] `outx_target1`: Vị trí an toàn
- [ ] `outx_target2`: Vị trí lấy khay đã xử lý

### **Servo 5: OutY (Trục Y Đầu Ra)**
- [ ] `outy_home`: 0.0 mm (mặc định)
- [ ] `outy_target1`: Vị trí an toàn
- [ ] `outy_target2`: Vị trí đặt khay vào stack

---

## ⚠️ Lưu Ý Quan Trọng

### **1. Đơn Vị**
- File YAML sử dụng **mm** (millimeters)
- Code tự động convert sang **μm** (micrometers) khi gửi lệnh
- **1 mm = 1000 μm**

### **2. An Toàn**
- **Luôn home servo trước khi đo vị trí**
- Kiểm tra không va chạm khi di chuyển thủ công
- Test từng vị trí riêng lẻ trước khi chạy full system
- Điều chỉnh `safety_thresholds` để tránh va chạm giữa các trục

### **3. Độ Chính Xác**
- `position_tolerance`: ±5mm mặc định
- Nếu cần chính xác hơn, giảm giá trị này

### **4. Velocity**
- `iny_search_velocity`: Tốc độ tìm kiếm hàng
  - Thấp hơn = chính xác hơn nhưng chậm hơn
  - Cao hơn = nhanh hơn nhưng có thể bỏ lỡ

---

## 🚀 Quick Start

```bash
# 1. Home servos
python3 test_festo_servos.py

# 2. Đọc vị trí
python3 read_servo_positions.py

# 3. Sửa config
nano config/cartridge_config.yaml

# 4. Build & Test
cd ~/ros2_ws
colcon build --packages-select system_feed_cartridge
source install/setup.bash
ros2 run system_feed_cartridge cartridge_system_py
```

---

## 📝 Template Ghi Chú

Sử dụng template này để ghi lại các vị trí đã đo:

```
=== VỊ TRÍ ĐÃ ĐO - [Ngày/Tháng/Năm] ===

SERVO 1 - InX:
  Home: 0.0 mm
  Target2: _____ mm   (Vị trí khay đầu vào)

SERVO 2 - InY:
  Home: 0.0 mm
  Target2: _____ mm   (Vị trí an toàn)
  Row 1: _____ mm
  Row 2: _____ mm
  Row 3: _____ mm
  Row 4: _____ mm
  Row 5: _____ mm
  Row 6: _____ mm
  Row 7: _____ mm
  Row 8: _____ mm

SERVO 3 - Put Tray:
  Home: 0.0 mm
  Push Position: _____ mm   (Vị trí đẩy cho robot)

SERVO 4 - OutX:
  Home: 0.0 mm
  Target1: _____ mm   (Vị trí an toàn)
  Target2: _____ mm   (Vị trí khay đầu ra)

SERVO 5 - OutY:
  Home: 0.0 mm
  Target1: _____ mm   (Vị trí an toàn)
  Target2: _____ mm   (Vị trí stack)

GHI CHÚ:
- 
- 
```

---

## 🐛 Troubleshooting

### **Vấn đề: Servo không kết nối**
```bash
# Kiểm tra kết nối mạng
ping 172.16.11.35  # InX
ping 172.16.11.36  # InY
```

### **Vấn đề: Vị trí không chính xác**
- Kiểm tra servo đã home chưa
- Kiểm tra `position_tolerance` trong config
- Đo lại vị trí nhiều lần để verify

### **Vấn đề: Va chạm giữa các trục**
- Điều chỉnh `iny_safe_position_threshold`
- Điều chỉnh `outx_safe_position_threshold`
- Kiểm tra logic di chuyển trong state machine

---

## 📞 Support

Nếu gặp vấn đề, kiểm tra:
1. Log file: `/home/pi/ros2_ws/logs/`
2. Console output khi chạy node
3. Error messages có traceback chi tiết

---

**✅ Hoàn thành cấu hình! Chúc may mắn! 🚀**
