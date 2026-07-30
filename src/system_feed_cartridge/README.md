# 📚 TÀI LIỆU HỆ THỐNG NẠP CARTRIDGE - INDEX

## 🎯 Tổng Quan

Hệ thống tự động nạp cartridge với 5 servo motors Festo CMMT-AS điều khiển qua ROS 2.

**Status:** ✅ Sẵn sàng cấu hình và chạy thử nghiệm

---

## 📖 Danh Mục Tài Liệu

### **1. Mô Tả Hệ Thống** 🏗️
📄 **[SYSTEM_DESCRIPTION.md](./SYSTEM_DESCRIPTION.md)** *(15KB)*

**Nội dung:**
- ✅ Kiến trúc phần cứng (5 servos + IO module)
- ✅ State machine chi tiết (State 1 & 2)
- ✅ ROS 2 topics
- ✅ API reference
- ✅ Workflow tổng thể
- ✅ Collision avoidance logic

**Đọc đầu tiên để hiểu hệ thống hoạt động như thế nào!**

---

### **2. Sơ Đồ Vật Lý** 📐
📄 **[PHYSICAL_LAYOUT.md](./PHYSICAL_LAYOUT.md)** *(18KB)*

**Nội dung:**
- ✅ Sơ đồ top view ASCII art
- ✅ Chi tiết Input Area (InX, InY)
- ✅ Chi tiết Output Area (OutX, OutY)
- ✅ Servo 3 mechanism
- ✅ Collision zones
- ✅ Sensor layout
- ✅ Network topology

**Xem để hiểu bố trí vật lý và vị trí các servo!**

---

### **3. Hướng Dẫn Cấu Hình** ⚙️
📄 **[CONFIG_GUIDE.md](./CONFIG_GUIDE.md)** *(5.7KB)*

**Nội dung:**
- ✅ Quy trình 4 bước cấu hình
- ✅ Checklist 13 vị trí cần đo
- ✅ Hướng dẫn sử dụng tool đọc vị trí
- ✅ Template ghi chú
- ✅ Troubleshooting

**Làm theo guide này để cấu hình vị trí servo!**

---

### **4. Tổng Quan YAML Config** 📝
📄 **[/home/pi/YAML_CONFIG_SUMMARY.md](/home/pi/YAML_CONFIG_SUMMARY.md)** *(6.1KB)*

**Nội dung:**
- ✅ Giải thích hệ thống YAML
- ✅ Cách dùng tool đọc vị trí
- ✅ Ví dụ đo vị trí row positions
- ✅ Quy trình nhanh
- ✅ Troubleshooting YAML

**Đọc để hiểu cách config hoạt động!**

---

### **5. API Fix Summary** 🔧
📄 **[/home/pi/FESTO_API_FIX_SUMMARY.md](/home/pi/FESTO_API_FIX_SUMMARY.md)** *(4.4KB)*

**Nội dung:**
- ✅ Protocol discovery (Modbus TCP)
- ✅ API method mapping
- ✅ Import statements fix
- ✅ Test results
- ✅ Next steps

**Reference khi cần hiểu API thay đổi!**

---

## 🗂️ Files Quan Trọng

### **Configuration:**
```
config/cartridge_config.yaml          ← File cấu hình chính (YAML)
scripts/cartridge_system_py_node.py   ← ROS 2 Node Python
CMakeLists.txt                        ← Build config
package.xml                           ← ROS 2 package info
```

### **Tools:**
```
/home/pi/read_servo_positions.py      ← Tool đọc vị trí servo
/home/pi/test_festo_servos.py         ← Test script cho servos
```

---

## 🚀 Quick Start

### **1. Đọc Tài Liệu**
```bash
# Mô tả hệ thống
cat ~/ros2_ws/src/system_feed_cartridge/SYSTEM_DESCRIPTION.md

# Layout vật lý
cat ~/ros2_ws/src/system_feed_cartridge/PHYSICAL_LAYOUT.md

# Hướng dẫn config
cat ~/ros2_ws/src/system_feed_cartridge/CONFIG_GUIDE.md
```

### **2. Cấu Hình Vị Trí**
```bash
# Home servos
python3 ~/test_festo_servos.py

# Đọc vị trí
python3 ~/read_servo_positions.py

# Sửa config
nano ~/ros2_ws/src/system_feed_cartridge/config/cartridge_config.yaml
```

### **3. Build & Run**
```bash
cd ~/ros2_ws
colcon build --packages-select system_feed_cartridge
source install/setup.bash
ros2 run system_feed_cartridge cartridge_system_py
```

---

## 📊 Tóm Tắt Thông Số

### **Hardware:**
- **5 Servo Motors**: Festo CMMT-AS-C2-3A-MP-S1
- **Protocol**: Modbus TCP/IP
- **IO Module**: Festo CPX-AP (EtherNet/IP)
- **Network**: 172.16.11.x

### **Servos:**
| ID | Name | IP | Function |
|----|------|-----|----------|
| 1 | InX | .247 | Trục X đầu vào |
| 2 | InY | .248 | Trục Y đầu vào (8 rows) |
| 3 | Put Tray | .103 | Đẩy khay cho robot |
| 4 | OutX | .104 | Trục X đầu ra |
| 5 | OutY | .105 | Trục Y đầu ra |

### **Positions to Configure:**
- ⚠️ **13 vị trí CRITICAL** cần đo
- ✅ Default values trong YAML
- 📐 Unit: mm (auto-convert to μm)

---

## 🎯 Workflow Summary

```
START → HOME → WAIT_FOR_TRAY
              ↓
         [STATE 1: Input Loading]
              ↓
         InX → Target2
              ↓
         InY → Search & Count Trays
              ↓
         Loop 8 Rows:
           InY → Row Position
           Cylinder Extend
           InY → Home
           Cylinder Retract
              ↓
         InX → Home
              ↓
         TRAY_LOADED
              ↓
         [STATE 2: Retrieve] (On Signal)
              ↓
         OutX & OutY → Get Done Tray
              ↓
         Return to Rows
              ↓
         Back to WAIT
```

---

## ⚙️ Key Features

✅ **State Machine**: 2 main states (Input + Retrieve)  
✅ **Auto Stack Detection**: 1-8 trays  
✅ **Collision Avoidance**: Safety thresholds  
✅ **Flexible Config**: YAML-based positions  
✅ **Error Handling**: Detailed traceback  
✅ **ROS 2 Integration**: Topics & services  

---

## 🔍 Chi Tiết Kỹ Thuật

### **API:**
```python
# Servo Control
ComModbus + MotionHandler
- referencing_task()      # Home
- position_task()         # Move
- velocity_task()         # Jog
- get_actual_position()   # Read

# IO Module
CpxAp
- get_channel()           # Read sensor
- set_channel()           # Activate
- reset_channel()         # Deactivate
```

### **Units:**
- Config: **mm** (millimeters)
- Internal: **μm** (micrometers)
- Auto conversion: 1mm = 1000μm

### **Parameters:**
```yaml
Position Tolerance: ±5mm
Velocities:
  - Default: 300 mm/s
  - Search: 30 mm/s
  - Jog: 50 mm/s
Timeouts:
  - Homing: 30s
  - Move: 20s
  - Cylinder: 5s
```

---

## 📞 Troubleshooting

### **Common Issues:**

1. **Servo không kết nối**
   - Check network: `ping 172.16.11.35`
   - Check library: `pip list | grep festo`

2. **Config không load**
   - Check path: `ls config/cartridge_config.yaml`
   - Check syntax: YAML format

3. **Vị trí không chính xác**
   - Check servo đã home
   - Check tolerance setting
   - Đo lại vị trí

---

## 📝 Checklist Trước Khi Chạy

- [ ] Đã đọc `SYSTEM_DESCRIPTION.md`
- [ ] Đã đọc `PHYSICAL_LAYOUT.md`
- [ ] Đã home tất cả servos
- [ ] Đã đo 13 vị trí critical
- [ ] Đã cập nhật `cartridge_config.yaml`
- [ ] Đã build package ROS 2
- [ ] Đã test từng servo riêng lẻ
- [ ] Đã kiểm tra collision thresholds
- [ ] Đã verify sensor hoạt động
- [ ] Đã test xi lanh

---

## 🎓 Học Tập & Tham Khảo

### **State Machine:**
- State 1: Input tray loading (9 states)
- State 2: Retrieve done tray (12 states)
- Collision avoidance rules

### **Servo Programming:**
- Modbus TCP protocol
- MotionHandler API
- Position vs Velocity control

### **ROS 2:**
- Node architecture
- Topic pub/sub
- YAML configuration

---

## ✅ Next Steps

1. **✅ Hoàn thành**: Tài liệu đầy đủ
2. **⚠️ Cần làm**: Đo 13 vị trí critical
3. **📋 Sau đó**: Test từng state
4. **🚀 Cuối cùng**: Full system integration

---

## 📚 Document Tree

```
/home/pi/ros2_ws/src/system_feed_cartridge/
├── README.md                    ← Index này
├── SYSTEM_DESCRIPTION.md        ← Mô tả hệ thống (15KB)
├── PHYSICAL_LAYOUT.md           ← Sơ đồ vật lý (18KB)
├── CONFIG_GUIDE.md              ← Hướng dẫn config (5.7KB)
├── config/
│   └── cartridge_config.yaml    ← Config file
├── scripts/
│   └── cartridge_system_py_node.py
├── CMakeLists.txt
└── package.xml

/home/pi/
├── YAML_CONFIG_SUMMARY.md       ← YAML overview (6.1KB)
├── FESTO_API_FIX_SUMMARY.md     ← API reference (4.4KB)
├── read_servo_positions.py      ← Position tool
└── test_festo_servos.py         ← Test script
```

---

**🎉 Hệ thống đã sẵn sàng! Bắt đầu cấu hình và test thôi!**

📄 **Bắt đầu từ:** `SYSTEM_DESCRIPTION.md`  
🔧 **Sau đó:** `CONFIG_GUIDE.md`  
🚀 **Cuối cùng:** Build & Run!
