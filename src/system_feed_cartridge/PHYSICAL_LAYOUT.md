# 🏗️ SƠ ĐỒ VẬT LÝ HỆ THỐNG

## Góc Nhìn Tổng Thể (Top View)

```
                          ROBOT ARM
                              ↓
    ╔═════════════════════════════════════════════════════════╗
    ║                   KHOANG LÀM VIỆC                        ║
    ║                                                          ║
    ║    ┌──────────────┐              ┌──────────────┐       ║
    ║    │              │              │              │       ║
    ║    │   INPUT      │              │   OUTPUT     │       ║
    ║    │   AREA       │              │   AREA       │       ║
    ║    │              │              │              │       ║
    ║    │  ┌────────┐  │              │  ┌────────┐  │       ║
    ║    │  │ SERVO  │  │              │  │ SERVO  │  │       ║
    ║    │  │  InY   │  │              │  │  OutY  │  │       ║
    ║    │  │   ↕    │  │              │  │   ↕    │  │       ║
    ║    │  └────────┘  │              │  └────────┘  │       ║
    ║    │       ↑      │              │      ↑       │       ║
    ║    │       │      │              │      │       │       ║
    ║    └───────┼──────┘              └──────┼───────┘       ║
    ║            │                            │               ║
    ║    ┌───────┴──────┐              ┌──────┴───────┐       ║
    ║    │   SERVO InX  │              │  SERVO OutX  │       ║
    ║    │      ←→      │              │      ←→      │       ║
    ║    └──────────────┘              └──────────────┘       ║
    ║                                                          ║
    ╚═════════════════════════════════════════════════════════╝
                 ↑                              ↑
              CONVEYOR                    SERVO 3
            (Khay mới)                 (Put Tray - Đẩy khay)
```

---

## Chi Tiết Input Area

```
                    ┌─────────────────────────────┐
                    │    CONVEYOR BELT (IN)       │
                    │         ↓ ↓ ↓                │
                    └─────────────────────────────┘
                              │
                              ▼
    ╔═════════════════════════════════════════════════════════╗
    ║               INPUT TRAY HANDLER                         ║
    ║                                                          ║
    ║     SERVO 2 - InY (Vertical)                            ║
    ║          │                                               ║
    ║          ├── Home (0mm)                                  ║
    ║          │                                               ║
    ║          ├── Target2 (200mm) ← Vị trí an toàn           ║
    ║          │                                               ║
    ║          ├── Row 1 (250mm)  ◄─┐                          ║
    ║          ├── Row 2 (300mm)    │                          ║
    ║          ├── Row 3 (350mm)    │ 8 Rows                   ║
    ║          ├── Row 4 (400mm)    │ trên khay                ║
    ║          ├── Row 5 (450mm)    │ đầu vào                  ║
    ║          ├── Row 6 (500mm)    │                          ║
    ║          ├── Row 7 (550mm)    │                          ║
    ║          └── Row 8 (600mm)  ◄─┘                          ║
    ║                 ▲                                        ║
    ║                 │                                        ║
    ║          ┌──────┴──────┐                                 ║
    ║          │   SENSOR 5  │ ← Phát hiện khay               ║
    ║          └─────────────┘                                 ║
    ║                                                          ║
    ║     SERVO 1 - InX (Horizontal)                          ║
    ║          │                                               ║
    ║    Home ─┴────────────────────────► Target2             ║
    ║    (0mm)                          (500mm)                ║
    ║                                   ▲                      ║
    ║                                   │                      ║
    ║                            ┌──────┴──────┐               ║
    ║                            │ CYLINDER 1  │               ║
    ║                            │  Extend ◄─► │               ║
    ║                            │   Retract   │               ║
    ║                            └─────────────┘               ║
    ║                                                          ║
    ╚═════════════════════════════════════════════════════════╝
```

### **Workflow Input Area:**
```
1. Khay mới đến từ conveyor
2. InX di chuyển ra → Target2 (500mm)
3. InY tìm kiếm row → Sensor 5 phát hiện khay
4. InY di chuyển đến Row position
5. Cylinder 1 Extend → Lấy cartridges
6. InY về Home → về Target2
7. Cylinder 1 Retract
8. InX về Home
9. Lặp lại cho row tiếp theo
```

---

## Chi Tiết Output Area

```
    ╔═════════════════════════════════════════════════════════╗
    ║              OUTPUT TRAY HANDLER                         ║
    ║                                                          ║
    ║     SERVO 5 - OutY (Vertical)                           ║
    ║          │                                               ║
    ║          ├── Home (0mm)                                  ║
    ║          │                                               ║
    ║          ├── Target1 (50mm) ← Vị trí an toàn            ║
    ║          │                                               ║
    ║          └── Target2 (300mm) ← Stack đầu ra             ║
    ║                 ▲                                        ║
    ║                 │                                        ║
    ║                 │                                        ║
    ║     SERVO 4 - OutX (Horizontal)                         ║
    ║          │                                               ║
    ║    Home ─┴──────────────► Target1 ─────► Target2        ║
    ║    (0mm)                (100mm)       (400mm)            ║
    ║                                          ▲               ║
    ║                                          │               ║
    ║                                   ┌──────┴──────┐        ║
    ║                                   │ CYLINDER 2  │        ║
    ║                                   │  Extend ◄─► │        ║
    ║                                   │   Retract   │        ║
    ║                                   └─────────────┘        ║
    ║                                                          ║
    ╚═════════════════════════════════════════════════════════╝
                              ▲
                              │
                    ┌─────────┴─────────┐
                    │    SERVO 3        │
                    │   Put Tray        │
                    │   Push/Retract    │
                    └───────────────────┘
                              ▲
                              │
                         ROBOT ARM
                    (Lấy khay đã xử lý)
```

### **Workflow Output Area (State 2):**
```
1. Robot xử lý xong → Signal /cartridge/load_tray_input
2. OutX di chuyển ra → Target2 (400mm)
3. OutY di chuyển ra → Target2 (300mm)
4. Cylinder 2 Extend → Lấy khay đã xử lý
5. OutY về Target1 → về Home
6. OutX về Target1 → về Home
7. OutY tìm kiếm row position (giống InY)
8. OutY di chuyển đến Row
9. Cylinder 2 Retract → Đặt cartridges
10. OutY về Home
11. OutX về Home
```

---

## Servo 3 - Put Tray System

```
                    ROBOT ARM WORKSPACE
                           ▲
                           │
                           │
    ╔══════════════════════╧═══════════════════════╗
    ║         SERVO 3 - PUT TRAY MECHANISM         ║
    ║                                              ║
    ║    Home                    Push Position    ║
    ║    (0mm) ───────────►      (300mm)          ║
    ║                                ▲             ║
    ║      Khay ở đây───────────────┘             ║
    ║       để robot                               ║
    ║       có thể lấy                             ║
    ║                                              ║
    ╚══════════════════════════════════════════════╝
```

### **Function:**
```
- Đẩy khay ra vị trí để robot có thể pick
- Sau khi robot lấy xong, rút về Home
- Chờ khay tiếp theo
```

---

## Collision Avoidance Zones

```
    SAFE ZONES (Vùng An Toàn)

    INPUT AREA:
    ┌────────────────────────────────────┐
    │  InY Safe Zone                     │
    │  ├─ 0mm - 50mm                     │
    │  └─ InX có thể di chuyển           │
    │                                    │
    │  InY Danger Zone                   │
    │  ├─ > 50mm                         │
    │  └─ InX KHÔNG được di chuyển       │
    └────────────────────────────────────┘

    OUTPUT AREA:
    ┌────────────────────────────────────┐
    │  OutX Safe Zone                    │
    │  ├─ 0mm - 100mm                    │
    │  └─ OutY có thể di chuyển          │
    │                                    │
    │  OutX Danger Zone                  │
    │  ├─ > 100mm                        │
    │  └─ OutY KHÔNG được di chuyển      │
    └────────────────────────────────────┘
```

---

## Sensor Layout (IO Module)

```
    ╔═════════════════════════════════════════════════════════╗
    ║          FESTO CPX-AP IO MODULE                         ║
    ║              172.16.11.41                             ║
    ╠═════════════════════════════════════════════════════════╣
    ║                                                         ║
    ║  INPUT SENSORS (15 channels):                          ║
    ║  ┌──────────────────────────────────────────────────┐  ║
    ║  │ Sensor 1:  ___________________________           │  ║
    ║  │ Sensor 2:  ___________________________           │  ║
    ║  │ Sensor 3:  ___________________________           │  ║
    ║  │ Sensor 4:  ___________________________           │  ║
    ║  │ Sensor 5:  Phát hiện khay trên InY ◄─ CRITICAL   │  ║
    ║  │ Sensor 6:  ___________________________           │  ║
    ║  │ ...                                              │  ║
    ║  │ Sensor 15: ___________________________           │  ║
    ║  └──────────────────────────────────────────────────┘  ║
    ║                                                         ║
    ║  OUTPUT CHANNELS (Cylinders):                          ║
    ║  ┌──────────────────────────────────────────────────┐  ║
    ║  │ Channel 4:  Cylinder 1 Retract                   │  ║
    ║  │ Channel 5:  Cylinder 1 Extend                    │  ║
    ║  │ Channel 6:  Cylinder 2 Retract                   │  ║
    ║  │ Channel 7:  Cylinder 2 Extend                    │  ║
    ║  └──────────────────────────────────────────────────┘  ║
    ║                                                         ║
    ╚═════════════════════════════════════════════════════════╝
```

---

## Network Topology

```
    ┌────────────────────────────────────────────────────┐
    │         NETWORK: 172.16.11.x                      │
    │              Modbus TCP/IP                         │
    └──────────────────┬─────────────────────────────────┘
                       │
         ┌─────────────┴──────────────┐
         │                            │
    ┌────▼─────┐                 ┌────▼─────┐
    │  SERVOS  │                 │ IO MODULE│
    └──────────┘                 └──────────┘
    │                            │
    ├─ 247: InX                  └─ 254: CPX-AP
    ├─ 248: InY
    ├─ 103: Put Tray
    ├─ 104: OutX
    └─ 105: OutY

              ▲
              │
         ┌────┴─────┐
         │ ROS 2 PC │
         │ (RPi/PC) │
         └──────────┘
```

---

## Dimension Guidelines

### **Tray Dimensions:**
```
┌──────────────────────────────────┐
│        INPUT TRAY                │
│                                  │
│  ├── Row 1  ─┐                   │
│  ├── Row 2   │                   │
│  ├── Row 3   │                   │
│  ├── Row 4   ├─ 8 Rows           │
│  ├── Row 5   │  Cartridges       │
│  ├── Row 6   │                   │
│  ├── Row 7   │                   │
│  └── Row 8  ─┘                   │
│                                  │
│  Spacing: ~50mm giữa các row     │
│  (CẦN ĐO CHÍNH XÁC)              │
└──────────────────────────────────┘
```

### **Working Area:**
```
┌────────────────────────────────────┐
│  InX Travel: 0 → 500mm             │
│  InY Travel: 0 → 600mm             │
│                                    │
│  OutX Travel: 0 → 400mm            │
│  OutY Travel: 0 → 300mm            │
│                                    │
│  Servo 3: 0 → 300mm                │
└────────────────────────────────────┘
```

---

## Legend

```
Symbol    Meaning
──────    ─────────────────────────────
  ↕       Vertical movement (Y axis)
  ←→      Horizontal movement (X axis)
  ▲       Direction of motion
  ◄─►     Extend/Retract
  │       Connection/Path
  ─       Fixed structure
  ═       Heavy duty component
  ┌─┐     Component boundary
```

---

**📐 Để đo chính xác:**
1. Home tất cả servos
2. Di chuyển thủ công đến vị trí
3. Dùng `read_servo_positions.py` để đọc
4. Cập nhật vào `cartridge_config.yaml`

**🔧 Xem file:** `SYSTEM_DESCRIPTION.md` để hiểu workflow chi tiết!
