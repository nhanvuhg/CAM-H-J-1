# Robot Manual Control — Dobot Nova5

> Documentation cho GUI Manual Control trên hệ thống Dobot Nova5 + ROS 2 Jazzy  
> Cập nhật: 2026-03-20

---

## 📋 Mục lục

- [1. Kiến trúc tổng quan](#1-kiến-trúc-tổng-quan)
- [2. JOG Control](#2-jog-control)
- [3. MoveR — Relative Movement](#3-mover--relative-movement)
- [4. GUI Manual Features](#4-gui-manual-features)
- [5. Dobot API Reference](#5-dobot-api-reference)
- [6. Issues & Solutions](#6-issues--solutions)
- [7. Cách chạy hệ thống](#7-cách-chạy-hệ-thống)

---

## 1. Kiến trúc tổng quan

```
┌─────────────────────────────────────────────────────────┐
│                    QML GUI (CartridgePage.qml)           │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────┐ │
│  │ JOG Pad  │  │ Cart     │  │ Joint    │  │ Save    │ │
│  │ X±Y±Z±   │  │ SEND     │  │ SEND     │  │ to YAML │ │
│  │ J1±..J6± │  │ MovL     │  │ MovJ     │  │         │ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬────┘ │
│       │              │              │              │      │
├───────┴──────────────┴──────────────┴──────────────┴──────┤
│              RobotController (C++ / Qt)                   │
│  jogStart()  moveLinear()  moveJoint()  saveJointPose()   │
├───────────────────────────────────────────────────────────┤
│              ROS 2 Service Clients                        │
│  MoveJog  ServoP  MovL  JointMovJ  GetPose  GetAngle     │
├───────────────────────────────────────────────────────────┤
│         dobot_bringup_v3 (Python, port 29999)             │
│              Dashboard API → Robot Controller             │
└───────────────────────────────────────────────────────────┘
```

### Kết nối

| Component        | Port  | Protocol |
|------------------|-------|----------|
| Dashboard + Motion | 29999 | TCP (Dobot API text commands) |
| Robot IP         | 172.16.11.34 | — |

> **Lưu ý**: Firmware Nova5 mới gộp tất cả lệnh (dashboard + motion) qua port 29999. Port 30003/30004 không dùng.

---

## 2. JOG Control

### ⚠️ QUAN TRỌNG: Hybrid Architecture

Firmware Dobot Nova5 trên port 29999 **KHÔNG hỗ trợ Cartesian MoveJog**:

| Axis Type | Method | Lý do |
|-----------|--------|-------|
| **Joint** (J1±...J6±) | `MoveJog("J1+")` | Native firmware support |
| **Cartesian** (X±,Y±,Z±,Rx±,Ry±,Rz±) | `ServoP` streaming | MoveJog trả `res=0` nhưng robot KHÔNG di chuyển |

### Joint JOG — MoveJog

```
Giữ nút → jogStart("J1+") → MoveJog("J1+") → robot quay liên tục
Nhả nút → jogStop()       → MoveJog("")     → robot dừng
```

- Minimum hold time: 200ms (delay trước khi gửi stop)
- Axis IDs: `J1+`, `J1-`, `J2+`, `J2-`, ..., `J6+`, `J6-`
- **Chữ J phải UPPERCASE** (`J1+` ✅, `j1+` → tự convert)

### Cartesian JOG — ServoP Streaming

```
Giữ nút → jogStart("X+")
  ├─ Đọc cartesian_pose_ hiện tại làm base
  ├─ Khởi tạo QTimer 33ms (30Hz)
  └─ Mỗi tick: target[axis] += 0.5mm → ServoP(x,y,z,rx,ry,rz)

Nhả nút → jogStop() → timer.stop() → robot dừng ngay
```

- **Tốc độ**: 0.5mm/tick × 30Hz = ~15mm/s (XYZ), 0.2°/tick (rotation)
- Ngừng gửi ServoP = robot dừng ngay lập tức
- Cần `cartesian_pose_` chính xác (poll mỗi 500ms)

### Test JOG từ CLI

```bash
# Joint JOG
ros2 service call /nova5/dobot_bringup/MoveJog dobot_msgs_v3/srv/MoveJog \
  "{axis_id: 'J1+', param_value: []}"
sleep 2
ros2 service call /nova5/dobot_bringup/MoveJog dobot_msgs_v3/srv/MoveJog \
  "{axis_id: '', param_value: []}"

# Cartesian — ServoP (single step +5mm X)
# 1. Get current pose
ros2 service call /nova5/dobot_bringup/GetPose dobot_msgs_v3/srv/GetPose \
  "{user: 0, tool: 0}"
# 2. Send new pose (X += 5)
ros2 service call /nova5/dobot_bringup/ServoP dobot_msgs_v3/srv/ServoP \
  "{x: 348.0, y: 41.69, z: 143.3, rx: -135.08, ry: 0.886, rz: 97.05}"
```

---

## 3. MoveR — Relative Movement

`moveR(dx, dy, dz)` trong `motion_executor.cpp` — di chuyển robot **tương đối** so với vị trí hiện tại.

### Logic

```
1. prepareLinearMotion()      — ClearError + EnableRobot
2. getCurrentPose(tool=0)     — GetPose với tool=0 (base frame)
3. target = current + (dx, dy, dz), giữ nguyên rx, ry, rz
4. MovL(target)               — Di chuyển tuyến tính
5. sync()                     — Poll RobotMode cho đến IDLE (mode≠7)
```

### ⚠️ Lưu ý quan trọng

1. **Tool frame = 0**: `getCurrentPose()` phải dùng `tool=0` vì `MovL` hoạt động trong base frame
2. **sync()**: Phải poll `RobotMode` đợi robot về IDLE — lệnh `Sync()` firmware không hoạt động
3. **Rz singularity**: Khi robot ở một số pose, Rz có thể flip ±180°. Giữ nguyên Rz từ `getCurrentPose()`

### Code (motion_executor.cpp)

```cpp
bool moveR(double dx, double dy, double dz) {
    if (!prepareLinearMotion()) return false;

    auto current_pose = getCurrentPose();  // tool=0
    if (current_pose.size() < 6) return false;

    auto req = std::make_shared<MovL::Request>();
    req->x  = current_pose[0] + dx;
    req->y  = current_pose[1] + dy;
    req->z  = current_pose[2] + dz;
    req->rx = current_pose[3];  // giữ nguyên
    req->ry = current_pose[4];
    req->rz = current_pose[5];

    auto res = callService<MovL>(movl_client_, req, "MovL");
    if (!res || res->res != 0) return false;

    return sync();  // poll RobotMode until IDLE
}
```

### Test MoveR

```bash
# Di chuyển X+10mm (không đổi Y, Z)
ros2 service call /nova5/dobot_bringup/GetPose dobot_msgs_v3/srv/GetPose \
  "{user: 0, tool: 0}"
# Lấy X hiện tại, cộng 10, giữ nguyên Y/Z/Rx/Ry/Rz
ros2 service call /nova5/dobot_bringup/MovL dobot_msgs_v3/srv/MovL \
  "{x: <X+10>, y: <Y>, z: <Z>, rx: <Rx>, ry: <Ry>, rz: <Rz>, param_value: []}"
```

---

## 4. GUI Manual Features

### Page 3 → Robot Control

#### Cartesian Column (trái)
| Feature | Mô tả |
|---------|--------|
| X, Y, Z, Rx, Ry, Rz inputs | Nhập tọa độ đích |
| **GET POSE** | Đọc vị trí hiện tại → điền vào inputs |
| **SEND MovL** | Di chuyển đến tọa độ đã nhập |

#### Joint Column (phải)
| Feature | Mô tả |
|---------|--------|
| J1–J6 inputs | Nhập góc khớp |
| **GET ANGLES** | Đọc góc hiện tại → điền vào inputs |
| **SEND MovJ** | Di chuyển đến góc đã nhập |
| **SAVE to YAML** | Lưu vị trí + tên vào `joint_pose_params.yaml` |

#### Speed Control
- Slider 1–100%
- **Tự động lưu** vào `~/.config/RobotControl/ManualMode.conf`
- Khởi động lại GUI → load speed đã lưu (không reset 100%)

#### JOG Pad
- 12 nút: X±, Y±, Z±, J1±, J2±, J3±, ...
- **Giữ = di chuyển, nhả = dừng**
- Joint dùng MoveJog, Cartesian dùng ServoP streaming

---

## 5. Dobot API Reference

### Lệnh hoạt động trên port 29999

| Service | Input | Output | Ghi chú |
|---------|-------|--------|---------|
| `EnableRobot` | load | res | Bật robot → mode 5 (IDLE) |
| `ClearError` | — | res | Xóa lỗi |
| `RobotMode` | — | mode, res | 5=IDLE, 7=RUNNING |
| `GetPose` | user, tool | res, pose | Format: `"0,{x,y,z,rx,ry,rz},GetPose();"` |
| `GetAngle` | — | res, angle | Format: `"0,{j1,...,j6},GetAngle();"` |
| `MovL` | x,y,z,rx,ry,rz | res | Di chuyển tuyến tính |
| `JointMovJ` | j1...j6 | res | Di chuyển joint |
| `MoveJog` | axis_id | res | **Chỉ Joint** (J1+, J2-) |
| `ServoP` | x,y,z,rx,ry,rz | res | Real-time position streaming |
| `SpeedFactor` | ratio | res | Set tốc độ 1-100% |
| `DO` | index, status | res | Digital Output |

### Parsing Response

```
Response format: "ErrorCode,{Data},CommandName();"
Ví dụ: "0,{338.09,41.69,143.30,-135.08,0.89,97.05},GetPose();"

✅ Đúng: Chỉ lấy phần giữa { và }
❌ Sai:  Xóa hết {} rồi parse → ErrorCode "0" bị nhầm là data
```

---

## 6. Issues & Solutions

### Issue 1: MoveJog Cartesian không di chuyển
- **Triệu chứng**: `MoveJog("x+")` trả `res=0` nhưng robot không di chuyển
- **Nguyên nhân**: Firmware Nova5 trên port 29999 không hỗ trợ Cartesian MoveJog
- **Giải pháp**: Dùng ServoP streaming (QTimer 30Hz) cho Cartesian

### Issue 2: MoveJog uppercase `X+` → res=-6
- **Triệu chứng**: `MoveJog("X+")` → `res=-6`, `MoveJog("x+")` → `res=0`
- **Nguyên nhân**: Firmware yêu cầu lowercase cho Cartesian axis IDs
- **Giải pháp**: Không dùng MoveJog cho Cartesian (dùng ServoP), nhưng nếu cần thì lowercase

### Issue 3: GetPose/GetAngle hiển thị sai giá trị
- **Triệu chứng**: X luôn = 0, tọa độ lệch 1 vị trí
- **Nguyên nhân**: Response `"0,{x,y,...}"` — code xóa `{}` → error code `0` bị parse thành X
- **Giải pháp**: Extract chỉ phần giữa `{` và `}` bằng `find('{')` + `substr()`

### Issue 4: GUI crash — `placeholderTextColor`
- **Triệu chứng**: `Cannot assign to non-existent property "placeholderTextColor"`
- **Nguyên nhân**: Qt version trên RPi không có property này
- **Giải pháp**: Dùng overlay Text element + check `text.length === 0`
- **Lưu ý**: Cần xóa QML cache: `rm -rf ~/.cache/unified_control_gui/qmlcache/`

### Issue 5: Speed reset 100% mỗi lần restart
- **Triệu chứng**: Chỉnh speed 19% → restart GUI → lại 100%
- **Nguyên nhân**: `speedVal` hardcode `100` trong QML, không persist
- **Giải pháp**: `QSettings("RobotControl", "ManualMode")` lưu/load speed

### Issue 6: MoveR di chuyển sai vị trí
- **Triệu chứng**: `moveR(0,0,-30)` không di chuyển thẳng Z mà loạn xạ
- **Nguyên nhân**: `getCurrentPose()` dùng `tool=1` thay vì `tool=0`
- **Giải pháp**: Đổi thành `tool=0` để match với `MovL` base frame

### Issue 7: sync() không chờ robot hoàn thành
- **Triệu chứng**: Lệnh tiếp theo gửi trước khi robot đến vị trí
- **Nguyên nhân**: `Sync()` firmware không hoạt động trên port 29999
- **Giải pháp**: Poll `RobotMode` mỗi 200ms, đợi mode ≠ 7 (RUNNING)

---

## 7. Cách chạy hệ thống

### Full system (tất cả components)
```bash
bash ~/ros2_ws/src/unified_control_gui/scripts/start_all.sh
```

### Chỉ Robot + GUI (không cần feeder/camera)
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash

# 1. Dobot driver
ros2 launch dobot_bringup_v3 nova5.launch.py &

# 2. GUI
DISPLAY=:0 XAUTHORITY=/home/pi/.Xauthority \
  ~/ros2_ws/install/unified_control_gui/lib/unified_control_gui/unified_control_gui &
```

### Build sau khi sửa code
```bash
cd ~/ros2_ws
MAKEFLAGS="-j 1" colcon build --packages-select unified_control_gui
# Sau build, restart GUI
pkill -9 -f unified_control_gui
rm -rf ~/.cache/unified_control_gui/qmlcache  # nếu sửa QML
# Chạy lại GUI command ở trên
```

### Kiểm tra trạng thái robot
```bash
# Robot mode (5=IDLE, 7=RUNNING)
ros2 service call /nova5/dobot_bringup/RobotMode dobot_msgs_v3/srv/RobotMode

# Current pose
ros2 service call /nova5/dobot_bringup/GetPose dobot_msgs_v3/srv/GetPose \
  "{user: 0, tool: 0}"

# Current angles
ros2 service call /nova5/dobot_bringup/GetAngle dobot_msgs_v3/srv/GetAngle
```
