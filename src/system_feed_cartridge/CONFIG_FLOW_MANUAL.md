# Phân tích Luồng Hoạt Động (Flow) & Biến Cấu Hình (Config Mapping)
Cập nhật: 2026-03-31
File Node chính: `cartridge_providesystem_py_node.py`
File Cấu Hình chính: `cartridge_config.yaml`

---

## 1. Mục tiêu (Goals)
Đảm bảo hệ thống ở chế độ **MANUAL** và **AUTO** hoạt động theo luồng song song chuẩn xác, tối ưu thời gian chu kỳ:
- **STATE 1 (Input Load)**: Nạp khay rỗng vào hệ thống bằng ngàm kéo Cyl1 qua cơ cấu con trượt độc lập.
- **STATE 2 (Input Replace)**: Rút và thả khay khi khay Input đã được gắp hết. Chỉ chạy khi robot emit `/robot/done_tray_input`. (Hoặc nhấn Button trên GUI).
- **STATE 3 (Output Feed)**: Đùn tịnh tiến mâm cấp khay rỗng (S7) vào vị trí hứng thành phẩm (S8). State 3 tự động kích hoạt liên tục **ở chế độ nền đếm ngược 5 giây sau khi S8 trống**, chạy song song với bất kỳ lệnh nào.
- **STATE 4 (Output Replace)**: Ôm khay đã đầy (S8) đưa lên stack (Row 1-8). Trục gắp này hoàn toàn độc lập với Servo nạp khay, cho phép máy làm 2 việc cùng lúc.

## 2. Ánh Xạ Biến Cấu Hình (Config Keys Mapping)
> Không sử dụng cơ chế nội suy (`__getattr__`) hoặc biến tự tạo. Code backend Python BẮT BUỘC map chính xác 1-1 với cấu trúc biến trong file `cartridge_config.yaml`.

| YAML Config Key | Kiểu | Ý nghĩa và Sử dụng trong Code |
|-----------------|------|---------------------------------|
| `inx_target2`   | Float | Đích x (mm) của InX khi kéo khay từ đầu vào (Mặc định: 500.0). |
| `target_scaninp1`| Float | Mục tiêu cho quét cảm biến input (InY). |
| `inx_output_stack`| Float | Vị trí x (mm) của InX khi đến Output Stack để thả khay. |
| `inx_home`      | Float | Vị trí Home an toàn cho trục Input. |
| `iny_target2`   | Float | Vị trí thả khay rỗng cho Input tray. |
| `outx_target2`  | Float | Đích x (mm) của OutX khi gầm khay Output. |
| `outy_pick`     | Float | Đích y (mm) của OutY khi gắp khay (S4). |
| `move_timeout`  | Float | Thời gian chờ tối đa cho 1 di chuyển Non-blocking (Mặc định: ~ 60s) |

## 3. Luồng Chạy Song Song (Parallel Execution Flow)

### 3.1. Phân luồng Dispatch
Trạng thái tĩnh (State Machine) được chia làm 3 biến theo dõi song song:
- `self.state_in`: Xử lý cụm Load/Unload đầu vào (`State 1` & `State 2A`).
- `self.state_s3`: Xử lý việc đùn khay Output rỗng (`State 3` - Servo 3).
- `self.state_s4`: Xử lý việc bưng mâm Output đưa vào ngăn xếp thư viện (`State 4` - Servo 4/5).

Vì `state_s3` và `state_s4` độc lập tịnh tiến trên không gian nhớ, máy gắp khay (`S4`) vừa kẹp xong nhấc lên rời khỏi S8 là `S3` đã vào nhịp đếm ngược 5 giây để đùn khay mới vào chuẩn bị, không có độ trễ logic chờ nhau.

### 3.2. Điều kiện kích hoạt State Nội Bộ
- **Trigger STATE 1**:
  - `(S1 OR S2 OR S3) ON` (Có khay vào)
  - `S11` OFF (Vị trí gắp đang trống)
  - `S15` ON (Ngàm đã nhả)
- **Trigger STATE 3**:
  - `S7` ON (Có stack khay nhựa dự phòng)
  - `S8` OFF liên tục trong **5.0 giây** (Vị trí chờ robot đang trống trải). 
  - *Trigger tự động kể cả khi Robot đang chạy S4.*
- **Trigger STATE 4**:
  - Nhận cờ `/robot/done_tray_output` (do Robot gọi hoặc nhấn button Navigation GUI).
  - `S8` ON (Khay thực tế có nằm đó).

## 4. Tương tác GUI
Bảng mô phỏng Simulation Sensor đã được làm tinh gọn:
- Nút **STATE 2 (Thay Khay In)**: Không chạy code cưỡng ép vòng C++, mà gửi tín hiệu Node ROS an toàn (`_input_tray_done`), từ đó State Machine tự điều hướng vào quy trình `S2A` nếu thỏa mãn `_can_start_s2a()`.
- Nút **STATE 4 (Thay Khay Out)**: Gửi tín hiệu `/robot/done_tray_output`. Nếu máy báo bận hoặc không có S8, Node sẽ từ chối nhận lệnh và popup báo cáo thao tác rỗng.
