# Industrial 24/7 Deployment Architecture
Cập nhật lần cuối: 2026-03-31

Tài liệu này ghi lại kiến trúc vận hành công nghiệp song song đã được thiết lập để chống treo (Anti-deadlock) và chia sẻ kinh nghiệm xử lý các `Lỗi đã qua` (Past Errors) trong quá trình nâng cấp hệ thống Cấp/Nhả khay Festo CMMT-AS Modbus.

## 1. Parallel State Machine (S3 & S4 Xử lý Song Song)
**Lý do tái thiết kế:**
Ban đầu, ROS 2 Python Node sử dụng chung 1 biến trạng thái `state_out` cho vòng lặp phía đầu ra (Output Pipeline). Điều này gây tắc hẹp (bottleneck): Nếu tay gắp khay (State 4) đang đi lấy khay đầy dời đi, cụm đùn khay dự phòng (State 3) phải đợi S4 báo cáo `IDLE` mới được phép đùn khay tiếp theo tới.

**Kiến trúc mới:**
Đã tách cấu trúc `state_out` thành hai cỗ máy trạng thái (State machines) vận hành độc lập trong hàm ngắt `timer_cb` của `cartridge_providesystem_py_node.py`:
- `self.state_s3` kiểm soát mâm đùn Servo 3.
- `self.state_s4` kiểm soát mâm nâng hạ, tay gắp khay Servo 4, Servo 5 và Relay Kẹp khí nén (Cyl 2).

### Logic Đùn Tự Động Đầu Ra (S8 Timer Delay)
Để hệ thống không bị xung đột vật lý giữa Servo 3 đẩy khay và Servo 4 nhấc khay: 
1. Hệ thống giám sát cảm biến `S8` (Khay tại vị trí hứng robot).
2. Ngay khi S8 từ `ON` chuyển sang `OFF` (Do gầm máy gắp S4 khênh lấy khay đi), bộ dò thời gian nội bộ `self._s8_off_time` tự động tính giây.
3. Chờ chính xác an toàn **5.0 giây**, cờ `_can_start_s3()` nhảy `True` tự động phát lệnh nhả `State 3` đùn khay rỗng xuống.

## 2. Loại bỏ Deadlocks & Lỗi Timeout Cũ
Các lỗi cũ đã trải qua (Past Errors) và cách chặn đứng:
- **Treo Giao Diện (GUI Freeze)**: Việc sử dụng `while(rclcpp::ok() && pending)` bên Backend C++ để đợi node Python xử lý các vòng Check Cảm Biến/Timeout (`JOG`, `ABORT`, `TRIGGER`) sẽ gây nghẽn Thread GUI nếu Modbus TCP mất kết nối quá lâu. Giải pháp là ứng dụng cơ chế `Non-blocking Jog` và chuyển Trigger Mode bằng tín hiệu ROS `Bool` bất đồng bộ `/robot/done_tray_input`.
- **Modbus Timeout Limit**: Nếu dây cáp hở, `ModbusTcpClient` kẹt timeout, ROS timer ngắt mạch, dẫn đến vòng lặp Homing bị fail vĩnh viễn ở `_step_timeout_in`. Giải pháp: Máy luôn giữ quyền năng `ABORT_TO_JOG` mọi lúc. Lệnh "START" có quyền khơi lại kết nối `tcp_retry` nội bộ nếu `_can_start_sX()` bị hụt.
- **State Navigation Phức Tạp**: Dọn sạch các nút 'Simulation Done Input / Output'. Các nút `STATE 2` và `STATE 4` trong QML giờ gắn thẳng vào Signal `simulateDoneTrayInput/Output()`. An toàn tuyệt đối 100% không làm bể chu trình Sensor.
- **Treo Scanning Do Đóng Cắt Bất Thường (Noise/Nháy Sáng)**: Trục INY/OUTY khi lao xuống quét (Scan) có nguy cơ gục nếu cảm biến S4/S10 nháy sáng ở điểm quá cao do bụi kẹt hoặc ánh sáng môi trường. Đã quy hoạch lại thành cơ chế **Retract & Retry** (Nhấc trục - Quét lại). Cảm biến S4/S10 bị kích hoạt sớm (trước mốc `arm_mm`, mặc định `50mm`) sẽ bị hủy lệnh, trục chạy lùi về điểm an toàn `10mm` ngay lập tức, và thử lại. Nếu fail 2 lần liên tục, hệ thống chuyển sang `IDLE` (ERROR) để an toàn thay vì cố gắng kéo nhầm.
- **Tiền Kiểm Trạng Thái (Pre-check Logic) Ở Chế Độ MANUAL**: Sau khi bỏ Sensor Simulation (branch `feature/manual-real-sensors`), MANUAL đọc sensor THẬT từ IO module như AUTO/AI. State 1 và State 3 KHÔNG tự trigger trong MANUAL (`_state1_enabled = False`, `_can_start_s3` lock auto/ai). Chỉ State 2/4 trigger được qua nút GUI (pub `/robot/done_tray_input` / `/robot/done_tray_output` + check sensor S7/S9/S18 thật). Sau khi State 2 hoặc State 4 hoàn tất → dừng chain, không tự kích lại State 1 như AUTO.

## 3. Launch System & Systemd 24/7 Profile
Để hệ thống hoạt động liên hoàn (Crash-recovery), file launch chính là: `launch/cartridge_production.launch.py`.
- **Cơ chế Respawn**: Node GUI và Node Điều khiển Python đều chạy cờ `respawn=True`, nếu tràn RAM hoặc kẹt Modbus Fatal, ROS tự hồi sinh lại PID trong vòng vài giây.
- **Systemd**: `scripts/cartridge_robot.service` (nằm ngoài /etc/) làm nhiệm vụ nạp `DISPLAY=:0`, `.bashrc` và khởi động Launch ROS ngay sau khi RPi5 boot xong màn hình Wayland/X11.

## 4. Test Scripts
Test scripts cũ (`test_s1_full.py`, `test_s2_full.py`, `test_s3_s4_full.py`) dựa trên cơ chế Sensor Simulation đã bị bỏ ở branch `feature/manual-real-sensors` — không còn dùng được. Test mới `tests/test_state_machine.py` inject sensor state thẳng vào `_io_sensor_cache` qua helper `_set_sensors()`.
