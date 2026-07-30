# 🛠 Troubleshooting Log — Dual Camera System

Tài liệu này lưu trữ các sự cố khó (edge-case bugs) đã gặp phải trong quá trình phát triển hệ thống và cách giải quyết tận gốc để phòng tránh lỗi lặp lại.

---

## 1. Màn hình GUI (Qt/QML) bị ám màu Xanh Dương (Blue Tint)
**Ngày ghi nhận:** 2026-04-22

### Biểu hiện:
- Màu sắc hiển thị trên màn hình ứng dụng (`ros2_qml_gui1`) liên tục bị ám xanh dương nặng (kim loại ngả xanh, da người tái nhợt).
- Camera Node publish dữ liệu hình ảnh với định dạng màu `bgr8`.

### Nguyên nhân gốc (Root Cause):
File `cam_node.cpp` sử dụng `QImage::Format_BGR888` để render ảnh BGR trực tiếp lên QML:
```cpp
QImage qimg(resized.data, resized.cols, resized.rows, resized.step, QImage::Format_BGR888);
```
Tuy nhiên, `Format_BGR888` không được hỗ trợ ổn định trong GPU rendering pipeline của Qt/QML (đặc biệt trên các bo mạch nhúng như Raspberry Pi). Khi đẩy vào GPU, QML tự động coi dữ liệu đó là `RGB`, dẫn đến 2 kênh **Red và Blue bị hoán đổi** (R↔B channel swap). Trắng/xám bị đổi thành Xanh dương.

### Cách khắc phục:
Tuyệt đối không dùng `QImage::Format_BGR888`. Luôn dùng `cv::cvtColor` để chuyển trực tiếp ảnh `BGR` thành `RGB` trong C++ trước, rồi mới bọc nó bằng `QImage::Format_RGB888` (đây là format chuẩn quốc tế được hỗ trợ 100% trên mọi nền tảng đồ họa).

**File đã sửa:** `src/ros2_qml_gui1/src/cam_node.cpp`
```cpp
cv::Mat rgb;
cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
QImage qimg(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888);
```

---

## 2. Camera IMX477 bị crash (đơ đứng hình) khi thay đổi độ sáng đột ngột / đưa tay che ống kính
**Ngày ghi nhận:** 2026-04-21

### Biểu hiện:
- Đang streaming bình thường, nếu đèn bật/tắt hoặc có vật thể đen to (như tay người, thiết bị) che ngang ống kính, ROS2 node báo lỗi `Dequeue timer expired`.
- Hệ thống bị đơ hoàn toàn mạch CFE (Camera Frontend), không thể khôi phục nếu không khởi động lại hoặc `modprobe` lại driver.

### Nguyên nhân gốc (Root Cause):
Cơ chế **AEC/AGC (Cân bằng sáng/Gain tự động)** của thuật toán ISP (Image Signal Processor).
Khi ánh sáng sụt giảm đột ngột, ISP cố gắng tự động tăng thời gian phơi sáng (shutter speed) lên mức rất cao (>33ms). Lúc này, frame xuất ra từ sensor bị chậm lại. Do mạch CFE (hardware) mong đợi 1 thời lượng frame (VBLANK intervals) cố định, sự bất đồng bộ giữa hardware và driver V4L2 này trực tiếp làm tràn DMA buffers, gây deadlock toàn phần ở mức phần cứng.

### Cách khắc phục:
Chặn tuyệt đối không cho AE và AG tự động chạy bằng cách cung cấp giá trị fix cứng trực tiếp vào tham số khởi động của `rpicam-vid`:
- `--shutter 30000` (khóa chết phơi sáng ở mức 30ms - đảm bảo tốc độ xuất >30fps).
- `--analoggain 8.0` (khóa cứng analog gain - đảm bảo ảnh vẫn sáng với tốc độ chớp 30ms).
Việc khóa 2 thông số này giúp pipeline phần cứng không bao giờ bị dao động, loại bỏ 100% lỗi crash do che ống kính.

**Lưu ý:** Không thiết lập hardcode tham số `--awbgains` (Cân bằng trắng) vì nó không ảnh hưởng tới timing của mạch CFE mà chỉ làm sai lệch màu sắc môi trường. AWB vẫn được phép chạy Auto.
