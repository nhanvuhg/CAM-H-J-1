# Pending System Synchronization & Future Improvements

Tài liệu này ghi nhận lại các phần gỡ lỗi tạm thời (Technical Debt) và các tính năng cố tình tách rời để phục vụ mục đích kiểm thử (Testing) độc lập, nhằm nhắc nhở việc đồng bộ hoàn thiện trong tương lai.

## 1. Trạng Thái Hiện Tại Của Nút Bấm Chế Độ (Mode Buttons)
**Vấn Đề:**
Hiện tại, các nút chuyển chế độ (MANUAL / AUTO / AI) trên giao diện điều khiển (GUI) dành cho Node Cartridge Feeder (Python) và Node Robot Nova5 (C++) đang **KHÔNG ĐƯỢC ĐỒNG BỘ** với nhau.
- Nếu bạn chuyển Robot sang chế độ MANUAL, hệ thống Cartridge không tự động chuyển theo mà vẫn giữ nguyên state cũ. Và ngược lại.

**Lý Do:**
Sếp (User) yêu cầu **CỐ TÌNH CHIA TÁCH** độc lập như hiện tại để phục vụ việc test cơ khí phần cứng hoặc test luồng phần mềm. Việc chia tách giúp ép chạy Cartridge liên tục mà không bắt Robot phải cử động theo, hoặc test tay Robot ở MANUAL nhưng vẫn để băng chuyền chạy AUTO.

## 2. Checklist Đồng Bộ Hoàn Thiện (Khi Chạy Thực Tế)
Khi dự án bước vào giai đoạn hoàn thiện cuối cùng (Production Ready) và không cần test rời rạc nữa, **bắt buộc** phải thực hiện đồng bộ 3 hạng mục sau:

### Mục 1: Đồng Nhất Các ROS Topics (Global Mode Topic)
Thay vì dùng 2 Topic điều khiển trạng thái riêng rẽ:
- `/providesystem/set_operation_mode` (Dành cho Cartridge Feeder)
- `/robot/set_mode` (Dành cho Robot C++)

*Hướng giải quyết:* Cần tạo một Topic chung (Ví dụ: `/system/global_mode`) và quy định cả 2 Node phải `Subscribe` lắng nghe chung Topic này. Khi nhận lệnh, cả 2 sẽ đồng loạt chuyển Mode đánh rầm một phát.

### Mục 2: Cập Nhật `cartridge_providesystem_py_node.py`
Code hiện tại nhận Mode và xử lý ở hàm `_cb_mode`:
```python
def _cb_mode(self, msg: String):
    # Đổi sang Auto / Manual...
```
*Hướng chuẩn hoá:* Node sẽ phải subscribe vào Topic Global. Nếu Global chuyển sang MANUAL, Python Node bắt buộc chốt `self.state = SystemState.MANUAL_MODE` và khóa mọi chuyển động AUTO. Hơn nữa, cần chia tay rạch ròi khái niệm `self._auto_mode` bên trong logic State Machine của Feeder.

### Mục 3: Cập Nhật `robot_logic_node.cpp`
Code hiện tại điều phối ở hàm `setModeCallback`:
```cpp
void RobotLogicNode::setModeCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    // case 1: AUTO, case 3: MANUAL
}
```
*Hướng chuẩn hoá:* Lắng nghe Topic Global. Đảm bảo trạng thái `manual_mode_` và `use_ai_for_control_` gập khuôn 1-1 với trạng thái của hệ thống Feeder. 

### Mục 4: Gom Nút Bấm Trên Giao Diện QML (Unified Control GUI)
- Xóa bỏ kiểu chia tab "Robot Mode" và "Cartridge Mode".
- Thiết lập một góc bảng điều khiển duy nhất (Global System Action) dành cho Hệ thống để điều phối cả 2 hệ Node trong một cú bấm. (Giống như cách chúng ta vừa hợp nhất Nút `STOP` khẩn cấp trên mọi màn hình).

---
*P/S: AI Assistant được dặn dò phải đọc file này bất cứ khi nào User đề cập tới việc "Hợp nhất/Đồng bộ/Hoàn thiện nút AUTO/MANUAL", từ đó triển khai đúng 4 Mục Checklist trên mà không cần User phải giải thích lại kiến trúc.*
