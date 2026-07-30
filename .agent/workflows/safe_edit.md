---
description: Quy trình kiểm tra kiến trúc vĩ mô (Macro Architecture) trước khi chỉnh sửa mã nguồn. Áp dụng cho mọi yêu cầu sửa lỗi, nâng cấp hoặc thay đổi thuật toán.
---

Để đảm bảo hệ thống máy chiết rót Cartridge chạy ROS 2 không bao giờ bị phá vỡ cấu trúc và luôn hoạt động ổn định nhất, Agent (Antigravity) BẮT BUỘC phải thực hiện quy trình này trước khi sử dụng các công cụ sửa file (như `replace_file_content`):

1. **Phân tích Kiến trúc Vĩ Mô (Macro Assessment):**
   - Đối chiếu file C++ / Python / QML sắp sửa đổi với "Bức tranh tổng thể" trong Knowledge Items (hoặc `mcp-server-memory`).
   - Xác định file này thuộc thành phần nào: Vision (YOLO/libcamera), Festo (CMMT-AS Modbus), hay Robot Logic (Dobot/State Machine).
   - Trích xuất luồng I/O: File này hiện đang Publish/Subscribe vào những topic nào của hệ thống ROS 2?

2. **Dò tìm Rủi ro phụ thuộc (Dependency Risk Analysis):**
   - Nếu thay đổi kiểu dữ liệu hoặc ngắt vòng lặp ở đây, Node nào ở đầu kia hệ thống sẽ chịu hậu quả?
   - Tính toán xem sự thay đổi có dẫn tới deadlock (treo loop) của Modbus TCP hay vướng safety lock của 13 sensors hay không.

3. **Trình bày Báo Cáo Chớp Nhoáng (Impact Report):**
   - Trước khi sửa code, in ra cửa sổ chat cho USER thấy các Node sẽ bị ảnh hưởng.
   - Ví dụ: *"Nếu sửa đoạn này, Node YOLO sẽ nhận mảng số thực thay vì số nguyên, dẫn đến GUI có thể bị treo."*

4. **Tiến hành Sửa Chữa (Micro Execution):**
   - Đợi USER xác nhận đồng ý với báo cáo trên.
   - Nếu được duyệt, tiến hành chỉnh sửa file nhanh chóng và thông minh bằng công cụ edit ưu tiên cho code.

5. **Lưu Trí nhớ mới:**
   - Hoàn tất mọi thứ, ghi lại các thành phần kiến trúc vừa bị thay đổi vào Knowledge Graph để cập nhật lại "Bức tranh tổng thể".
