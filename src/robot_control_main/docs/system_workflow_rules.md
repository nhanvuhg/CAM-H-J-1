# Robot Core Logic & Pick-and-Place Workflow (Rule Flow)

Tài liệu này định nghĩa luồng hoạt động chuẩn (Rule Flow) và logic tối ưu hoá của Robot Control Main. 
**MỌI CHỈNH SỬA VỀ SAU ĐỀU PHẢI BÁM THEO VÀ ƯU TIÊN FLOW ĐÃ ĐƯỢC CHUẨN HOÁ TẠI ĐÂY.**

## 1. Mục Đích Hệ Thống
Robot có tác dụng đem cartridge (vỏ/kim rỗng) trên khay Input vào máy bơm (Chamber Fill). Sau khi bơm xong, đem cartridge ra Cân (Scale) để kiểm tra khối lượng, và từ đó quyết định đặt lên khay thành phẩm (Output Tray) hoặc khay lỗi (Fail Tray) tuỳ theo tín hiệu cân.

## 2. Ý Nghĩa Của Vùng Đệm (Buffer)
Thời gian để máy bơm đầy 1 cartridge rơi vào khoảng ~90s. Để tránh việc Robot phải chạy đi chạy lại quãng đường dài từ Khay Input tới Chamber làm mất thời gian gối đầu, hệ thống sử dụng 1 Vùng đệm (Buffer). 
Tác dụng của Buffer là: Robot gắp thả sẵn 1 cartridge chờ ở Buffer. Khi Chamber vừa bơm xong và nhả hàng ra, Robot chỉ việc lấy hàng từ Buffer nhét tắp lự vào Chamber. Nhờ vậy, máy bơm không có độ trễ chết (dead-time), còn Robot sẽ tận dụng thời gian máy đang bơm để quay về Khay Input lấy hàng bù vào Buffer.

## 3. Quy Trình Vận Hành Chuẩn (Cycle)

### Bước 1: Khởi tạo (Init Cycle - Chu trình đầu)
- **`INIT_LOAD_CHAMBER_DIRECT`**: Do hệ thống vừa khởi động, Chamber đang rỗng. Robot bắt buộc phải gắp 1 cartridge trực tiếp từ Input Tray đặt thẳng vào Chamber để máy bơm bắt đầu làm việc.
- **`INIT_REFILL_BUFFER`**: Sau khi đút vào Chamber, Robot quay lại Input Tray gắp thêm 1 cartridge thứ 2, mang nạp lên vùng đợi (Buffer).

### Bước 2: Vòng Lặp Bơm & Tối Ưu Hoá (Main Continuous Loop)
Đây là quy trình lặp đi lặp lại liên tục:
1. **Lấy Hàng Đã Bơm (`TAKE_CHAMBER_TO_SCALE`)**: Đợi Chamber bơm xong, Robot lấy cartridge (đã có mực) từ Chamber ra và đặt lên Cân.
2. **Tiếp Đạn Nhanh (`LOAD_CHAMBER_FROM_BUFFER`)**: Trong lúc chờ Cân xử lý, gắp ngay cartridge rỗng đang chờ sẵn ở Buffer đút vào Chamber để máy bơm lặp lại chu kỳ 90s.
3. **Bù Đạn Cho Buffer rảnh rỗi (`REFILL_BUFFER`)**: Tranh thủ Idle time trong lúc Chamber đang bơm và Cân đang đo, Robot chạy về Khay Input gắp 1 cartridge rỗng mang bù vào Buffer.
4. **Xử Lý Hàng Trên Cân (`PLACE_TO_OUTPUT` / `PLACE_TO_FAIL`)**: Quay lại Scale, đọc kết quả cân và gắp cartridge từ Cân phân loại ra Output Tray hoặc Fail Tray.
*(Sau đó lặp lại Quy trình 2)*

### Bước 3: Xử Lý Dứt Điểm (Clear Pipeline) & Reset Khay
Xảy ra ở những hàng cuối cùng của lượng khay Input trên băng chuyền (khi băng chuyền trên hệ thống Cartridge báo `input_trays_empty` vì không còn khay nào đợi, S1/S2/S3 = OFF). Không còn cartridge rỗng ở Input Tray để nạp lại vào Buffer nữa:
1. Đợi Chamber bơm xong cái gần cuối -> Lên gắp từ Chamber đem sang Cân.
2. Lúc này, gắp nốt cái cartridge cuối cùng của hệ thống (đang nằm sẵn ở Buffer) đút nốt vào Chamber Fill.
3. Quay sang Cân để xử lý cái vừa đặt lên (đem ra Output hoặc Fail).
4. Do Buffer lúc này đã rỗng sạch và không có nguồn bù, Robot sẽ **chỉ đứng chờ** cái cartridge cuối cùng đang nằm trong Chamber Fill bơm cho đầy.
5. Sau khi bơm đầy cục cuối -> Lấy từ Chamber đem sang Cân -> Đợi cân xong -> Xử lý đem ra Output/Fail.
6. Sau khi xử lý sạch sẽ toàn bộ cartridge còn tồn đọng trong chu trình (Pipeline Empty), Robot quay về trạng thái đứng đợi Khay mới chạy vào trên băng chuyền Input (Reset lại từ đầu INIT CYCLE).

## 4. Logic Điều Kiện Kích Hoạt Lệnh Gắp (Trigger Node Logic)
Mọi quyết định gắp và xả ống của hệ thống hiện tại đều được quản lý bằng sự kết hợp của các điều kiện (Topic) cốt lõi sau:

- **Logic ra lệnh gắp thông thường (Bản lề)**: Vẫn giữ nguyên như cũ, Robot chỉ tiến hành gắp khi thoả mãn đồng thời 2 yếu tố `feed_chamber` + `new_tray_loaded` (Có lệnh cấp từ bộ điều khiển + Khay đã load vào đúng vị trí).
- **Khay hoạt động liên tục (Continuous Tray)**: `feed_chamber` + `new_tray_loaded` + **"Topic còn khay"** (`input_trays_empty = FALSE`, tức là S1/S2/S3 vẫn ON). Hệ thống gắp bình thường và kết thúc khay sẽ kiên nhẫn đứng đợi khay tiếp theo.
- **Khay cuối cùng chạy Drain (Last Tray Pipeline Drain)**: `feed_chamber` + `new_tray_loaded` + **"Không có topic còn khay"** (`input_trays_empty = TRUE`, tức là S1/S2/S3 đã tắt ngóm). Ngay khi Robot gắp tới Row 5 của khay này, hệ thống sẽ chốt hạ và bắt đầu chuyển sang chu kỳ Drain vét sạch ống như đã định nghĩa ở Bước 3.

---
*Note: Quá trình xử lý dứt điểm các cartridge tồn đọng (Clear Pipeline) bắt buộc phải phụ thuộc vào cảm biến vật lý `input_trays_empty` của hệ thống Cartridge (S1, S2, S3). Tuyệt đối không được tự ý chuyển sang dọn dẹp kết thúc chu trình nếu như phía sau băng chuyền vẫn còn Khay đang đợi hoặc đang chạy vào chậm.*
