# Kế hoạch & Phân tích Trùng lặp Topic ROS 2 (Hệ thống Robot-HP)

Khi triển khai hệ thống thứ 2 (ví dụ `HP_2`) trên cùng một mạng ROS 2 (cùng `ROS_DOMAIN_ID`), nếu các Node vẫn dùng tên topic cố định (hardcoded), chúng sẽ phát chéo cho nhau dẫn đến nhiễu loạn điều khiển (GUI của máy 1 điều khiển nhầm máy 2, camera máy 1 chiếu lên màn hình máy 2, v.v.).

Dưới đây là danh sách phân tích các topic hiện tại và **Kế hoạch đặt tên/Namespace cách ly**.

## 1. Danh sách Topic hiện tại và Mức độ rủi ro trùng lặp

### 📸 A. Hệ thống Camera (CSI Camera)
| Topic Hiện Tại | Mức Độ | Vấn Đề Gặp Phải |
| :--- | :---: | :--- |
| `/cam0HP/image_raw` | 🔴 Cao | Dùng cố định tên `HP`. Máy 2 sẽ ghi đè lên luồng của máy 1. |
| `/cam1HP/image_raw` | 🔴 Cao | Tương tự cam0. |
| `/camera/status` | 🔴 Cao | Rất chung chung. Cả 2 máy tranh nhau báo status. |
| `/camera/active` | 🔴 Cao | GUI không biết hình ảnh đang được chiếu của máy nào. |

### 🧠 B. Hệ thống AI / YOLO (HailoRT)
| Topic Hiện Tại | Mức Độ | Vấn Đề Gặp Phải |
| :--- | :---: | :--- |
| `/yolo/detections` | 🔴 Cao | Tọa độ AI từ máy 1 sẽ vô tình gửi cho Robot máy 2. |
| `/yolo/image_overlay` | 🔴 Cao | GUI máy nào cũng nhận được hình chứa bounding box này. |
| `/pipeline_status` | 🔴 Cao | Chung chung, gây nhiễu log hệ thống. |
| `/camera_active` | 🟡 Vừa | Cờ dùng để logic chuyển Cam nhưng dễ dán nhầm sang băng chuyền khác. |

### 🤖 C. Hệ thống Robot (Dobot Nova5 & Motion Executor)
| Topic Hiện Tại | Mức Độ | Vấn Đề Gặp Phải |
| :--- | :---: | :--- |
| `/nova5/dobot_bringup/*` | 🔴 Cao | Tên cố định theo dòng xe "nova5". Gọi service `MovL` sẽ chạy nhầm tay máy. |
| `/robot/system_status` | 🔴 Cao | Trạng thái bị trộn lẫn. |
| `/robot/motion_busy` | 🔴 Cao | Cả Cartridge máy 1 và máy 2 cùng bị block vì cờ busy của robot kia. |
| `/robot/done_tray_output` | 🔴 Cao | Gửi nhầm tín hiệu lấy khay. |
| Các lệnh Service (`/robot/start_system`, `/robot/set_ai_mode`) | 🔴 Cao | Nhấn Start trên GUI máy 1 sẽ làm Robot máy 2 chạy theo. |

### ⚙️ D. Hệ thống Cartridge & Băng chuyền & Cân
| Topic Hiện Tại | Mức Độ | Vấn Đề Gặp Phải |
| :--- | :---: | :--- |
| `/system_state` | 🔴 C.Cấp | Tên quá chung chung (chỉ ghi `system_state`). Cần được gom cụ thể vào Cartridge. |
| `/providesystem/sensors_state` | 🔴 Cao | Các sensor 1-20 bị đọc chéo giữa máy 1 và máy 2. |
| `/cartridge/busy` | 🔴 Cao | Tín hiệu interlock sẽ khóa chéo 2 hệ thống. |
| `/cartridge_providesystem/new_tray` | 🔴 Cao | Máy 1 có khay mới nhưng báo nhầm cho máy 2 chạy. |
| `/system/start_button` | 🔴 Cao | Tên quá chung chung. Các nút bấm vật lý/GUI sẽ kích hoạt cả xưởng. |
| `/scale/result` | 🔴 Cao | Cân của máy 1 đẩy kết quả Pass/Fail cho máy 2. |
| `/fill_machine/fill_done` | 🔴 Cao | Máy chiết rót xong nhưng máy khác chạy. |

---

## 2. Kế hoạch Cách ly Quy mô lớn (Namespace Policy)

Thay vì đi đổi tên từng topic thành `/hp_1/cam0/image_raw` một cách thủ công trong C++ và Python, thông lệ tốt nhất trong ROS 2 (Best Practices) là ứng dụng **ROS Namespace** từ cấp Launch File.

### Quy chuẩn Đặt Nhãn (Naming Convention)
Toàn bộ tên topic bên trong code sẽ chuyển về dạng **Relative (Tương đối)** (bỏ đi dấu `/` ở đầu). Khi khởi chạy, ROS 2 sẽ tự động gắn Namespace ở ngoài cùng.
- Máy HP số 1: namespace = `hp_1`
- Máy HP số 2: namespace = `hp_2`

KẾT QUẢ TOPIC SẼ TỰ ĐỘNG THÀNH:
| Hệ thống | Đổi Code (Relative) | Tên thực tế lúc chạy ở Máy 1 | Tên thực tế lúc chạy ở Máy 2 |
| :--- | :--- | :--- | :--- |
| **Camera** | `camera/cam0/image_raw` | `/hp_1/camera/cam0/image_raw` | `/hp_2/camera/cam0/image_raw` |
| **AI** | `ai/yolo/detections` | `/hp_1/ai/yolo/detections` | `/hp_2/ai/yolo/detections` |
| **Robot Hardware** | `dobot/joint_states` | `/hp_1/dobot/joint_states` | `/hp_2/dobot/joint_states` |
| **Robot Logic** | `robot/system_status` | `/hp_1/robot/system_status` | `/hp_2/robot/system_status` |
| **Cartridge** | `cartridge/state` | `/hp_1/cartridge/state` | `/hp_2/cartridge/state` |
| **Bơm / Cân** | `scale/result` | `/hp_1/scale/result` | `/hp_2/scale/result` |
| **GUI Controls** | `gui/start_button` | `/hp_1/gui/start_button` | `/hp_2/gui/start_button` |

---

## 3. Các bước Triển khai thực tế

1. **Chuẩn hoá Code (Xoá Dấu Gạch Chéo Đầu):**
   - Rà soát toàn bộ `.cpp` và `.py`. Bất cứ chỗ nào tạo publisher/subscriber/service đang dùng `/topic_name` (Ví dụ `/system_state`), phải xóa dấu `/` ở đầu thành `cartridge/system_state` hoặc `system/start_button`.
   - Lưu ý cấu hình QML trên giao diện cũng phải gọi topic dạng động (gắn chuỗi `"/" + namespace_may + "/" + topic`).

2. **Áp dụng NameSpace vào File Launch:**
   - Trong các file khởi động ở thư mục `launch/`, sử dụng `PushRosNamespace`.
   Ví dụ file Python Launch:
   ```python
   from launch_ros.actions import PushRosNamespace
   
   def generate_launch_description():
       return LaunchDescription([
           PushRosNamespace('hp_1'), # Hoặc lấy từ tham số args khi bash gọi chạy
           Node(package='csi_camera', executable='dual_csi_camera_node'),
           # ...
       ])
   ```

3. **Cấu hình trên Giao diện GUI (C++ & QML):**
   - Sửa file `robot_controller.cpp` để GUI nhận một Tham số cấu hình (vd: biến môi trường `export SYSTEM_NS=hp_1` trong file `.sh`).
   - Lúc khởi tạo `RobotController`, lấy tham số này ra và dính nó vào tên topic khi `create_client` hoặc `create_publisher`.
   - Ví dụ: `this->create_client<...>(std::string("/") + system_ns + "/robot/start_system");`

### Lợi ích: 
- Có thể chạy chung 2 hệ thống trên **cùng một mạng LAN, thậm chí trên CÙNG 1 Máy tính trung tâm** mà không hề xung đột tín hiệu.
- Tái sử dụng source code 100% (Không cần hardcode file tên topic riêng cho từng máy).
