# Thông số train lại YOLO — hệ thống Jetson Orin Nano (CAM-H-J-1)

Cập nhật: 2026-08-11. Mọi số liệu lấy trực tiếp từ code đang chạy
(`csi_camera`, `yolo_tensorrt_ros2`, `robot_control_main`) và từ metadata của
ONNX/engine đang deploy, không phải ước lượng.

Thay hoàn toàn bản Raspberry Pi 5 + Hailo-8L. Nếu bạn quen bản cũ, đọc mục 9
trước.

---

## 1. Ảnh đầu vào — thứ cần gán nhãn

| | |
|---|---|
| Kích thước | **640 × 360** (16:9) |
| Định dạng file | PNG 8-bit, 3 kênh BGR (không mất mát) |
| Nguồn | topic ROS `/cam0HP/image_raw`, `/cam1HP/image_raw` |
| Thư mục | `~/Datasets/Jetson_YOLO_Data/<phiên>/input/images/` và `.../output/images/` |
| Công cụ thu | `python3 ~/ros2_ws/scripts/dataset_capture_gui.py` |

Ảnh được lưu **nguyên xi** từ đúng topic mà node AI đọc, cùng QoS, không qua bước
xử lý nào thêm. Pixel trong file = pixel model nhìn thấy.

**Gán nhãn trên đúng ảnh 640×360 này. Không resize, không crop, không pad trước
khi gán nhãn.**

⚠️ **Không lấy data từ `capture_ai_data.sh` / `capture_yolo_gui.py`.** Tool đó
đọc thẳng `/dev/video`, lưu 1280×720 và bỏ qua bước CUDA tone/clarity của node
ROS — sai cả tỉ lệ khung lẫn tông màu so với ảnh model thật sự nhận.

---

## 2. Đường đi của ảnh (để tái lập nếu cần)

```
IMX477  ──V3Link──>  /dev/video0, /dev/video1
  └─ V4L2 raw 1920x1080 RG10 (Bayer 10-bit)
       exposure = 43000        phơi sáng cố định, KHÔNG auto
       gain     = 66           analog gain cố định
       └─ CUDA debayer + tone curve + chroma denoise
            └─ CUDA clarity (luma-edge lift): cam0 0.45 / cam1 0.60
                 └─ thu nhỏ 1920x1080 -> 640x360 BGR8
                      └─ publish ROS @25 fps, BEST_EFFORT depth 1
```

Không dùng AE/AWB tự động — độ sáng và tông màu ổn định giữa các lần chụp.

Node xử lý: `csi_camera/v4l2_dual_camera_cuda_node`
(`src/csi_camera/src/v4l2_dual_camera_cuda_node.cu`).

---

## 3. Tiền xử lý trước khi vào model

Node AI letterbox **bên trong** trước khi đẩy vào TensorRT
(`yolo_tensorrt_node.cpp::preprocess`):

```
640x360  →  letterbox  →  640x640
scale    = min(640/640, 640/360) = 1.0
new size = 640 x 360        ← KHÔNG hề co giãn
padding  = 140 px trên + 140 px dưới, màu xám (114,114,114)
sau đó   : BGR → RGB, chia 255, đổi sang FP32 NCHW
```

Vì bề rộng đã đúng 640, letterbox ở đây là **padding thuần tuý, không có phép
nội suy nào**. Model nhìn thấy pixel ở đúng tỉ lệ gốc.

Bbox trả về được trừ padding, chia scale và clamp về hệ toạ độ **640×360** trước
khi publish.

**Khi train**: đặt `imgsz=640` và để pipeline tự letterbox. Ultralytics làm đúng
như trên (pad 114, giữ tỉ lệ). Không tự pad trước — sẽ bị pad chồng pad.

---

## 4. Model hiện tại

| | cam0 — khay đầu vào | cam1 — khay đầu ra |
|---|---|---|
| Engine | `/home/nhan/models/data_input_hp1.engine` | `/home/nhan/models/data_output_hp1.engine` |
| ONNX/PT | `data_input_hp1.onnx` / `.pt` | `data_output_hp1.onnx` / `.pt` |
| Kiến trúc | YOLOv8 (Ultralytics) | YOLOv8 (Ultralytics) |
| Dataset gốc | `tray_input_hp2_dataset` | `tray_output_datasets` |
| Input tensor | FP32 NCHW `1×3×640×640` | FP32 NCHW `1×3×640×640` |
| Output tensor | FP32 `1×6×N` (4 + 2 class) | FP32 `1×7×N` (4 + 3 class) |
| Số class | **2** | **3** |
| NMS | **KHÔNG có trong engine** | **KHÔNG có trong engine** |
| Precision | FP16 (I/O vẫn FP32) | FP16 (I/O vẫn FP32) |

Phần cứng: **GPU Ampere của Jetson Orin Nano**, TensorRT 10.3, không có
accelerator rời.

NMS do node tự làm, **class-aware** — box `tray` cố ý bao trọn các box
`cartridge`, NMS class-agnostic sẽ xoá mất cartridge hợp lệ.

Ngưỡng lúc chạy (khai trong `dual_camera_system.launch.py`, chỉnh được):

```
cam0: confidence_threshold = 0.60
cam1: confidence_threshold = 0.30
cả hai: nms_threshold = 0.45, max_detections = 300
max_inference_fps = 20  (camera publish 25, AI bỏ bớt frame)
```

---

## 5. Bảng class — PHẢI giữ nguyên thứ tự

### cam0 (khay đầu vào) — 2 class

| id | ý nghĩa |
|---|---|
| 0 | `tray` — khung bao cả khay (mỗi khung hình đúng 1 cái) |
| 1 | `cartridge` — đối tượng đếm cho row1..row5 |

### cam1 (khay đầu ra) — 3 class

| id | ý nghĩa |
|---|---|
| 0 | `tray` — khay |
| 1 | `cartridge` — cartridge đặt đúng |
| 2 | `cartridgefall` — cartridge đổ / sai hướng |

Khay đầu vào có **5 row**, khay đầu ra có **10 slot**.

### Không có lệch class id

Node đọc thẳng output thô của YOLOv8: class id = **argmax** trên các kênh
`4 … 4+nc` của tensor `1×(4+nc)×N`, rồi publish nguyên số đó dưới dạng chuỗi
trong `Detection2D.results[0].hypothesis.class_id`.

Khác hẳn bản Hailo cũ (class id 1-based rồi node trừ 1). Trên Jetson **không có
phép trừ nào** — id trong `data.yaml` lúc train chính là id robot logic thấy.

`vision_decision_node` so sánh bằng chuỗi số (`"1"` = cartridge). Đảo thứ tự
class trong `data.yaml` là hỏng toàn bộ logic robot.

---

## 6. Yêu cầu bàn giao

- Weights `.pt`
- `.onnx` export với **`nms=False`**, `imgsz=640`, `batch=1`, `dynamic=False`,
  `simplify=True`
- `.engine` build **trên chính Jetson này** (engine gắn chặt với GPU + phiên bản
  TensorRT, copy từ máy khác sang không nạp được)
- File `classes.txt` liệt kê class theo đúng thứ tự, mỗi dòng một tên

Engine phải thoả **đúng** những điều kiện sau, nếu không node ném lỗi lúc khởi
động và respawn vô hạn:

| | |
|---|---|
| Số IO tensor | đúng **2** (1 input + 1 output) |
| Kiểu dữ liệu | cả hai **FP32** |
| Shape input | `1 × 3 × 640 × 640` |
| Shape output | `1 × (4 + số_class) × N` |
| NMS nhúng | **không được có** (EfficientNMS / `--nms` là hỏng) |

Lệnh export và build:

```bash
# trên máy train
yolo export model=best.pt format=onnx imgsz=640 batch=1 dynamic=False \
     simplify=True nms=False

# trên Jetson
/usr/src/tensorrt/bin/trtexec \
  --onnx=/home/nhan/models/<ten>.onnx \
  --saveEngine=/home/nhan/models/<ten>.engine \
  --fp16 --memPoolSize=workspace:2048 --builderOptimizationLevel=5 \
  --timingCacheFile=/home/nhan/models/<ten>.timing.cache --skipInference

# kiểm tra lại
/usr/src/tensorrt/bin/trtexec --loadEngine=/home/nhan/models/<ten>.engine --skipInference
```

Build mất khoảng 80 giây. `--fp16` chỉ bật FP16 cho phần tính bên trong, IO vẫn
FP32 — đúng yêu cầu của node.

Sau khi thay engine, cập nhật `class_names` trong
`~/ros2_ws/src/csi_camera/launch/dual_camera_system.launch.py` cho khớp số class.

---

## 7. Thu thập thêm dữ liệu

Hệ thống ROS phải đang chạy (công cụ nghe topic, không tự mở camera):

```bash
~/ros2_ws/scripts/start_all.sh
export DISPLAY=:0
python3 ~/ros2_ws/scripts/dataset_capture_gui.py
```

`Space` lưu cả 2 cam, `0`/`1` lưu từng cam, checkbox auto để chụp theo chu kỳ và
tự bỏ khung hình không đổi.

Tool tự từ chối lưu khi sai shape (khác 640×360), frame cũ hơn 1 giây, ảnh
trắng/đen bất thường, hoặc đĩa còn dưới 5 GiB.

Mỗi phiên sinh ra `session_info.yaml` (ghi lại toàn bộ thông số pipeline lúc
chụp) và `metadata.csv` (mỗi ảnh một dòng: thời gian, độ sáng trung bình, độ
nét). Giữ hai file này kèm dataset — đó là bằng chứng ảnh được chụp đúng
pipeline nào.

PNG 640×360 khoảng 400 KB/ảnh → 1000 ảnh ≈ 0.4 GB.

---

## 8. ROI phải cùng hệ toạ độ với model

`vision_decision_node` đếm cartridge bằng cách kiểm tra tâm bbox có nằm trong
ROI hay không. ROI để trong
`~/ros2_ws/src/robot_control_main/config/vision_roi.yaml`, có `ref_width`/
`ref_height` và được node scale sang `image_width`/`image_height` (640×360).

**Chấm ROI trên đúng ảnh 640×360 đã thu ở mục 7, đặt `ref_width: 640` /
`ref_height: 360` để tỉ lệ scale bằng 1.0.** Đó là cách duy nhất đảm bảo ROI
khớp tuyệt đối với bbox model trả về.

Quy trình chi tiết: xem mục 3 trong `HUONG_DAN.md`.

Đổi độ phân giải publish của camera thì **phải** đổi cả `image_width`/
`image_height` của `vision_decision_node` trong launch file **và** chấm lại ROI
**và** train lại model. Ba thứ đó đi liền nhau.

---

## 9. Khác biệt so với bản Raspberry Pi 5 + Hailo

| | Pi 5 + Hailo-8L | Jetson Orin Nano |
|---|---|---|
| Độ phân giải publish | 640 × 480 | **640 × 360** |
| Tỉ lệ khung | 4:3 | **16:9** |
| fps | 20 | 25 (AI cap 20) |
| Topic | `/camXFunai/image_raw` | `/camXHP/image_raw` |
| Letterbox | pad 80 trên + 80 dưới | **pad 140 trên + 140 dưới** |
| Định dạng model | `.hef`, UINT8 NHWC | `.engine`, **FP32 NCHW** |
| NMS | on-chip trong HEF | **trong node, class-aware** |
| Class cam0 | 2 (`tray`, `cartridge`) | 2 — giữ nguyên |
| Class cam1 | 4 (`tray`, `cartridge_ok`, `misoriented`, `cartridge_fall`) | **3** (`tray`, `cartridge`, `cartridgefall`) |
| Lệch class id | có (Hailo 1-based) | **không** |
| Số slot khay ra | 9 | **10** |
| ROI | hard-code C++ | `vision_roi.yaml` |

Dataset của Pi (640×480, 4:3) **không dùng lại được trực tiếp**: vừa đổi tỉ lệ
khung vừa đổi FOV dọc, phân bố đầu vào lệch hẳn. Nhãn cũ có thể tái sử dụng nếu
ảnh gốc còn ở độ phân giải cao và bạn render lại đúng khung 16:9 của Jetson,
nhưng ép co 640×480 → 640×360 thì vật thể bị bóp dọc, không nên.
