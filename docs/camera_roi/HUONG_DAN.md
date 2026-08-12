# Hướng dẫn lệnh — hệ thống camera Jetson (CAM-H-J-1)

Jetson Orin Nano + IMX477 qua V3Link, ROS 2 **Humble**, AI chạy **TensorRT trên GPU**.
Bản này thay hoàn toàn bản Raspberry Pi 5 + Hailo-8L. Mọi số liệu lấy trực tiếp
từ code đang chạy, không phải ước lượng.

Env đã có sẵn trong `~/.bashrc` qua `~/ros2_ws/ros2_env.sh` (`ROS_DOMAIN_ID=22`).

---

## 0. Nguyên tắc xuyên suốt — MỘT hệ toạ độ duy nhất

Cả hệ thống chỉ có **một** khung hình: **BGR8 640×360 (16:9)** trên
`/cam0HP/image_raw` và `/cam1HP/image_raw`.

```
IMX477 ──V3Link──> /dev/video0,1  RG10 1920x1080
     │
     └─> CUDA debayer + tone + clarity  (v4l2_dual_camera_cuda_node)
              │
              └─> publish  640x360 BGR8  @25 fps  BEST_EFFORT depth 1
                     │
                     ├─> yolo_tensorrt_node ─ letterbox 640x640 ─ TensorRT
                     │        └─> /camXHP/yolo/bounding_boxes   (toạ độ 640x360)
                     │
                     ├─> overlay_bboxes_node ─> /camXHP/image_overlay
                     │
                     ├─> vision_decision_node + vision_roi.yaml ─> chọn row/slot
                     │
                     └─> dataset_capture_gui.py ─> ảnh train (lưu nguyên xi)
```

Ba thứ sau **bắt buộc** cùng hệ 640×360, sai một cái là lệch hết:

| | |
|---|---|
| Ảnh gán nhãn để train | 640×360, lấy từ topic |
| Ảnh dùng để chấm ROI | 640×360, **cùng file ảnh với ảnh train** |
| bbox YOLO publish | 640×360 |

⚠️ **Không dùng `capture_ai_data.sh` / `capture_yolo_gui.py` để lấy data train.**
Tool đó đọc thẳng `/dev/video`, lưu **1280×720**, bỏ qua bước CUDA tone/clarity
của node ROS — màu, độ nét và tỉ lệ đều khác ảnh model thật sự nhìn thấy. Nó chỉ
dùng để soi camera khi hệ thống ROS đang tắt.

---

## 1. Chạy hệ thống

```bash
~/ros2_ws/scripts/start_all.sh          # hoặc icon "Start All ROS 2" trên Desktop
```

Chỉ camera + AI, không cần robot:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch csi_camera dual_camera_system.launch.py
```

Dừng:

```bash
~/ros2_ws/stop_all.sh
```

Kiểm tra camera chạy đúng:

```bash
tail -f ~/ros2_ws/logs/dual_camera_system.log
ros2 topic echo /camera/cam0/health --once
```

Mong đợi `STREAMING ... capture_fps=25.0 ... frame_age_ms=<100`.

Nếu camera không mở được, thường là driver chưa init xong:

```bash
sudo ~/ros2_ws/scripts/camera_tools/prepare_imx477.sh --attempts 1 --settle 10
systemctl status cam-h-j-1-imx477.service
```

---

## 2. Thu ảnh để train

**Hệ thống phải đang chạy** — công cụ nghe topic ROS, không tự mở camera.
Cần màn hình (hoặc X-forward), đây là GUI PyQt5.

```bash
export DISPLAY=:0
python3 ~/ros2_ws/scripts/dataset_capture_gui.py            # PNG không mất mát
python3 ~/ros2_ws/scripts/dataset_capture_gui.py --jpg      # JPG q95, nhẹ ~4 lần
python3 ~/ros2_ws/scripts/dataset_capture_gui.py --jpg 90   # nhẹ ~5 lần
```

Định dạng cố định theo phiên, chọn lúc khởi động — trộn PNG với JPG trong cùng
một thư mục ảnh là nguồn sai lệch âm thầm khi chia train/val. Định dạng đang
dùng hiện trên thanh thông số và ghi vào `session_info.yaml` + `metadata.csv`.

Đo trên ảnh mẫu 640×360:

| | dung lượng | PSNR |
|---|---|---|
| PNG | 384 KB | không mất mát |
| JPG q95 | 102 KB | 39.4 dB |
| JPG q90 | 73 KB | 37.3 dB |
| JPG q85 | 59 KB | 36.0 dB |

q95 là điểm cân bằng hợp lý. Sai lệch nén ở mức đó không ảnh hưởng đo được tới
detection — dataset COCO và hầu hết dataset Ultralytics đều là JPG.

Phím tắt trong cửa sổ:

| | |
|---|---|
| `Space` | lưu 1 ảnh cả 2 cam |
| `0` | chỉ cam0 |
| `1` | chỉ cam1 |
| `Ctrl+L` | đổi thư mục gốc |
| checkbox auto | tự lưu theo chu kỳ, tự bỏ khung hình không đổi |

Tool tự từ chối lưu khi: sai shape (khác 640×360), frame cũ hơn 1 giây, ảnh
trắng/đen bất thường, hoặc đĩa còn dưới 5 GiB. Ảnh lưu ra **không overlay,
không crop, không resize** — chỉ khác nhau ở bước nén.

Thư mục ra:

```
~/Datasets/Jetson_YOLO_Data/jetson_yolo_<ngày_giờ>[_<tên>]/
├── session_info.yaml          # ghi lại toàn bộ thông số pipeline lúc chụp
├── metadata.csv               # mỗi ảnh 1 dòng: thời gian, độ sáng, độ nét…
├── input/
│   ├── images/cam0_input_<nhóm>_000001.png     640x360 (.jpg nếu --jpg)
│   ├── labels/
│   └── classes.txt            # tray, cartridge
└── output/
    ├── images/cam1_output_<nhóm>_000001.png    640x360 (.jpg nếu --jpg)
    ├── labels/
    └── classes.txt            # tray, cartridge, cartridgefall
```

PNG 640×360 khoảng 400 KB/ảnh → 1000 ảnh ≈ 0.4 GB. JPG q95 ≈ 0.1 GB.

Chi tiết cho người train model: xem `TRAIN_SPEC.md` cùng thư mục.

---

## 3. Chấm ROI

Trên Jetson ROI nằm trong **file YAML**, không nằm trong C++ như bản Pi:

```
~/ros2_ws/src/robot_control_main/config/vision_roi.yaml
```

Sửa xong **chỉ cần restart `vision_decision_node`, KHÔNG cần build lại.**

Node đọc `ref_width`/`ref_height` trong file rồi scale ROI sang
`image_width`/`image_height` của nó (đang là 640×360, khai trong
`dual_camera_system.launch.py`). **Đặt `ref_*` = 640×360 để tỉ lệ scale bằng 1.0** —
đó là cách duy nhất đảm bảo ROI khớp tuyệt đối với bbox mà node AI trả về.

### 3.1 Lấy ảnh nền

Dùng đúng ảnh 640×360 vừa thu ở mục 2 — cùng nguồn, cùng tone, cùng hệ toạ độ
với bbox:

```bash
mkdir -p ~/Pictures/roi
cp ~/Datasets/Jetson_YOLO_Data/<phiên>/input/images/cam0_input_*_000001.*  ~/Pictures/roi/cam0.png
cp ~/Datasets/Jetson_YOLO_Data/<phiên>/output/images/cam1_output_*_000001.* ~/Pictures/roi/cam1.png
```

(Phiên chụp bằng `--jpg` thì đổi đuôi đích thành `.jpg` và truyền
`--image ~/Pictures/roi/cam0.jpg` cho `roi_pick.py`/`roi_preview.py`.)

Chọn khung hình khay **đầy và đặt đúng vị trí làm việc**, sáng đều.

### 3.2 Chấm — qua trình duyệt, dùng được qua SSH

```bash
python3 ~/ros2_ws/scripts/camera_tools/roi_pick.py --cam cam0
python3 ~/ros2_ws/scripts/camera_tools/roi_pick.py --cam cam1
```

Rồi mở trên máy có chuột: **http://172.16.11.10:8011**
(script tự in đúng IP khi khởi động).

Script **không resize ảnh** — toạ độ chấm ra chính là toạ độ 640×360.

Vùng phải chấm:

- **cam0**: `outer`, rồi `row1` … `row5`
- **cam1**: `slot1` … `slot10`

Kết quả lưu `~/Pictures/roi/cam0_roi.yaml` và `~/Pictures/roi/cam1_roi.yaml`,
đã kèm sẵn `ref_width: 640` / `ref_height: 360`.

⚠️ **Chấm lại thì phải chấm CẢ HAI cam trong cùng một đợt.** File
`vision_roi.yaml` chỉ có **một** cặp `ref_width`/`ref_height` dùng chung cho cả
`input_tray` lẫn `output_tray`. Trộn ROI cam0 hệ 640×360 với ROI cam1 hệ 640×480
cũ là hỏng toàn bộ khay ra.

⚠️ **Thứ tự vùng phải giữ nguyên như file hiện tại**, robot không tự suy ra được:
- `row1` … `row5` trong file đang chạy là **TRÁI → PHẢI** (row1 x≈157-237, row5
  x≈415-496). Đổi chiều là phải sửa logic robot.
- `slot1` … `slot10` là **thứ tự robot đặt hàng**, khớp pose index 14-23 trong
  `joint_pose_params.yaml` và nút O1-O10 trên GUI.

Mỗi ROI là 4 góc bấm theo vòng quanh, không bắt chéo.

### 3.3 Dán vào `vision_roi.yaml`

`roi_pick.py` xuất dạng phẳng (`row1:`, `slot1:` …), còn node đọc dạng **list**
(`rows:`, `slots:`). Phải chuyển tay khi dán, giữ đúng thứ tự:

```yaml
# roi_pick.py xuất ra          →   dán vào vision_roi.yaml
input_tray:                        input_tray:
  outer: [[..],[..],[..],[..]]       outer: [[..],[..],[..],[..]]
  row1:  [[..],[..],[..],[..]]       rows:
  row2:  [...]                         - [[..],[..],[..],[..]]   # row1
                                       - [...]                   # row2
```

Tương tự `slot1`…`slot10` → `output_tray.slots:` (10 dòng `-`).

`roi_pick.py --cam cam1` **không chấm `output_tray.outer`** (không có trong
profile của nó). Đây là khung bao chống false-positive class `tray` ở mép ảnh,
không cần chính xác từng pixel — quy đổi giá trị cũ sang hệ 640×360 bằng cách
nhân toạ độ Y với 0.75:

```yaml
output_tray:
  outer: [[145, 263], [155, 83], [505, 83], [525, 263]]
```

Nhớ sửa đầu file:

```yaml
ref_width: 640
ref_height: 360
```

### 3.4 Kiểm tra trước khi tin

```bash
python3 ~/ros2_ws/scripts/camera_tools/roi_preview.py --cam cam0
python3 ~/ros2_ws/scripts/camera_tools/roi_preview.py --cam cam1
```

Hệ thống đang chạy thì script lấy bbox thật từ YOLO vẽ chồng lên và **đếm số
cartridge rơi vào từng ROI** — đúng phép tính `vision_decision_node` làm. Ra
`~/Pictures/roi/camX_preview.png`.

Mong đợi: mỗi row/slot có khay đầy phải đếm ra đúng số cartridge nhìn thấy, và
dòng cuối `Cartridge ngoai moi ROI: 0/N`. Nếu ra `KHONG cartridge nao roi vao
ROI` là sai hệ toạ độ.

⚠️ `roi_preview.py` đọc định dạng phẳng của `roi_pick.py`, **không** đọc được
định dạng `rows:`/`slots:` của `vision_roi.yaml`. Chạy nó trên
`~/Pictures/roi/camX_roi.yaml` (mặc định), trước bước chuyển đổi ở 3.3.

### 3.5 ROI neo theo khay — bù camera xê dịch

ROI chấm tay là toạ độ tuyệt đối. Camera trượt vài pixel là tâm cartridge rơi
sang row bên cạnh: YOLO vẫn detect đúng, chỉ riêng bước quy ROI sai. Node có
chế độ neo ROI theo bbox `tray` (class 0) để tự bù.

Camera trên máy này chỉ trượt, không xoay, nên hình chữ nhật là đủ — phép bù
rút gọn thành scale + translate.

**Lấy `anchor`** — chạy `roi_preview.py` khi hệ thống đang chạy và khay đang
nằm đúng vị trí làm việc. Script in sẵn dòng để dán:

```
bbox tray (class 0, score 0.94) -> dan vao input_tray trong vision_roi.yaml:
  anchor: [133, 83, 530, 270]
```

Dán vào `vision_roi.yaml`, mỗi khay một dòng, cùng cấp với `outer`:

```yaml
input_tray:
  anchor: [133, 83, 530, 270]
  outer: [...]
  rows: [...]
```

⚠️ `anchor` phải lấy trên **đúng khung hình đã dùng để chấm ROI**. Lấy ở khung
khác là bù lệch ngay từ đầu.

**Bật chế độ** — đã là mặc định của launch. Tắt để so sánh:

```bash
ros2 launch csi_camera dual_camera_system.launch.py roi_anchor_mode:=static
```

Bật riêng từng khay: khay nào có `anchor` trong `vision_roi.yaml` thì được neo,
khay chưa có tự chạy ROI tĩnh. Log lúc khởi động nói rõ khay nào đang ở chế độ
nào:

```
[VISION] ROI loaded: ... | anchor cam0=tray cam1=static alpha=0.25 max_scale_dev=0.25
```

⚠️ `roi_anchor_mode` chỉ đọc **một lần lúc khởi động**. `ros2 param set` không
có tác dụng — phải launch lại.

Hai tham số tinh chỉnh, sửa trong params của `vision_decision_node`:

```python
'roi_anchor_alpha': 0.25,        # EMA, nhỏ hơn = mượt hơn, bám chậm hơn
'roi_anchor_max_scale_dev': 0.25 # bbox lệch quá ±25% coi như detect sai
```

Kiểm tra khi chạy:

```bash
grep "anchor:" ~/ros2_ws/logs/dual_camera_system.log
```

Log mỗi 5 giây một dòng: `[VISION] cam0 anchor: active scale=1.002,0.998
offset=11.3,-4.1 tray_seen=1 rejected=0`.

| dấu hiệu | nghĩa |
|---|---|
| `scale` ≈ 1.00, `offset` nhỏ | camera đúng vị trí hiệu chuẩn |
| `offset` lớn dần theo ngày | camera đang trôi — siết lại cơ khí |
| `rejected` tăng | bbox tray không đáng tin, node đang giữ transform cũ |
| `CHUA HOI TU` | chưa thấy khay lần nào, đang chạy ROI tĩnh |

Ba hành vi an toàn đã có sẵn: thiếu `anchor` thì tự quay về ROI tĩnh và báo qua
`/vision/roi_status`; mất tray một khung thì giữ nguyên transform thay vì giật
về ROI tĩnh; khay bị nhấc ra thì quên vị trí cũ để khay mới không bị kéo theo.

Gate "có khay hay không" **cố ý vẫn chạy trên ROI tĩnh** — nếu gate đó dùng ROI
đã bù theo chính bbox tray thì thành vòng lặp kín, ROI có thể trượt theo một
detection sai rồi tự xác nhận mình.

Hình học này có test chạy độc lập, không cần camera:

```bash
cd ~/ros2_ws && colcon test --packages-select robot_control_main \
  --ctest-args -R roi_anchor_test
```

### 3.6 Nạp ROI mới

```bash
pkill -f vision_decision_node          # launch tự respawn sau 3 s
ros2 topic echo /vision/roi_status --qos-durability transient_local --once
```

`roi_status` rỗng = ROI nạp OK. Topic này là latched, chỉ publish một lần lúc
node khởi động — thiếu cờ `--qos-durability transient_local` là `echo` treo.
Log node cũng in `[VISION] ROI loaded: 2 outer + 5 rows + 10/10 slots`.

Không cần `colcon build`: bản trong `install/` hiện là **symlink** tới file
trong `src/`, sửa `src/` là node đọc ngay. Nếu có ai build lại bằng
`colcon build` **không** có `--symlink-install`, symlink bị thay bằng bản copy —
lúc đó sửa `src/` sẽ không có tác dụng. Kiểm tra:

```bash
ls -l ~/ros2_ws/install/robot_control_main/share/robot_control_main/config/vision_roi.yaml
# phải thấy: ... -> /home/nhan/ros2_ws/src/robot_control_main/config/vision_roi.yaml
```

---

## 4. Đổi model AI

Model là file TensorRT `.engine`, không phải `.hef`.

```bash
ls -lh ~/models/*.engine
```

Đổi tạm cho một lần chạy:

```bash
ros2 launch csi_camera dual_camera_system.launch.py \
  cam0_model:=/home/nhan/models/model_moi.engine
```

Đổi vĩnh viễn — sửa `default_value` trong
`~/ros2_ws/src/csi_camera/launch/dual_camera_system.launch.py`
(`cam0_model` / `cam1_model`), hoặc export biến trước khi chạy `start_all.sh`:

```bash
export CAM0_AI_MODEL=/home/nhan/models/model_moi.engine
~/ros2_ws/scripts/start_all.sh
```

⚠️ Số class trong `class_names` của launch file **phải khớp đúng** số class của
engine, nếu không node ném lỗi ngay lúc khởi động và respawn vô hạn.

Kiểm tra engine trước khi dùng:

```bash
/usr/src/tensorrt/bin/trtexec --loadEngine=/home/nhan/models/model_moi.engine --skipInference
```

Cần thấy: đúng **1 input + 1 output**, cả hai **FP32**, input `1x3x640x640`,
output `1x(4+số_class)xN`. Engine có NMS nhúng sẵn (`--nms`, EfficientNMS) sẽ
**không chạy được** — node tự làm NMS.

Build engine từ ONNX:

```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=/home/nhan/models/model_moi.onnx \
  --saveEngine=/home/nhan/models/model_moi.engine \
  --fp16 --memPoolSize=workspace:2048 --builderOptimizationLevel=5 \
  --timingCacheFile=/home/nhan/models/model_moi.timing.cache --skipInference
```

Mất khoảng 80 giây. `.engine` gắn chặt với đúng máy + đúng phiên bản TensorRT —
build trên Jetson này, không copy từ máy khác sang.

---

## 5. Build lại code

```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --executor sequential \
  --packages-select yolo_tensorrt_ros2 csi_camera bbox_drawer_cpp
```

⚠️ Workspace này dùng layout **isolated** (mặc định) — **không** thêm
`--merge-install` như bản Pi. Build 1 luồng vì Orin Nano 8 GB dễ hết RAM khi
compile CUDA/TensorRT song song.

---

## 6. Lệnh kiểm tra nhanh

```bash
# tốc độ topic (image_raw là BEST_EFFORT — ros2 topic hz không có cờ qos)
source ~/ros2_ws/install/setup.bash
ros2 topic hz /cam0HP/image_raw          # ~25 Hz
ros2 topic hz /cam0HP/yolo/bounding_boxes # ~20 Hz (bị cap)
ros2 topic echo /cam0HP/yolo/bounding_boxes --once

# sức khoẻ camera và AI
ros2 topic echo /camera/cam0/health --once
ros2 topic echo /camera/cam0/ai_health --once
ros2 topic echo /vision/roi_status --qos-durability transient_local --once  # rỗng = ROI OK

# quyết định vision
ros2 topic echo /vision/input_tray/selected_row
ros2 topic echo /vision/output_tray/selected_slot

# xem ảnh có bbox
ros2 run rqt_image_view rqt_image_view       # chọn /cam0HP/image_overlay

# tiến trình đang chạy
pgrep -af "v4l2_dual_camera_cuda_node|yolo_tensorrt_node|overlay_bboxes|vision_decision"

# ai đang giữ camera
sudo fuser -v /dev/video0 /dev/video1

# nhiệt độ, GPU, RAM
tegrastats --interval 1000

# log
tail -f ~/ros2_ws/logs/dual_camera_system.log
tail -f ~/ros2_ws/logs/start_all_console.log

# dừng sạch
~/ros2_ws/stop_all.sh
```

---

## 7. Thông số hiện tại

| | |
|---|---|
| Cảm biến | 2 × IMX477 qua V3Link, `/dev/video0` (input) + `/dev/video1` (output) |
| Raw | 1920×1080 RG10, exposure 43000, gain 66 |
| Xử lý | CUDA debayer + tone + clarity (cam0 0.45 / cam1 0.60) |
| Publish ROS | **640×360 @25 fps**, `bgr8`, BEST_EFFORT depth 1 |
| Model input | 640×640×3 FP32 NCHW, letterbox pad **140 px trên + 140 px dưới** |
| Inference | TensorRT FP16 trên GPU, cap 20 fps/cam |
| Model cam0 | `/home/nhan/models/data_input_hp_final_2_fp16.engine` — 2 class, conf 0.60 |
| Model cam1 | `/home/nhan/models/data_output_hp1.engine` — 3 class, conf 0.30 |
| NMS | trong node, class-aware, IoU 0.45, tối đa 300 box |
| ROI | `robot_control_main/config/vision_roi.yaml` — 5 row + 10 slot + 2 outer |
| ROS | Humble, `ROS_DOMAIN_ID=22`, `rmw_fastrtps_cpp` |

---

## 8. Khác biệt so với bản Raspberry Pi 5

Đọc kỹ nếu bạn quen bản cũ — hầu hết lệnh cũ **không còn đúng**.

| | Pi 5 + Hailo-8L | Jetson Orin Nano |
|---|---|---|
| ROS | Jazzy, domain 42 | Humble, domain 22 |
| Camera | libcamera/rpicam, 2028×1520 → 1280×960 | V4L2 RG10 trực tiếp, 1920×1080 |
| Publish | 640×480 (4:3) @20 fps | **640×360 (16:9) @25 fps** |
| Topic | `/camXFunai/image_raw` | `/camXHP/image_raw` |
| AI | Hailo-8L, `.hef`, NMS on-chip | GPU, TensorRT `.engine`, NMS trong node |
| Letterbox | pad 80/80 | **pad 140/140** |
| Class cam1 | 4 class | **3 class** (`tray, cartridge, cartridgefall`) |
| Lệch class id | có (Hailo 1-based, node trừ 1) | **không** — id là argmax trực tiếp |
| ROI | hard-code C++, phải build lại | `vision_roi.yaml`, chỉ restart |
| Số slot cam1 | 9 | **10** |
| Thu data | `collect_data.sh` (CLI) | `dataset_capture_gui.py` (GUI) |
| Chấm ROI | `roi_web.sh` / `roi_draw.sh` | `roi_pick.py` + `roi_preview.py` |
| Build | `--merge-install` | isolated, **không** `--merge-install` |
| Nhiệt độ | `vcgencmd measure_temp` | `tegrastats` |

Các script `collect_data.*`, `roi_web.*`, `roi_draw.*`, `roi_capture.sh`,
`tray_bbox.*` trong thư mục này là **bản Pi, không chạy trên Jetson** (gọi
`rpicam-still`, topic `Funai`, xuất khối C++ cho `robot_logic_node_dual.cpp` vốn
không tồn tại ở đây). Giữ để tham khảo, đừng chạy.
Tương tự: `ros2_ws/scripts/camera_tools/grab_roi_frames.py`, `camera_roi.py`,
`roi_picker*.py` và `camera_tools/README.md` cũng là di sản Pi (1280×720,
`rpicam-still`) — chỉ `roi_pick.py` và `roi_preview.py` là bản Jetson.
