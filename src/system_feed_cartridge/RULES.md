# RULES — Cartridge Feeding System

**File này định nghĩa các nguyên tắc thiết kế KHÔNG ĐƯỢC THAY ĐỔI khi refactor/bảo trì code `cartridge_providesystem_py_node.py`.**

Mỗi rule bên dưới có:
- **WHY** — lý do tại sao rule này tồn tại (sự cố đã xảy ra hoặc giới hạn vật lý)
- **HOW** — cách hiện thực trong code
- **RỦI RO** — hậu quả nếu vi phạm

---

## RULE 1 — Xác nhận vị trí servo bằng `_pos()` đọc về, KHÔNG chỉ tin `_arrived()`

### WHY
`_arrived(servo_id)` chỉ đọc status bit `target_position_reached` từ firmware Festo CMMT-AS. Theo source code thư viện `festo-edcon` upstream:

```python
def target_position_reached(self):
    self.update_inputs()
    return self.telegram.zsw1.target_position_reached
```

**Tolerance được cấu hình trong PNU của drive, KHÔNG nằm trong library Python.** Drive có thể bật flag `target_position_reached=1` khi servo còn cách target vài mm (phụ thuộc "position window" cấu hình trên drive). Đã xảy ra sự cố: InX báo arrived nhưng thực tế còn cách 4-5mm → InY scan chạy → va chạm cơ khí.

### HOW
Mỗi sub-state move servo phải dùng pattern:

```python
elif self._arrived(N) and self._at_position(N, TARGET):
    self._enter_*(NEXT_STATE)
```

Helper `_at_position(servo_id, target_mm, tol=None)` đọc `_pos()` (encoder counts → mm) và so sánh `|pos - target| ≤ tol`. `tol` mặc định = `config.position_tolerance` = **2.0mm**.

### RỦI RO
- Bỏ check → drive flag false-positive → next state chạy khi servo chưa thực sự dừng → va chạm cơ khí, kẹp khay sai vị trí, Cyl1/Cyl2 extend nhầm chỗ.
- Tăng `position_tolerance` lên 5mm hoặc cao hơn → giảm tính chặt chẽ; chỉ làm khi drive overshoot rõ rệt và retry vô ích.

### ÁP DỤNG CHO
Tất cả sub-state trong S1, S2A, S3, S4 có pattern `elif self._arrived(...)`. Xem [`cartridge_providesystem_py_node.py`](scripts/cartridge_providesystem_py_node.py) các vị trí có comment `[BLOCKING-FIX]`.

---

## RULE 2 — Kết nối phần cứng PHẢI song song (parallel), KHÔNG tuần tự

### WHY
`ComModbus(__init__)` của edcon là **blocking TCP connect** — sẽ chờ đến `timeout_ms` (mặc định 3000ms, hiện cấu hình 3000ms) nếu thiết bị offline. Hệ thống có **5 servo + 2 IO module = 7 thiết bị**. Nếu connect tuần tự, worst-case = 7 × 3s = 21s blocking trước khi GUI có thể đọc vị trí. Đã xảy ra: GUI mở chậm, người dùng tưởng node treo.

### HOW
Trong `_connect_hardware()`:

```python
threads = []
for sid, ip in self.config.servo_ips.items():
    t = threading.Thread(target=self._connect_servo, ...)
    threads.append(t)
threads.append(threading.Thread(target=self._connect_io, args=(1, ...)))
threads.append(threading.Thread(target=self._connect_io, args=(2, ...)))
for t in threads: t.start()
for t in threads: t.join(timeout=15)
```

Tất cả 5 servo + 2 IO module connect đồng thời. Tổng thời gian = thời gian thiết bị chậm nhất (~3s nếu offline, <1s nếu online).

### RỦI RO
- Quay về tuần tự → GUI chậm, người dùng nhấn restart nhiều lần → race condition với `_servo_reconnect_loop`.
- Quên `join(timeout=15)` → main thread chạy trước khi tất cả thiết bị connect → motion command đầu fail.

---

## RULE 3 — Gate S4 bằng InX position TRONG khi InY scan

### WHY
S4 (cảm biến quét stack) là **Normally Closed (NC)** — falling edge khi chạm khay. Trong quá trình InY scan từ home → 970mm, S4 có thể bị nhiễu nếu:
- InX bị nudge khỏi `inx_target2` (505.5mm) → tọa độ tray map sai zone → row index sai.
- InY chưa vào vùng quét hợp lệ (< 20mm hoặc > 970mm) → nhiễu cạnh giả.

### HOW
Trong `_s1_iny_scan` mỗi tick:

```python
inx_at_target = self._at_position(1, self.config.inx_target2)
iny_in_valid  = (self.config.iny_scan_valid_min_mm <= iny <=
                 self.config.iny_scan_valid_max_mm)
new_armed     = inx_at_target and iny_in_valid

if self._s4_armed and not inx_at_target:
    self._log_once("S1_SCAN_INX_DRIFT", ...)   # disarm warn

self._s4_armed = new_armed
```

S4 chỉ được "armed" khi **đồng thời**:
1. InX ở `inx_target2` ± `position_tolerance`
2. InY ∈ [`iny_scan_valid_min_mm`, `iny_scan_valid_max_mm`]

Re-evaluate mỗi tick → tự động disarm nếu InX trôi.

### RỦI RO
- Bỏ điều kiện 1 → InX trôi giữa chừng vẫn nhận S4 → row index sai → Cyl1 kẹp sai height.
- Bỏ điều kiện 2 → S4 trigger ngoài vùng quét hợp lệ → fallback row1 nhầm.
- Bỏ disarm khi InX drift → một lần arm là arm mãi.

---

## RULE 4 — Trục InX bị KHOÁ khi InY > `iny_safe_zone`

### WHY
Interlock cơ khí: nếu InX di chuyển ngang khi InY còn ở vị trí cao (chưa rút về home), 2 trục có thể va chạm trong workspace overlap.

### HOW
Hàm `_iny_safe()`:

```python
def _iny_safe(self) -> bool:
    p = self._pos(2)
    return p is not None and p <= self.config.iny_safe_zone
```

Mọi sub-state move InX (vd `_s1_inx_move`, `_s1_inx_10`, `_s2a_inx_500`, `_s2a_inx_10`, `_s2a_inx_20`) phải gọi `if not self._iny_safe(): return` ở đầu hàm.

### RỦI RO
- Bỏ check → InX chạy ngang khi InY còn ở row N → va chạm cụm Cyl1/Cyl2 với khung gia công.
- Nâng `iny_safe_zone` cao quá → InX vào sớm hơn cần thiết → giảm hiệu quả interlock.

---

## RULE 5 — Cylinder extend chỉ được khi đối ứng retracted

### WHY
Mỗi cylinder có 2 sensor: retracted (S9/S15/S21) và extended (S10/S16/S22). Trạng thái "unknown" (cả 2 sensor OFF, cylinder đang trượt giữa chừng) là dấu hiệu hỏng cơ khí hoặc khí nén yếu. Extend trong trạng thái unknown có thể gãy chốt, kẹp khay sai.

### HOW
Ví dụ `_s1_check_s5`:

```python
if self.sensor(S5_OUTPUT_DETECT):
    # Verify Cyl1 RETRACTED trước khi extend
    if not self.sensor(S9_CYL1_RETRACTED):
        self._log_once("S1_CYL1_UNKNOWN", "Cyl1 unknown state")
        return
    self._cyl1_extend()
```

### RỦI RO
- Bỏ check → extend trong unknown state → gãy chốt, kẹt khay.

---

## RULE 6 — Retry cylinder có giới hạn (≤ 5 lần / ~15s), KHÔNG vô hạn

### WHY
Cylinder không retract/extend được trong 3s đầu là dấu hiệu hỏng khí nén hoặc kẹt cơ khí. Retry vô hạn = treo state machine, không escalate.

### HOW
Pattern chuẩn:

```python
if not self._cmd_sent_*:
    self._cyl1_extend()
    self._cyl_retry_t     = time.time() + 3.0
    self._cyl_retry_count = 0
    self._cmd_sent_*      = True

if SENSOR_OK:
    self._enter_*(NEXT)
    return

if time.time() > self._cyl_retry_t:
    self._cyl_retry_count += 1
    if self._cyl_retry_count >= 5:
        self._notify('error', 'Cylinder stuck', ...)
        self._go_gui_confirm()    # hoặc self._error(...)
        return
    self._cyl1_extend()
    self._cyl_retry_t = time.time() + 3.0
```

### RỦI RO
- Bỏ counter → retry mãi → state stuck → operator phải restart node.

---

## RULE 7 — Monitor sensor "đang giữ khay" trong khi servo di chuyển

### WHY
Khi Cyl2 đang giữ khay (S18/S22 ON) và InY/OutY di chuyển sang vị trí khác, nếu tray trượt khỏi kẹp giữa chừng (S18/S22 OFF), code phải phát hiện ngay để dừng — KHÔNG được di chuyển tiếp đến target và retract Cyl1.

### HOW
Ví dụ `_s1_iny_200` (di chuyển InY trong khi Cyl2 đang giữ khay):

```python
if self._cmd_sent_in and not self.sensor(S18_CYL2_EXTENDED):
    self._stop(2)
    self.get_logger().error("S18 OFF khi dang di chuyen — TRAY SLIP!")
    self._notify('error', 'Tray slipped', ...)
    self._go_gui_confirm()
    return
```

### RỦI RO
- Bỏ check → tray rơi giữa hành trình + code vẫn retract Cyl1 → khay rơi sai vị trí, có thể kẹt cơ khí.

---

## RULE 8 — Mỗi pipeline có timeout escalation rõ ràng

### WHY
Nếu một sub-state stuck (không hết điều kiện chuyển), state machine sẽ treo vô hạn nếu không có timeout. Khác pipeline cần khác cách escalate:

| Pipeline | Timeout var | Khi timeout |
|----------|-------------|-------------|
| S1 / S2A (INPUT) | `_step_timeout_in` | `_go_gui_confirm()` — chờ operator |
| S3 (Servo3 feed) | `_step_timeout_s3` | `_error()` — vào ERROR state |
| S4 (Output) | `_step_timeout_s4` | `_error()` — vào ERROR state |

### HOW
Mỗi sub-state move-servo phải set timeout khi gửi lệnh:

```python
self._step_timeout_in = time.time() + self.config.move_timeout   # = 80s
```

Và check ở mỗi tick:

```python
if time.time() > self._step_timeout_in:
    # escalate theo pipeline
```

### RỦI RO
- Quên set timeout → state stuck vô hạn.
- Dùng sai timeout var giữa pipeline → escalation không kích hoạt.

---

## RULE 9 — Sensor cross-check cho cylinder

### WHY
Mỗi cylinder dùng 2 sensor (retracted/extended) để xác minh chéo. Pattern "đã retract" = `(S_retracted ON) AND (S_extended OFF)`, KHÔNG chỉ `S_retracted ON`.

### HOW
Ví dụ `_s1_wait_release`:

```python
if cyl1_ret and not cyl1_ext:    # S9 ON + S10 OFF
    self._enter_in(NEXT)
```

### RỦI RO
- Chỉ check 1 sensor → trạng thái sensor bị stuck/short không phát hiện được → tin nhầm vị trí cylinder.

---

## RULE 10 — `_arrived()` có "ignore window" 0.5s đầu sau `_nb_move()`

### WHY
Drive Festo cần ~50-200ms xử lý lệnh `position_task` rồi mới bật `target_position_reached=0`. Nếu poll `_arrived()` ngay sau `_nb_move()`, có thể đọc trạng thái CỦA LẦN MOVE TRƯỚC (chưa kịp clear) → false-positive.

### HOW
Trong `_nb_move()`:

```python
self._ignore_arrived_{servo_id} = time.time() + 0.5
```

Trong `_arrived()`:

```python
if time.time() < self._ignore_arrived_{servo_id}:
    return False
```

### RỦI RO
- Bỏ ignore window → race condition giữa command gửi xuống drive và status đọc về → sub-state bỏ qua move thật sự, chuyển sang state tiếp theo khi servo chưa di chuyển.

---

## RULE 11 — Vận tốc JOG đọc từ FAS, không hardcode

### WHY
Vận tốc JOG cấu hình trên drive (PNU 11352) phải khớp với vận tốc trong code, nếu không operator nhấn JOG button thấy servo chạy nhanh/chậm khác mong đợi.

### HOW
Trong `_connect_servo()`:

```python
fas_v1 = float(com.read_pnu(11352, 0))
self._fas_jog_vel[sid] = fas_v1
if not self._jog_vel_from_fas and 0.001 <= fas_v1 <= 0.08:
    self._jog_velocity_ms = fas_v1
```

### RỦI RO
- Hardcode vận tốc → khác cấu hình drive → unsafe (vận tốc cao bất ngờ).

---

## RULE 12 — Falling edge cho sensor NC, KHÔNG dùng raw ON/OFF

### WHY
S4 (Scan Stack) và một số sensor khác là **Normally Closed (NC)** — bình thường ON, chạm khay → OFF. Nếu code dùng `if self.sensor(S4):` thì luôn True khi không có khay → sai logic.

### HOW
Trong `_s1_iny_scan`:

```python
s4_now     = self.sensor(S4_SCAN_STACK_P1)
s4_falling = self._s4_prev_in and (not s4_now)
self._s4_prev_in = s4_now

if self._s4_armed and s4_falling:    # phát hiện cạnh xuống
    ...
```

### RỦI RO
- Đọc raw → sai logic NC sensor → trigger nhầm hoặc không trigger.

---

## RULE 13 — Sub-state nguy hiểm KHÔNG được skip qua dispatcher

### WHY
Mỗi sub-state có pre-condition cụ thể (vd InY safe, cylinder retracted). Skip qua bằng cách set `self.state = SystemState.XXX` trực tiếp sẽ bỏ qua các check này.

### HOW
Chỉ dùng `_enter_in()`, `_enter_s3()`, `_enter_s4()` — KHÔNG `self.state_in = SystemState.XXX` trực tiếp.

```python
def _enter_in(self, next_state):
    self.get_logger().info(f"[IN] -> {next_state.name}")
    self.state_in        = next_state
    self._cmd_sent_in    = False    # reset flag để sub-state mới gửi lệnh lại
    self._step_start_in  = 0.0
    self._step_timeout_in = 0.0
```

### RỦI RO
- Skip dispatcher → cờ `_cmd_sent_*` và `_step_timeout_*` không reset → state mới đọc cờ cũ → behavior unpredictable.

---

## RULE 14 — YAML config field PHẢI match attribute trong code

### WHY
Pydantic schema `SystemConfig` trong `config.py` xác định những field nào load được từ YAML. Đã từng có bug: YAML có `inx_target2` nhưng code đọc `self.config.inx_target` → KHÔNG load → chạy với default hardcode.

### HOW
Khi đổi tên field:
1. Đổi trong `config.py` (Pydantic schema)
2. Đổi trong `cartridge_config.yaml`
3. Đổi mọi `self.config.<field>` trong code
4. Test bằng cách đặt giá trị lạ trong YAML, restart, kiểm tra log có pick up không

### RỦI RO
- Mismatch → YAML edit không có hiệu lực → tham số cũ vẫn dùng → debug khó.

---

## RULE 15 — Position tolerance KHÔNG được nới quá 2mm cho hệ thống production

### WHY
Vùng zone của row khít nhau (vd row 1 = 880-970mm, row 2 = 780-879mm — cách 1mm). Tolerance 5mm có thể nhảy nhầm row khi servo dừng ngay biên.

### HOW
- `config.position_tolerance` = **2.0mm** (default — chạm trần RULE 15).
- KHÔNG nới lên 5mm trong production.

### RỦI RO
- Tolerance 5mm + zone width 90mm → 5/90 = 5.5% lệch row → Cyl1 kẹp sai height.

---

## Danh sách servo / sensor mapping

### Servo
| ID | Tên | Mục đích | Limit (mm) |
|----|-----|----------|-----------|
| 1 | InX | Trục ngang vào/ra | 560 |
| 2 | InY | Trục dọc input/scan | 1050 |
| 3 | Servo3 | Platform đẩy khay vào robot | 400 |
| 4 | OutX | Trục ngang output | 600 |
| 5 | OutY | Trục dọc output | 600 |

### Sensor (22 sensors, từ 2 module CPX-AP)
| ID | Tên | Loại | Mục đích |
|----|-----|------|----------|
| 1-3 | S1-S3 BELT | conveyor | Khay đầu/giữa/cuối băng tải |
| 4 | S4 | stack_detect (NC) | Falling edge khi InY chạm khay scan |
| 5 | S5 | tray_detect | Khay đến vị trí cấp robot |
| 6 | S6 | stack_detect | Snapshot có khay output stack |
| 7 | S7 | tray_detect | Khay tại Robot |
| 9 | S9 | cyl_retracted | Cyl1 đã thu |
| 10 | S10 | cyl_extended | Cyl1 đã đẩy ra |
| 15-16 | S15-S16 | cyl_*_2 | Cyl2 retract/extend |
| 17-18 | S17-S18 | platform / feed_ok | S17=Platform có khay, S18=Cấp khay OK |
| 19-22 | S19-S22 | output side | Sensor output stack + Cyl2 output |

Chi tiết đầy đủ ở [`cartridge_config.yaml`](config/cartridge_config.yaml) phần `sensors`.

---

## RULE 16 — STATE 1 / STATE 2 (Pos1) workflow ĐÃ CHỐT — không refactor logic core

### WHY
STATE 1 (cấp khay input) + STATE 2A (thay khay vào robot) đã chạy thực tế ổn định trên hardware production (Pos1: S1=InX, S2=InY, Cyl1, Cyl3, CPX 253 sensors S1-S16). Mọi thay đổi vào logic core 2 state này phải có lý do nghiệp vụ rõ ràng + test đầy đủ — không refactor "đẹp code" hoặc "tối ưu hiệu năng" mà đụng vào dispatcher / transition.

### HOW
Vùng code được khoanh "đã chốt":
- `_dispatch_input()` và toàn bộ sub-state handler `_s1_*`, `_s2a_*`
- `_can_start_s1()`, `_can_start_s2a()` — điều kiện auto-trigger
- `_s2a_cyl3_extend()` + `_check_row1_interlock()` + `_cyl3_safety_check()` — interlock chuỗi Cyl3
- Pattern `_arrived(N) and _at_position(N, TARGET)` đã có ở mọi sub-state move (RULE 1)

Thay đổi cho phép (không cần update rule này):
- Sửa lỗi bug rõ ràng (false-positive sensor, off-by-one)
- Thêm log/diagnostic
- Thêm sensor monitor không ảnh hưởng transition

Thay đổi BẮT BUỘC phải tag `rule-change: state1-2` + test 5 chu kỳ:
- Đổi thứ tự sub-state
- Đổi điều kiện vào sub-state mới
- Đổi vận tốc move trong sub-state
- Đổi sensor trigger logic

### RỦI RO
- Refactor không cần thiết → break workflow đã chạy ổn → mất thời gian debug + downtime sản xuất.
- "Cải thiện" mà không test đủ → regression im lặng chỉ phát hiện sau khi sản xuất chạy.

---

## RULE 17 — CPX 253 và CPX 254 PHẢI độc lập runtime

### WHY
2 IO module Festo CPX-AP nằm trên cùng subnet nhưng phục vụ 2 cụm thiết bị khác nhau:
- **CPX 253** (172.16.11.37) — sensor S1-S16, valve Cyl1/Cyl2/Cyl3, gripper, picker. Phục vụ **Pos1 = STATE 1/2**.
- **CPX 254** (172.16.11.41) — sensor S17-S22 (Platform, FeedOk, OutP2, Cyl2_output). Phục vụ **Pos2 = STATE 3/4**.

Lắp đặt thực tế tuần tự: Pos1 chạy production trước, Pos2 lắp sau. Một module fail/offline KHÔNG được kéo theo module kia.

### HOW
Đã implement trong `_io_bg_loop`:
- 2 nhánh try/except riêng cho module 1 và 2
- Cache riêng: `_io_sensor_cache` vs `_io_sensor_cache_2`
- Ready flag riêng: `_io_ready` vs `_io_ready_2`
- Reconnect thread độc lập: `_reconnect_io1()` / `_reconnect_io2()`

Sensor lookup `_sensor_raw` chia theo sid:
- sid 1-16 → module 1
- sid 17+ → module 2

### RỦI RO
- Gộp 2 cache vào 1 → CPX 254 fail kéo CPX 253 đứng → Pos1 production stop oan.
- Reconnect chung 1 thread → 1 module retry block module kia.
- Sensor lookup không chia rõ → đọc nhầm cache → false sensor reading.

---

## RULE 18 — Hardware presence flag (`*_present`) là CONTRACT, không bypass

### WHY
2 flag YAML/config gate workflow theo hardware đã lắp:
- `cyl3_present` — Cyl3 piston + sensor S13-S16 (trên CPX 253)
- `output_stack_present` — cụm Pos2: Servo3 + OutX/OutY + CPX 254

Khi flag=false, code phải đảm bảo workflow tương ứng KHÔNG chạy — kể cả ở manual button hay auto-trigger. Đây là contract giữa hardware lắp đặt và logic.

### HOW
Mỗi flag check ở 3 chỗ:
1. **Auto-trigger gate**: `_can_start_s2a` / `_can_start_s3` / `_can_start_s4` return False ngay khi flag=false
2. **Workflow step gate**: `_s2a_cyl3_extend` skip sang state kế nếu `cyl3_present=false`
3. **Manual button gate**: `_cb_goto_state` STATE3/STATE4 reject với notify rõ ràng khi flag=false

Khi thêm hardware mới có flag tương tự (vd `gripper_2_present`, `pos3_present`):
- Add field vào `SystemConfig` (`config.py`)
- Default `True`
- Document trong YAML với comment khi nào set false
- Gate ở cả 3 chỗ trên

### RỦI RO
- Bỏ check ở 1 trong 3 chỗ → command bay xuống hardware chưa lắp → no-op silent → state machine kẹt.
- Hardcode flag = True trong code → YAML override không hiệu lực → operator không thể disable khi cần debug.
- Đặt flag ở runtime detect (vd "nếu CPX không connect thì false") thay vì static config → behavior nhảy theo network state, khó reproduce bug.

---

## RULE 19 — Position publish ≥ 10Hz, KHÔNG block bởi state machine

### WHY
GUI hiển thị vị trí servo real-time để operator giám sát. Publish rate quá chậm (< 5Hz) làm GUI nhìn "đứng" sau homing/move, gây hiểu lầm "node treo". Publish loop chia sẻ `_servo_lock` với state machine Modbus calls — nếu publish chiếm lock lâu, JOG/STOP bị block.

### HOW
`_positions_bg_loop` chạy thread riêng `time.sleep(0.1)` (10Hz). `_publish_positions` dùng `_servo_lock.acquire(timeout=0.05)`:
- Nếu lock free → đọc thật từ Modbus
- Nếu lock busy (state machine đang dùng) → fallback `_pos_cache` (giá trị lần đọc trước), continue tới servo kế

Cache update mỗi lần đọc thành công. JOG/STOP có 50ms preempt window → không bao giờ chờ publish.

### RỦI RO
- Giảm rate xuống 2Hz (500ms) → GUI thấy 0.0mm chậm ~1s sau homing → user tưởng treo.
- Bỏ try-acquire, dùng `with self._servo_lock` → publish có thể giữ lock 100-500ms (Modbus timeout) → JOG bấm bị delay perceptible.
- Bỏ cache fallback → publish skip servo này cycle → JSON thiếu key → GUI hiển thị "--" thay vì giá trị cũ.
- Đẩy lên 50Hz (20ms) → Modbus read 5-20ms × 2 servo ≥ cycle → spam lock, không còn window cho JOG.

---

## Quy trình thay đổi rule

Nếu phải đổi một trong các rule trên:

1. **Document lý do** trong commit message (ví dụ: "drive PNU 6XX position window đổi, position_tolerance bumped to 2.0").
2. **Update RULES.md** ngay trong cùng commit.
3. **Test lại** ít nhất 5 chu kỳ STATE 1 + 2A + 3 + 4 đầy đủ.
4. **Tag commit** với prefix `rule-change:` để dễ tìm về sau.

---

*Created: 2026-05-16 — first version after blocking-fix-v2 series of changes.*
*Updated: 2026-05-25 — added RULE 16 (chốt STATE 1/2), 17 (CPX độc lập), 18 (presence flags), 19 (position publish 10Hz).*
