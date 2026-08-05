#!/usr/bin/env python3
"""
Cartridge Loading System — ROS 2 + festo-edcon

═══════════════════════════════════════════════════════════════════════════════
HARDWARE INSTALL STATUS (đọc trước khi sửa code)
═══════════════════════════════════════════════════════════════════════════════
Cụm Pos1 (đã lắp, production):
  • Servo S1=InX (172.16.11.35), S2=InY (172.16.11.36)
  • CPX 37 (172.16.11.37) — sensor S1-S16, valve Cyl1/Cyl3
  • Cyl3 piston + sensor S13-S16
  → STATE 1, STATE 2A chạy đầy đủ với interlock Cyl3.

Cụm Pos2 (đã lắp):
  • Servo S3=Servo3/Platform (172.16.11.38)
  • Servo S4=OutX (172.16.11.39), S5=OutY (172.16.11.40)
  • CPX 41 (172.16.11.41) — sensor S17-S28, valve Cyl2/Cyl4/Cyl5
  → STATE 3, STATE 4 được enable qua flag `output_stack_present: true`.

→ Khi user nói "đã lắp servo 3/4/5" / "đã lắp CPX 41" / "đã lắp xong cụm
  Pos2/output" → đổi YAML `output_stack_present: false → true` và restart.
  Không cần sửa code. Các check `_can_start_s3/s4`, manual STATE3/4 button,
  `_s2a_cyl3_extend` sẽ tự active khi flag=true.

→ Tương tự: nếu Cyl3 piston tháo ra (vd để service), đổi
  `cyl3_present: true → false` — workflow STATE 2 vẫn chạy nhưng skip step
  cố định khay Cyl3.
═══════════════════════════════════════════════════════════════════════════════

AUTO/AI mode: Đọc sensor thực từ IO module, tự động trigger STATE 1/2/3/4 khi
              đủ điều kiện cảm biến.
MANUAL mode : Đọc sensor THẬT (giống AUTO — không còn simulation từ v8).
              STATE 1 và STATE 3 KHÔNG auto-trigger — chỉ chạy khi operator
              nhấn nút STATE1/STATE3 trên GUI; nhấn nút → check sensor → enter.
              STATE 2/4 trigger qua nút GUI (simulateDoneTrayInput/Output) —
              pub /robot/done_tray_input và /robot/done_tray_output để set
              _input_tray_done / _s4_trigger flag, hệ thống tự enter S2A/S4.
              Sau khi state hoàn tất → về IDLE, không tự kích chain mới.

Manual/Auto mode redesign v8:
  [V8-1] sensor() và _snap() luôn đọc real IO module (bỏ sim_sensors dict).
  [V8-2] _cb_goto_state: STATE1/2/3/4 button đều manual-only, đều check
         sensor thật trước khi enter.
  [V8-3] Bảng "SENSOR SIGNAL DISPLAY" trong GUI là LED indicator read-only.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool, String, Int32
from enum import Enum
import time
from typing import Optional, Any
import yaml
import json
import threading
import os
import signal
import traceback

try:
    from edcon.edrive.com_modbus import ComModbus
    from edcon.edrive.motion_handler import MotionHandler
    from edcon.utils.logging import Logging as EdconLogging
    EDCON_AVAILABLE = True
except ImportError:
    ComModbus = MotionHandler = EdconLogging = None
    EDCON_AVAILABLE = False

try:
    from cpx_io.cpx_system.cpx_ap.cpx_ap import CpxAp
    CPXAP_AVAILABLE = True
except ImportError:
    CpxAp = None
    CPXAP_AVAILABLE = False

# ─── Constants ───────────────────────────────────────────────────
COUNTS_PER_MM        = 1000
CYLINDER_TIMEOUT_S   = 15.0
OUTPUT_DETECT_WAIT_S = 25.0
INY_JOG_VEL          = 40
OUTY_ROW1_LIMIT_MM   = 590.0
TRAY_ROBOT_CHECK_TIMEOUT_S = 3.0
JOG_OUTPUT_TIMEOUT_S = 120.0
POSITION_TOLERANCE_HOME = 5.0   # mm — tolerance kiểm tra vị trí gần home
HOME_SPEED              = 30    # mm/s cho position_task về 0

# ─── Sensor IDs — matching sensors.yaml ─────────────────────────
S1_BELT_START      = 1
S2_BELT_MID        = 2
S3_BELT_END        = 3
S4_SCAN_STACK_P1   = 4
S5_OUTPUT_DETECT   = 5
S6_CHECK_TRAY_P1   = 6
S7_TRAY_AT_ROBOT   = 7
# S8 reserved
S9_CYL1_RETRACTED  = 9
S10_CYL1_EXTENDED  = 10
# S11/S12 ATV Run/Fault — monitored by vfd_logic_node, not used here
S13_OUT1_TRAYPOS1  = 13
S14_OUT2_TRAYPOS1  = 14
S15_CYL3_RETRACTED = 15
S16_CYL3_EXTENDED  = 16
S17_PLATFORM       = 17
S18_FEED_OK        = 18
S19_CHECK_TRAY_P2  = 19
S20_SCAN_STACK_P2  = 20
S21_CYL2_RETRACTED = 21
S22_CYL2_EXTENDED  = 22
# S23/S24 reserved
S25_CYL4_RETRACTED = 25
S26_CYL4_EXTENDED  = 26
S27_CYL5_RETRACTED = 27
S28_CYL5_EXTENDED  = 28


# ─── State Enum ──────────────────────────────────────────────────

class SystemState(Enum):
    IDLE            = "idle"
    ERROR           = "error"
    HOMING          = "homing"
    HOMING_RUNNING  = "homing_running"

    # ══════════════════════════════════════════════════════════════
    # [RENAME 2026-05-16 — for-Agent-readers]
    # Enum state đã được đổi tên theo SEMANTIC (mục đích) thay vì NUMERIC
    # (vị trí số mm). Mapping cũ → mới dùng khi đọc commit/PR cũ:
    #
    #   Tên cũ                  →  Tên mới
    #   ----------------------- → ------------------------------------
    #   [S1+INX+MOVE]           →  S1_INX_MOVE_POS_PICK
    #   [S1+INY+50]             →  S1_INY_PICK_TRAY_UP
    #   [S1+INY+200]            →  S1_INY_PLACE_TRAY_ROBOT
    #   [S1+INX+10]             →  S1_INX_WAIT_TRAY_DONE
    #   [S1+RETRY+INX+500]      →  S1_INX_TRY_POS_PICK
    #   [S2A+INX+500]           →  S2A_INX_MOVE_POS_PICK
    #   [S2A+INY+200+CYL1]      →  S2A_POS_PLACE_TRAY_ROBOT_CYL1
    #   [S2A+INY+10]            →  S2A_INY_HOME
    #   [S2A+INX+10]            →  S2A_INX_PLACE_TRAY_OUT_POS1
    #   [S2A+INY+10+FINAL]      →  S2A_INY_FINAL
    #   [S2A+INX+20]            →  S2A_WAIT_NEW_TRAY
    #
    # GHI CHÚ (cho Agent khác):
    #   1. Tên cũ trong bảng trên dùng '+' thay '_' để code search không
    #      tự nhận nhầm là identifier (vd grep "S2A_INY_10" sẽ trả 0 hit).
    #   2. Tên method handler cũng được đổi tương ứng (vd `_s1_inx_move` →
    #      `_s1_inx_move_pos_pick`) để enum + method + log trace nhất quán.
    #   3. String value (lowercase) cũng được rename tương ứng (vd
    #      "s1_inx_move" → "s1_inx_move_pos_pick") để khớp GUI parse từ
    #      topic /system_state. Đã verify không file GUI/QML/test nào
    #      hardcode tên cũ.
    #   4. Backup trước rename: tag pre-blocking-fix-v2-2026-05-16 ở d2b97eb.
    #   5. Xem RULES.md (Servo / Sensor mapping table) để biết ngữ cảnh.
    # ══════════════════════════════════════════════════════════════

    # STATE 1
    S1_CONFIRM_SAFE     = "s1_confirm_safe"
    S1_INX_MOVE_POS_PICK         = "s1_inx_move_pos_pick"
    S1_WAIT_ARRIVE      = "s1_wait_arrive"
    S1_INY_SCAN         = "s1_iny_scan"
    S1_WAIT_STOP_S4     = "s1_wait_stop_s4"
    S1_INY_TO_ROW       = "s1_iny_to_row"
    S1_CHECK_S5         = "s1_check_s5"
    S1_FALLBACK_RETRACT = "s1_fallback_retract"
    S1_FALLBACK_WAIT_INY = "s1_fallback_wait_iny"
    S1_WAIT_GUI_CONFIRM = "s1_wait_gui_confirm"
    S1_INX_TRY_POS_PICK    = "s1_inx_try_pos_pick"
    S1_RETRY_JOG        = "s1_retry_jog"
    S1_CYL1_EXTEND      = "s1_cyl1_extend"
    S1_INY_PICK_TRAY_UP           = "s1_iny_pick_tray_up"
    S1_INY_PLACE_TRAY_ROBOT          = "s1_iny_place_tray_robot"
    S1_WAIT_RELEASE     = "s1_wait_release"
    S1_WAIT_S7         = "s1_wait_s7"
    S1_INX_WAIT_TRAY_DONE           = "s1_inx_wait_tray_done"
    S1_COMPLETE         = "s1_complete"
    S1_RETRY_SCAN_HOME  = "s1_retry_scan_home"

    # STATE 2
    S2A_CHECK_INTERLOCK   = "s2a_check_interlock"
    S2A_INX_MOVE_POS_PICK           = "s2a_inx_move_pos_pick"
    S2A_POS_PLACE_TRAY_ROBOT_CYL1      = "s2a_pos_place_tray_robot_cyl1"
    S2A_WAIT_OUTPUT_CLEAR    = "s2a_wait_output_clear"
    S2A_WAIT_CYL_EXT          = "s2a_wait_cyl_ext"
    S2A_INY_HOME            = "s2a_iny_home"
    S2A_INX_PLACE_TRAY_OUT_POS1            = "s2a_inx_place_tray_out_pos1"
    S2A_INY_JOG_OUTPUT    = "s2a_iny_jog_output"
    S2A_INY_TARGETROW    = "s2a_iny_targetrow"
    S2A_WAIT_CYL1_RET     = "s2a_wait_cyl1_ret"
    S2A_CYL3_EXTEND       = "s2a_cyl3_extend"
    S2A_INY_FINAL      = "s2a_iny_final"
    S2A_WAIT_NEW_TRAY            = "s2a_wait_new_tray"
    S2A_RETRY_SCAN_HOME   = "s2a_retry_scan_home"
    S2A_COMPLETE          = "s2a_complete"

    # STATE 3
    S3_CHECK_OUTXY_SAFE  = "s3_check_outxy_safe"
    S3_OUTX_WAIT         = "s3_outx_wait"
    S3_SERVO3_TARGET1    = "s3_servo3_target1"
    S3_CHECK_S17         = "s3_check_s17"
    S3_WAIT_S17          = "s3_wait_s17"
    S3_CYL4_EXTEND       = "s3_cyl4_extend"
    S3_WAIT_GUI_CONFIRM  = "s3_wait_gui_confirm"
    S3_SERVO3_FEED       = "s3_servo3_feed"
    S3_WAIT_S18          = "s3_wait_s18"
    S3_COMPLETE          = "s3_complete"

    # STATE 4
    S4_CYL2_RETRACT_READY = "s4_cyl2_retract_ready"
    S4_CHECK_OUTY_SAFE   = "s4_check_outy_safe"
    S4_OUTX_TARGET2      = "s4_outx_target2"
    S4_OUTY_PICK         = "s4_outy_pick"
    S4_CYL2_EXTEND       = "s4_cyl2_extend"
    S4_CYL4_RETRACT      = "s4_cyl4_retract"
    S4_OUTY_TARGET1      = "s4_outy_target1"
    S4_OUTX_TARGET3      = "s4_outx_target3"
    S4_CHECK_S19          = "s4_check_s19"
    S4_CYL5_SYNC         = "s4_cyl5_sync"
    S4_OUTY_ROW1         = "s4_outy_row1"
    S4_OUTY_SCAN_S20     = "s4_outy_scan_s20"
    S4_OUTY_DROP         = "s4_outy_drop"
    S4_CYL2_RETRACT      = "s4_cyl2_retract"
    S4_CYL5_EXTEND_HOLD  = "s4_cyl5_extend_hold"
    S4_OUTY_OUTX_HOME    = "s4_outy_outx_home"
    S4_CYL5_RETRACT_RELEASE = "s4_cyl5_retract_release"
    S4_OUTX_HOME         = "s4_outx_home"
    S4_RETRY_SCAN_HOME   = "s4_retry_scan_home"
    S4_COMPLETE          = "s4_complete"


# ─── Config ──────────────────────────────────────────────────────

from config import SystemConfig as CartridgeConfig

# ─── Main Node ───────────────────────────────────────────────────

class CartridgeSystem(Node):

    def __init__(self, config: CartridgeConfig):
        super().__init__('cartridge_providesystem')
        self.config = config

        # Hardware
        self._servo_lock        = threading.Lock()
        self.servos: dict       = {}
        self.zero_offset: dict  = {}
        # Cache trạng thái ready_for_motion mỗi servo để skip Modbus query lặp lại
        # trong các lệnh JOG/move liên tiếp. Invalidate khi STOP hoặc fault.
        self._ready_until: dict = {}     # sid -> expiry timestamp
        self._ready_ttl         = 3.0    # giây — drive thường giữ ready nếu không có lỗi
        # Cache vị trí (mm) lần đọc gần nhất — dùng cho _publish_positions khi lock
        # bận để fallback giá trị thay vì miss frame và GUI nhấp nháy "--".
        self._pos_cache: dict   = {}     # sid -> mm
        self.io_module          = None
        self.io_module_2        = None
        self._io_sensor_cache: list = []
        self._io_sensor_cache_2: list = []
        self._io_ready          = False
        self._io_ready_2        = False
        self._io_bg_lock        = threading.Lock()
        self._sensor_latch_until: dict = {}
        self._sensor_on_latch_s = max(0.0, float(getattr(self.config, 'sensor_on_latch_s', 0.10)))
        # Sensor scan/confirm và các cặp limit cylinder phải phản ánh mức hiện
        # tại. Latch ON trên limit đôi làm trạng thái cũ (RETRACTED) còn giữ
        # trong khi trạng thái mới (EXTENDED) đã ON, tạo giả S21=S22=1 hoặc
        # S25=S26=1 và khiến state machine kẹt cross-check.
        self._sensor_no_latch_sids = {
            S18_FEED_OK, S20_SCAN_STACK_P2,
            S21_CYL2_RETRACTED, S22_CYL2_EXTENDED,
            S25_CYL4_RETRACTED, S26_CYL4_EXTENDED,
            S27_CYL5_RETRACTED, S28_CYL5_EXTENDED,
        }

        # Motion flags
        self._inx_moving = False
        self._iny_moving = False
        # Lệnh position non-blocking đang còn hiệu lực, dùng để PAUSE dừng tức
        # thời và RESUME phát lại đúng target của step hiện tại.
        # sid -> (target_mm, velocity, continuous_update)
        self._active_servo_moves: dict = {}
        self._paused_servo_moves: dict = {}
        self._pause_started_at = 0.0
        self._pause_restart_homing = False
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # ⚠️  CRITICAL ZONE — đọc memory feedback_critical_code_zones.md trước khi sửa.
        # Timestamp lệnh motion gần nhất per-servo (sid → time.time()).
        # _publish_positions skip Modbus khi (now - last_motion) > _idle_skip_modbus_s
        # → cache reuse → JOG cold press nhanh.
        # INVARIANT: mọi LOOP kéo dài chạm servo (homing, scan, long move) PHẢI
        # refresh self._servo_motion_t[sid] = time.time() mỗi iteration < 2s,
        # nếu không GUI sẽ freeze position cache (bug 2026-05-27).
        # Cập nhật bởi: _jog, _nb_move, _home_all (poll loops + delay loops).
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        self._servo_motion_t: dict = {}
        self._idle_skip_modbus_s    = 2.0   # giây — sau motion command này thì còn đọc tươi
        # GUI cần feedback actual position liên tục cho cả 5 trục. Trước đây
        # idle-skip làm S3/S4/S5 reuse cache sau 2s vì các state output không
        # refresh _servo_motion_t đều như S1/S2.
        self._live_position_servos  = set(range(1, 6))
        # Servo đang JOG continuous (jog_task duration=0.0): set bởi _jog,
        # clear bởi _stop. _publish_positions bỏ qua idle-skip cho các sid
        # trong set này → vị trí luôn đọc tươi suốt thời gian giữ JOG, không
        # cần refresh _servo_motion_t mỗi tick. Fix freeze sau 2s giữ nút.
        self._jog_active: set        = set()
        # Drive warm-up gate sau referencing_task (FAS firmware quirk):
        #   -1.0 = pending flush; 0.0 = done; >0.0 = đang chờ settle (timestamp)
        # Khi drive vừa làm xong homing, lệnh position_task đầu tiên có thể bị
        # drive "nuốt" — nó accept lệnh, báo target_position_reached=True ngay,
        # nhưng không di chuyển vật lý. Phải gọi stop_motion_task() trước để
        # flush internal queue. _s1_confirm_safe sẽ tự lo chuyện này.
        self._drive_warm_t = -1.0
        # Per-servo warm-up flag: set True sau homing, clear sau flush trong _nb_move.
        # Giải quyết FAS quirk cho JOG move (state machine đã có gate riêng).
        self._need_warm_up: dict = {}

        # ── INPUT side state machine ──────────────────────────────
        self.state_in           = SystemState.IDLE
        self._cmd_sent_in       = False
        self._step_start_in     = 0.0
        self._step_timeout_in   = 0.0
        self._s4_armed          = False
        self._current_row       = 0
        self._s5_retry          = 0
        self._inx_arrived       = False
        self._30s_timeout       = 0.0
        self._tray_robot_check_start   = 0.0
        self._s10_warn_t        = 0.0
        self._cyl_retry_t       = 0.0
        self._row1_pos          = 0.0
        self._row1_key          = 1
        self._place_delay_start = 0.0
        self._s1_scan_noise_retry = 0
        self._s4_prev_in        = False

        # STATE 2A runtime
        self._s6_snapshot       = False
        self._s13_snapshot      = False
        self._s14_snapshot      = False
        self._s15_snapshot      = False
        self._s16_snapshot      = False
        self._output_row        = 0
        self._output_target_pos = 0.0
        self._s4_armed_out      = False
        self._s4_prev_out       = False
        # One-shot recheck request from the dedicated GUI ACCEPT button.
        # It never overrides S4; State 2 only resumes after a fresh S4=OFF.
        self._s2_output_full_accept = False
        self._s2a_s6off_phase   = 0   # 0=idle, 1=fast→950 @ vel150, 2=slow→r1_target @ vel50

        # ── STATE S3 runtime ─────────────────────────────────────────
        self.state_s3           = SystemState.IDLE
        self._cmd_sent_s3       = False
        self._step_start_s3     = 0.0
        self._step_timeout_s3   = 0.0
        self._cyl4_retry_t      = 0.0
        self._cyl4_retract_sent = False
        # True = ép Servo3 về home 0mm ở S3_SERVO3_TARGET1 (sau overshoot 850 chưa
        # thấy S18). Fresh trigger = False → S17 ON thì giữ vị trí, đẩy tiếp.
        self._s3_force_home     = False
        self._s3_finish_after_home_check = False
        self._s3_output_loaded  = False

        # ── STATE S4 runtime ─────────────────────────────────────────
        self.state_s4           = SystemState.IDLE
        self._cmd_sent_s4       = False
        self._step_start_s4     = 0.0
        self._step_timeout_s4   = 0.0
        self._s4_trigger        = False
        self._outy_jog_start    = 0.0
        self._outy_jog_pos      = 0.0
        # S4 scan S20 (NO, rising edge)
        self._s20_prev          = False
        self._s20_armed         = False
        self._s4_scan_noise_retry = 0
        # Cyl2 retry timer RIÊNG — không dùng chung _cyl_retry_t với chain input
        # (S4 chạy song song STATE 1/2, share biến = race condition)
        self._cyl2_retry_t      = 0.0
        self._cyl5_retry_t      = 0.0

        # ── Shared / Other ───────────────────────────────────────────
        self._motion_busy       = False
        self._last_cartridge_busy_published = None
        self._robot_last_seen   = 0.0   # timestamp of last /robot/motion_busy msg
        self._robot_connected   = False # True when robot node is actively publishing
        self._s10_off_time       = 0.0
        self._s10_prev           = False

        # Cyl3 safety watchdog state — force retract khi S13+S14 cùng OFF
        # State của _cyl3_safety_check: '' | 'extending' | 'extended' | 'retracting' | 'retracted'
        self._cyl3_safety_active    = ''
        self._cyl3_safety_last_fire = 0.0
        # Latch: khi S13+S14 cùng OFF → retract Cyl3 và chốt.
        # Chỉ xóa latch khi: manual extend từ GUI, hoặc auto mode + S6 ON.
        self._cyl3_s13s14_latch     = False

        # Cyl3 feedback monitor — đối chiếu lệnh extend/retract với S15/S16
        self._cyl3_expected         = None   # None | "extended" | "retracted"
        self._cyl3_cmd_time         = 0.0
        self._cyl3_mismatch_warned  = False
        self._cyl3_confirmed_logged = False

        # STATE 2 scan helpers
        self._row1_occupied         = False  # snapshot tại scan start (S16 ON + S6 ON)
        self._s4s6_mismatch_seen    = False  # S6 ON nhưng S4 ko trigger → fallback row1; sau S2A_COMPLETE sẽ pause

        # Homing completion flag (set by bg thread, consumed by main loop)
        self._homing_done_event  = threading.Event()
        self._homing_result      = False
        self._homing_abort       = threading.Event()  # set by STOP to cancel homing thread
        self._homing_generation  = 0
        self._homing_thread      = None
        self._servo_home_abort   = set()              # per-servo JOG home abort flags
        self._servo_home_generation = {}              # immutable token per Servo-card HOME run
        # Serializes STOP cancellation with the final "homed" state write.
        # Without this guard, a stale HOME thread can restore zero_offset just
        # after STOP cleared it, making the next START incorrectly skip homing.
        self._homing_guard_lock  = threading.Lock()
        self._homing_ref_offset  = {}                 # cleared when hardware homing must be repeated
        self._manual_state_cmd_lock = threading.Lock()
        self._last_manual_state_cmd = ''
        self._last_manual_state_cmd_t = 0.0

        # JOG velocity — only affects JOG; state/homing velocities fixed by FAS firmware
        self._jog_velocity_ms  = 0.05   # m/s (default, overwritten by FAS read on first connect)
        self._jog_velocity_max = 0.08   # m/s (hard limit per FAS firmware)
        self._jog_vel_from_fas = False  # flag: already initialized from drive PNU
        self._fas_jog_vel: dict = {}    # per-servo FAS JOG velocity (m/s), keyed by sid

        # Tray tracking
        self.stack_row_index  = 0
        self._tray_loaded_ack = False

        # Operation
        self.state           = SystemState.IDLE
        self.operation_mode  = 'manual'   # 'auto' | 'ai' | 'manual'
        self._input_tray_done     = False
        self._state1_enabled = False
        self._gui_confirmed  = False
        # Must exist before ROS subscriptions become active.  MODE and START
        # are separate topics and may arrive immediately after node startup;
        # _cb_mode uses this flag to converge a pending START into AUTO/AI
        # homing.  Without an explicit initial value that callback can crash
        # the entire cartridge node before /cartridge/homing_done is emitted.
        self._system_running = False
        self._system_paused  = False  # True = state machine + motion đã dừng tức thời
        self._pause_pending  = False  # Legacy compatibility; instant PAUSE luôn để False
        self._input_trays_empty_debounce_count = 0
        self._drain_armed_after_s2 = False
        self._drain_state = False

        # Watchdog
        self._watchdog_last_tick = time.time()
        self._notify_throttle: dict = {}
        self._guide_logged: set = set()

        # ROS Publishers
        qos = QoSProfile(depth=1)
        self.pub_state          = self.create_publisher(String, '/system_state',                                          qos)
        self.pub_new_tray       = self.create_publisher(Bool,   '/cartridge_providesystem/new_tray_loaded',        qos)
        self.pub_newtray_output = self.create_publisher(Bool,   '/cartridge_providesystem/new_trayoutput_loaded',  qos)
        self.pub_gui_notify     = self.create_publisher(String, '/providesystem/gui_notify',  qos)
        self.pub_servo_pos      = self.create_publisher(String, '/providesystem/servo_positions', qos)
        self.pub_sensors        = self.create_publisher(String, '/providesystem/sensors_state', qos)
        self.pub_busy_cartridge = self.create_publisher(Bool, '/cartridge/busy', qos)
        self.pub_busy_cartridge_pos2 = self.create_publisher(Bool, '/cartridge/pos2_busy', qos)
        self.pub_input_trays_empty = self.create_publisher(Bool, '/cartridge/input_trays_empty', qos)
        self.pub_drain = self.create_publisher(Bool, '/cartridge/drain', qos)
        self.pub_current_mode   = self.create_publisher(String, '/providesystem/current_mode', qos)        
        self.pub_robot_mode     = self.create_publisher(Int32, '/robot/set_mode', qos)
        self.pub_vfd_run        = self.create_publisher(Bool,   '/vfd/cmd_run', qos)
        self._vfd_current_cmd   = False

        qos_latching = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_config_data    = self.create_publisher(String, '/providesystem/config_data', qos_latching)
        self._config_mtime_ns   = self._config_file_mtime_ns()
        # homing_done dùng QoS latching để subscriber (vfd_logic_node) start sau
        # nhận được last state (True nếu đã homing xong). Default state = False
        # ngay khi node init → vfd_logic biết phải gate AUTO cho tới khi user
        # nhấn START → homing → publish True.
        self.pub_homing_done    = self.create_publisher(Bool, '/cartridge/homing_done', qos_latching)
        self._publish_homing_done(False)

        # ROS Subscribers
        self.create_subscription(Bool,   '/system/start_button',             self._cb_start,              qos)
        self.create_subscription(Bool,   '/system/stop_button',              self._cb_stop,               qos)
        self.create_subscription(Bool,   '/system/soft_stop',                self._cb_soft_stop,          qos)
        self.create_subscription(Bool,   '/system/pause_button',             self._cb_pause,              qos)
        self.create_subscription(Bool,   '/system/resume_button',            self._cb_resume,             qos)
        self.create_subscription(Bool,   '/robot/motion_busy',               self._cb_motion_busy,          qos)
        self.create_subscription(Bool,   '/robot/done_tray_output',          self._cb_done_tray_output,     qos)
        self.create_subscription(Bool,   '/robot/done_tray_input',           self._cb_done_tray_input,      qos)

        self.pub_gripper_status = self.create_publisher(Bool, '/robot/gripper_status', 10)
        self.pub_picker_status  = self.create_publisher(Bool, '/robot/picker_status', 10)
        self.pub_cyl_loadcell_status = self.create_publisher(Bool, '/robot/cyl_loadcell_status', 10)
        self.create_subscription(Bool, '/robot/gripper_cmd', self._cb_gripper_cmd, 10)
        self.create_subscription(Bool, '/robot/picker_cmd', self._cb_picker_cmd, 10)
        self.create_subscription(Bool, '/robot/cyl_loadcell_cmd', self._cb_cyl_loadcell_cmd, 10)

        self.create_subscription(String, '/providesystem/gui_confirm',       self._cb_gui_confirm,        qos)
        self.create_subscription(String, '/providesystem/jog_cmd',           self._cb_jog,                qos)
        self.create_subscription(String, '/providesystem/set_operation_mode',self._cb_mode,               qos)
        self.create_subscription(Int32, '/robot/set_mode', self._cb_robot_mode, qos)
        self.create_subscription(String, '/providesystem/goto_state',        self._cb_goto_state,         qos)
        self.create_subscription(String, '/providesystem/update_config',     self._cb_update_config,      qos)
        self.create_subscription(String, '/providesystem/get_config',        self._cb_get_config,         qos)
        self.create_subscription(String, '/providesystem/set_target_row',   self._cb_set_target_row,     qos)
        self.create_subscription(String, '/providesystem/cyl_cmd',           self._cb_cyl_cmd,            qos)

        self._connect_hardware()

        control_period = max(0.01, float(getattr(self.config, 'control_loop_period_s', 0.02)))
        self.create_timer(control_period, self._safe_control_loop)
        self.create_timer(1.0, self._config_file_watchdog)
        
        # Positions publisher chạy trong thread riêng
        # để Modbus timeout KHÔNG block ROS control loop
        self._pos_thread_stop = False
        threading.Thread(
            target=self._positions_bg_loop,
            daemon=True,
            name="pos_publisher"
        ).start()

        self.create_timer(5.0,  self._watchdog)

        # Sync initial mode (default 'manual') tới robot node — fix race condition:
        # cartridge default = manual, robot default = manual_mode_{false} (auto-like).
        # Nếu user không đổi mode rõ ràng → robot vẫn nghĩ auto → fire chain HOME khi
        # cartridge homing xong. Publish lặp 3 lần (1s/lần) để chắc chắn robot subscriber
        # đã sẵn sàng nhận (QoS depth=1, không transient_local).
        self._initial_mode_publish_count = 0
        self._initial_mode_timer = self.create_timer(1.0, self._publish_initial_mode_safe)

        self.get_logger().info("CartridgeSystem node started")

    def _publish_initial_mode_safe(self):
        """Publish operation_mode hiện tại tới robot 3 lần (1s/lần) ngay sau khi
        khởi động để đảm bảo sync mode dù robot subscribe trễ. Tự cancel timer
        sau lần thứ 3."""
        msg = Int32()
        if self.operation_mode == 'auto': msg.data = 1
        elif self.operation_mode == 'ai': msg.data = 2
        else: msg.data = 3
        self.pub_robot_mode.publish(msg)
        self._initial_mode_publish_count += 1
        if self._initial_mode_publish_count == 1:
            self.get_logger().info(
                f"[INIT] Sync initial mode to robot: {self.operation_mode.upper()}"
            )
        if self._initial_mode_publish_count >= 3:
            self._initial_mode_timer.cancel()

    def _safe_control_loop(self):
        """
        Wrapper bảo vệ cho vòng lặp điều khiển chính (20Hz, timer 0.05s).
        Bắt mọi exception không mong muốn và chuyển hệ thống sang trạng thái ERROR
        thay vì để node bị crash. Giúp hệ thống không bị treo hoàn toàn khi có lỗi
        bất ngờ trong logic state machine.
        """
        try:
            self._control_loop()
        except Exception as e:
            err_msg = f"FATAL CONTROL LOOP ERROR: {str(e)}"
            self.get_logger().error(err_msg)
            self.get_logger().error(traceback.format_exc())
            self._error(err_msg)

    # ══════════════════════════════════════════════════════════════
    # Hardware
    # ══════════════════════════════════════════════════════════════

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # ⚠️  CRITICAL ZONE — đọc memory feedback_critical_code_zones.md trước khi sửa.
    # INVARIANT: KHÔNG t.join() chờ hardware connect. Control loop +
    # position publisher PHẢI start NGAY sau spawn thread (bug 2026-05-27:
    # join(timeout=15) làm GUI lên rồi nhưng sensor trống 15-20s).
    # Mỗi servo có thread monitor RIÊNG trong _servo_monitor_one — KHÔNG
    # gom thành 1 loop tuần tự (3 servo offline = block 9s/vòng).
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    def _connect_hardware(self):
        """
        Fire-and-forget hardware connection.
        Each device connects in its own daemon thread; we do NOT join, so the
        ROS control loop + position publisher start IMMEDIATELY and don't wait
        for offline hardware to time out. Devices that come online publish as
        soon as their connect thread succeeds; offline ones keep retrying via
        the per-servo monitor / _io_bg_loop reconnect.
        """
        if EDCON_AVAILABLE:
            if EdconLogging:
                EdconLogging()
            for sid, ip in self.config.servo_ips.items():
                threading.Thread(
                    target=self._connect_servo, args=(sid, ip, 1),
                    daemon=True, name=f"connect_s{sid}",
                ).start()
            # Per-servo monitor: each servo gets its own thread, so an offline
            # servo's Modbus timeout never blocks the others.
            threading.Thread(
                target=self._servo_reconnect_loop, daemon=True, name="servo_reconnect"
            ).start()
        else:
            self.get_logger().warn("Simulation mode (edcon not installed)")

        if CPXAP_AVAILABLE:
            # One permanent supervisor per module.  Unlike the previous
            # three-shot startup connect, these loops keep retrying when a CPX
            # is powered after start_all.sh and also recover runtime drops.
            threading.Thread(
                target=self._io_supervisor_one, args=(1, self.config.io_ip),
                daemon=True, name="io_supervisor_1",
            ).start()
            io2_ip = getattr(self.config, 'io_ip_2', "172.16.11.41")
            threading.Thread(
                target=self._io_supervisor_one, args=(2, io2_ip),
                daemon=True, name="io_supervisor_2",
            ).start()
            # Start IO read loop unconditionally — it no-ops while io_module
            # is None and picks up automatically once a connect thread succeeds.
            threading.Thread(target=self._io_bg_loop, daemon=True, name="io_bg").start()
        else:
            self.get_logger().error(
                "CPX-AP driver unavailable (install festo-cpx-io) — "
                "valve and sensor reads are disabled"
            )

    def _connect_io(self, idx: int, ip: str, attempts: int = 1) -> bool:
        """
        Thử kết nối IO Module Festo CPX-AP qua EtherNet/IP.

        Hàm này chỉ thực hiện số lần hữu hạn để không giữ thread quá lâu.
        `_io_supervisor_one()` là vòng retry vô hạn, có backoff và độc lập cho
        từng module.
        idx=1 → Module chính (S1–S16, valve cylinder 1/3, gripper, picker)
        idx=2 → Module phụ output side (S17–S28, valve cylinder 2/4/5)
        Khi kết nối Module 1 thành công:
          1. Reset gripper + picker về trạng thái NHẢ an toàn (5/3 valve, ch0-3).
          2. Smart init Cyl3 theo sensor thực tế S6/S15/S16 (nếu cyl3_present=true).
          Cyl1/Cyl2 GIỮ NGUYÊN state (không reset — RULE 16 chốt workflow Pos1).
        """
        attempts = max(1, int(attempts))
        for attempt in range(1, attempts + 1):
            try:
                # cycle_time=None → tắt IOThread background của CpxAp (mặc định
                # refresh diagnosis_status mỗi 10ms). Chúng ta không dùng diagnosis
                # → tiết kiệm ~100 Modbus calls/s/module, nhường socket cho
                # _io_bg_loop read_channels và state machine write_channels.
                mod = CpxAp(ip_address=ip, cycle_time=None)
                if idx == 1:
                    valve = self._valve_module_from_cpx(mod)
                    valve_desc = (f"{getattr(valve, 'name', '?')}@"
                                  f"{getattr(valve, 'position', '?')}") if valve else 'NONE'
                    with self._io_bg_lock:
                        self.io_module = mod
                    self.get_logger().info(
                        f"Connected CPX IO Module 1. VALVE={valve_desc}. "
                        "Keeping valve states unchanged (only read)."
                    )
                else:
                    di_desc = [
                        f"{getattr(m, 'name', '?')}@{getattr(m, 'position', '?')}"
                        for m in self._io2_di_modules_from_cpx(mod)
                    ]
                    valve = self._valve_module_from_cpx(mod)
                    valve_desc = (f"{getattr(valve, 'name', '?')}@"
                                  f"{getattr(valve, 'position', '?')}") if valve else 'NONE'
                    with self._io_bg_lock:
                        self.io_module_2 = mod
                    self.get_logger().info(
                        f"IO 2 mapping: DI={di_desc} | VALVE={valve_desc}"
                    )
                self.get_logger().info(f"IO {idx} {ip} OK")
                return True
            except Exception as e:
                with self._io_bg_lock:
                    if idx == 1:
                        self.io_module = None
                        self._io_ready = False
                    else:
                        self.io_module_2 = None
                        self._io_ready_2 = False
                self.get_logger().warn(
                    f"IO {idx} {ip} attempt {attempt}/{attempts}: {e}"
                )
                if attempt < attempts:
                    time.sleep(3.0)
        return False

    def _io_supervisor_one(self, idx: int, ip: str):
        """Retry one CPX forever without blocking the other devices or ROS.

        Backoff is capped at 15s so a module powered on later is discovered
        promptly, while an intentionally absent optional module does not flood
        logs.  Reconnection only restores the driver/cache; it never writes a
        valve or resumes motion automatically.
        """
        retry_delay = 5.0
        while rclpy.ok() and not getattr(self, '_pos_thread_stop', False):
            with self._io_bg_lock:
                module = self.io_module if idx == 1 else self.io_module_2
            if module is None:
                if self._connect_io(idx, ip, attempts=1):
                    retry_delay = 5.0
                    self.get_logger().info(
                        f"IO {idx} supervisor: connected; background monitoring active"
                    )
                else:
                    self.get_logger().warn(
                        f"IO {idx} {ip} offline — retry in {retry_delay:.0f}s"
                    )
                    time.sleep(retry_delay)
                    retry_delay = min(15.0, retry_delay * 2.0)
                    continue
            else:
                retry_delay = 5.0
            time.sleep(1.0)

    def _init_cyl3_state(self):
        """
        Smart Cyl3 init khi CPX 253 vừa connect — đồng bộ Cyl3 với cảm biến khay
        theo policy `S6 ON ↔ S16 ON` (tray detect mirror Cyl3 extended).

        Logic:
          • S6 ON  → EXTEND ngay (giữ khay) — bất kể S15/S16 hiện tại.
          • S6 OFF → RETRACT (chừa chỗ cho khay sắp hạ).

        Sau init, _cyl3_safety_check (mỗi tick control loop) tiếp tục đồng bộ
        runtime — operator thao tác khay là Cyl3 tự update.

        Chỉ chạy nếu cyl3_present=true. Đọc sensor TRỰC TIẾP từ io_module (cache
        _io_bg_loop chưa start tại thời điểm này).
        """
        if not self._conf('cyl3_present', True):
            return
        if not self.io_module:
            return
        try:
            channels = []
            with self._io_bg_lock:
                for sub in self.io_module.modules:
                    if sub.is_function_supported("read_channels"):
                        ch = sub.read_channels()
                        if isinstance(ch, list):
                            channels.extend(ch)
            if len(channels) < 16:
                self.get_logger().warn(
                    f"[INIT-CYL3] Sensor read trả {len(channels)} channel (< 16) — skip smart init"
                )
                return

            s6  = bool(channels[5])    # S6_CHECK_TRAY_P1
            s15 = bool(channels[14])   # S15_CYL3_RETRACTED
            s16 = bool(channels[15])   # S16_CYL3_EXTENDED
            sensor_str = f"S6={'ON' if s6 else 'OFF'} S15={'ON' if s15 else 'OFF'} S16={'ON' if s16 else 'OFF'}"

            if s6:
                self.get_logger().info(
                    f"[INIT-CYL3] {sensor_str} → S6 ON (có khay) → EXTEND giữ khay"
                )
                self._cyl3_extend()
            else:
                self.get_logger().info(
                    f"[INIT-CYL3] {sensor_str} → S6 OFF (không khay) → RETRACT chừa chỗ"
                )
                self._cyl3_retract()
        except Exception as e:
            self.get_logger().warn(f"[INIT-CYL3] exception: {e}")

    def _setup_cpx_valves_for_auto_ai(self):
        """
        Setup/initialize CPX 253 valves when starting AUTO/AI mode:
          1. Gripper & Picker to NHẢ (safe initial states)
          2. Cylinder 1 & 2 SMART init theo sensor — giữ nguyên state qua node restart
          3. Smart init Cylinder 3 based on S6 sensor
        """
        self.get_logger().info("[CPX-SETUP] Initializing CPX valves for AUTO/AI mode...")
        # 1. Gripper & Picker
        if self.io_module:
            try:
                with self._io_bg_lock:
                    mod = self._io1_valve_module_locked()
                    if mod is None:
                        raise RuntimeError("CPX37 VABX valve manifold not found/ambiguous")
                    mod.reset_channel(0); mod.set_channel(1)  # nhả gripper
                    mod.reset_channel(2); mod.set_channel(3)  # nhả picker
                    mod.reset_channel(8); mod.set_channel(9)  # nhả cyl6/loadcell
                    self.pub_cyl_loadcell_status.publish(Bool(data=True))
                    self.get_logger().info("[CPX-SETUP] Gripper + Picker + Cyl6 → NHẢ (safe)")
            except Exception as e:
                self.get_logger().error(f"[CPX-SETUP] Failed to init gripper/picker: {e}")

        # 2. Cylinder 1 & 2 — smart init giữ state
        self._init_cyl12_state()

        # 3. Cylinder 3
        self._init_cyl3_state()

    def _init_cyl12_state(self):
        """
        Smart Cyl1/Cyl2 init khi START AUTO/AI — đọc sensor feedback và re-assert
        coil tương ứng để GIỮ vị trí vật lý hiện tại qua node restart.

        Mirror pattern của _init_cyl3_state. Trước đây force RETRACT vô điều kiện
        làm xi lanh đang giữ khay bị thu về → đánh rơi.

        Logic per cylinder:
          • EXTENDED sensor ON, RETRACTED OFF → re-assert EXTEND coil
          • RETRACTED ON, EXTENDED OFF       → re-assert RETRACT coil
          • Cả 2 OFF (đang transition / lệch) hoặc cả 2 ON (lỗi sensor) →
            default safe = RETRACT + log warn

        Cyl1 sensors: S9_CYL1_RETRACTED (mod1 ch8) + S10_CYL1_EXTENDED (mod1 ch9)
        Cyl2 sensors: S21_CYL2_RETRACTED (mod2 ch4) + S22_CYL2_EXTENDED (mod2 ch5)

        Module 2 chưa lắp (Pos2 pending) → bỏ qua Cyl2, GIỮ NGUYÊN coil.
        """
        # ── Cyl1 ──
        if self.io_module:
            try:
                channels1 = []
                with self._io_bg_lock:
                    for sub in self.io_module.modules:
                        if sub.is_function_supported("read_channels"):
                            ch = sub.read_channels()
                            if isinstance(ch, list):
                                channels1.extend(ch)
                if len(channels1) >= 10:
                    s9  = bool(channels1[8])   # S9_CYL1_RETRACTED
                    s10 = bool(channels1[9])   # S10_CYL1_EXTENDED
                    ss = f"S9={'ON' if s9 else 'OFF'} S10={'ON' if s10 else 'OFF'}"
                    if s10 and not s9:
                        self.get_logger().info(f"[INIT-CYL1] {ss} → EXTENDED → re-assert EXTEND (giữ khay)")
                        self._cyl1_extend()
                    elif s9 and not s10:
                        self.get_logger().info(f"[INIT-CYL1] {ss} → RETRACTED → re-assert RETRACT")
                        self._cyl1_retract()
                    else:
                        self.get_logger().warn(f"[INIT-CYL1] {ss} → sensor bất thường → default RETRACT")
                        self._cyl1_retract()
                else:
                    self.get_logger().warn(f"[INIT-CYL1] read {len(channels1)} ch < 10 → default RETRACT")
                    self._cyl1_retract()
            except Exception as e:
                self.get_logger().warn(f"[INIT-CYL1] exception: {e} → default RETRACT")
                self._cyl1_retract()

        # ── Cyl2 (cần Module 2 cho sensor S21/S22) ──
        if not self.io_module_2:
            self.get_logger().info("[INIT-CYL2] Module 2 chưa connect → giữ nguyên coil (no-op)")
            return
        try:
            with self._io_bg_lock:
                channels2 = self._read_io2_sensor_channels_locked()
            # S21 = cache2[4], S22 = cache2[5]
            if len(channels2) >= 6:
                s21 = bool(channels2[4])  # S21_CYL2_RETRACTED
                s22 = bool(channels2[5])  # S22_CYL2_EXTENDED
                ss = f"S21={'ON' if s21 else 'OFF'} S22={'ON' if s22 else 'OFF'}"
                if s22 and not s21:
                    self.get_logger().info(f"[INIT-CYL2] {ss} → EXTENDED → re-assert EXTEND (giữ khay)")
                    self._cyl2_extend()
                elif s21 and not s22:
                    self.get_logger().info(f"[INIT-CYL2] {ss} → RETRACTED → re-assert RETRACT")
                    self._cyl2_retract()
                else:
                    self.get_logger().warn(f"[INIT-CYL2] {ss} → sensor bất thường → default RETRACT")
                    self._cyl2_retract()
            else:
                self.get_logger().warn(f"[INIT-CYL2] read {len(channels2)} ch < 6 → giữ nguyên coil")
        except Exception as e:
            self.get_logger().warn(f"[INIT-CYL2] exception: {e} → giữ nguyên coil")

    def _connect_servo(self, sid: int, ip: str, attempts: int = 5) -> bool:
        """
        Kết nối đến một servo Festo CMMT-AS qua Modbus TCP.
        Các bước sau khi kết nối:
          1. Đọc/ghi PNU 3490 → đặt telegram group = 111 (yêu cầu của edcon)
          2. Đọc base_velocity từ firmware FAS
          3. Đọc PNU 11352 (FAS JOG velocity v1) → lưu vào _fas_jog_vel[sid] và
             cập nhật _jog_velocity_ms nếu chưa được khởi tạo từ FAS
          4. Acknowledge faults để drive sẵn sàng nhận lệnh
        Trả về True nếu kết nối thành công, False nếu hết số lần thử.
        """
        for attempt in range(1, attempts + 1):
            try:
                t_ms = int(getattr(self.config, 'modbus_timeout_ms', 3000))
                com = ComModbus(ip_address=ip, cycle_time=60, timeout_ms=t_ms)
                tg = com.read_pnu(3490, 0)
                if tg != 111:
                    com.write_pnu(3490, 0, 111)
                mot = MotionHandler(com)
                try:
                    mot.base_velocity = com.read_pnu(12345, 0)
                    self.get_logger().info(f"S{sid} base_velocity={mot.base_velocity}")
                except Exception as e:
                    self.get_logger().warn(f"S{sid} base_velocity read failed: {e}")
                try:
                    fas_v1 = float(com.read_pnu(11352, 0))
                    self._fas_jog_vel[sid] = fas_v1
                    self.get_logger().info(f"S{sid} FAS jog v1={fas_v1:.3f} m/s")
                    with self._servo_lock:
                        if not self._jog_vel_from_fas and 0.001 <= fas_v1 <= 0.08:
                            self._jog_velocity_ms = fas_v1
                            self._jog_vel_from_fas = True
                except Exception as e:
                    self.get_logger().warn(f"S{sid} FAS jog v1 read failed: {e}")
                mot.acknowledge_faults()
                with self._servo_lock:
                    self.servos[sid] = mot
                self.get_logger().info(f"S{sid} ({ip}) OK (attempt {attempt})")
                return True
            except Exception as e:
                self.get_logger().warn(f"S{sid} ({ip}) fail attempt {attempt}/{attempts}: {e}")
                if attempt < attempts:
                    time.sleep(3.0)
        self.get_logger().error(f"S{sid} ({ip}) failed — se retry tu dong")
        return False

    def _servo_reconnect_loop(self):
        """
        Thread nền giám sát + auto reconnect — PARALLEL per-servo.
        Mỗi servo có thread con riêng, nên 1 servo offline (block ~3s Modbus
        timeout) KHÔNG block các servo khác phát hiện mất/khôi phục kết nối.
        Servo online → ping current_position() mỗi 5s; fail 2 lần → reconnect.
        Servo offline → retry _connect_servo(attempts=1) mỗi 5s.
        """
        # Spawn 1 thread per servo, then idle (each child loops forever).
        for sid, ip in self.config.servo_ips.items():
            threading.Thread(
                target=self._servo_monitor_one,
                args=(sid, ip),
                daemon=True,
                name=f"servo_mon_s{sid}",
            ).start()
        # Block here so the parent thread is alive (some shutdown paths
        # check rclpy.ok()); children handle the actual work.
        while rclpy.ok():
            time.sleep(60.0)

    def _servo_monitor_one(self, sid: int, ip: str):
        """Per-servo monitor: ping if connected, reconnect if dropped.
        Runs forever in its own thread so each servo's Modbus timeouts are
        isolated from the rest of the fleet."""
        consecutive_fail = 0
        while rclpy.ok():
            time.sleep(5.0)
            mot = self.servos.get(sid)
            if mot is None:
                # Offline → try to bring it back. attempts=1 because the outer
                # loop will retry every 5s anyway; no need for inner sleep.
                if self._connect_servo(sid, ip, attempts=1):
                    consecutive_fail = 0
                    self._notify('info', f'S{sid} da ket noi', f'S{sid} ({ip}) OK')
                continue
            try:
                with self._servo_lock:
                    mot.current_position()
                consecutive_fail = 0
            except Exception:
                consecutive_fail += 1
                if consecutive_fail >= 2:
                    self.get_logger().error(f"S{sid} mat ket noi -> reconnect")
                    self._notify('warn', f'Servo S{sid} mat ket noi', f'Dang reconnect...')
                    with self._servo_lock:
                        self.servos.pop(sid, None)
                    if self._connect_servo(sid, ip, attempts=1):
                        consecutive_fail = 0
                        self._notify('info', f'S{sid} ket noi lai', f'S{sid} OK')

    def destroy_node(self):
        """
        Dọn dẹp tài nguyên khi node bị shutdown (Ctrl+C hoặc ros2 node kill).
        Dừng thread đọc vị trí, gửi lệnh stop + shutdown đến từng servo,
        và giải phóng kết nối IO module. Được ROS2 executor gọi tự động.
        """
        self._pos_thread_stop = True
        for sid, mot in list(self.servos.items()):
            try:
                mot.stop_motion_task()
                mot.shutdown()
            except Exception as e:
                self.get_logger().warn(f"S{sid} shutdown: {e}")

        if getattr(self, 'io_module', None) is not None:
            try:
                self.io_module = None
                self.get_logger().info("IO module 1 closed")
            except Exception as e:
                self.get_logger().warn(f"IO 1 close error: {e}")
                
        if getattr(self, 'io_module_2', None) is not None:
            try:
                self.io_module_2 = None
                self.get_logger().info("IO module 2 closed")
            except Exception as e:
                self.get_logger().warn(f"IO 2 close error: {e}")

        super().destroy_node()

    # ── IO background reader ──────────────────────────────────────

    def _io_bg_loop(self):
        """
        Thread nền đọc trạng thái toàn bộ IO module với chu kỳ cấu hình
        `io_poll_period_s` (mặc định 20ms).
        Đọc tất cả channel của Module 1 và Module 2 vào cache (_io_sensor_cache,
        _io_sensor_cache_2) để vòng lặp điều khiển chính không bị block bởi
        latency Modbus/EtherNet/IP.
        Nếu lỗi liên tiếp >= 3 lần → đánh dấu module không sẵn sàng (_io_ready=False)
        và tự động spawn thread reconnect.
        """
        fail1, fail2 = 0, 0
        poll_period = max(0.01, float(getattr(self.config, 'io_poll_period_s', 0.02)))
        fast_s18_period = max(
            0.005,
            min(poll_period, float(getattr(self.config, 's18_fast_poll_period_s', 0.01))),
        )
        while rclpy.ok():
            # S18 nằm trên Module 2. Khi Servo3 đang đẩy khay, đọc Module 2
            # trước để thời gian đọc Module 1 không nằm trên critical path dừng
            # Servo3. Ngoài state này giữ nguyên thứ tự lịch sử để không ảnh
            # hưởng timing của cụm input.
            s18_critical = (
                getattr(self, '_s3_state', SystemState.IDLE) == SystemState.S3_SERVO3_FEED
                and bool(getattr(self, '_cmd_sent_s3', False))
            )
            module_order = (2, 1) if s18_critical else (1, 2)

            for module_idx in module_order:
                if module_idx == 1 and self.io_module is not None:
                    try:
                        channels = []
                        with self._io_bg_lock:
                            for mod in self.io_module.modules:
                                if mod.is_function_supported("read_channels"):
                                    ch = mod.read_channels()
                                    if isinstance(ch, list):
                                        channels.extend(ch)
                            self._io_sensor_cache = channels
                            self._update_sensor_latches_locked(channels, 1)
                            self._io_ready = True
                        fail1 = 0
                    except Exception as e:
                        fail1 += 1
                        if fail1 >= 3:
                            with self._io_bg_lock:
                                self._io_ready = False
                                self.io_module = None
                            self.get_logger().warn(f"[IO-bg 1] reconnecting: {e}")

                elif module_idx == 2 and self.io_module_2 is not None:
                    try:
                        with self._io_bg_lock:
                            channels2 = self._read_io2_sensor_channels_locked()
                            self._io_sensor_cache_2 = channels2
                            self._update_sensor_latches_locked(channels2, 17)
                            self._io_ready_2 = True
                        fail2 = 0
                    except Exception as e:
                        fail2 += 1
                        if fail2 >= 3:
                            with self._io_bg_lock:
                                self._io_ready_2 = False
                                self.io_module_2 = None
                            self.get_logger().warn(f"[IO-bg 2] reconnecting: {e}")

            time.sleep(fast_s18_period if s18_critical else poll_period)

    @staticmethod
    def _io2_di_modules_from_cpx(cpx) -> list:
        """Return a CPX41's 8DI modules in physical rack-position order."""
        if cpx is None:
            return []
        modules = []
        for mod in cpx.modules:
            name = str(getattr(mod, 'name', '')).lower()
            if mod.is_function_supported("read_channels") and "8di" in name:
                modules.append(mod)
        return sorted(modules, key=lambda mod: int(getattr(mod, 'position', 999)))

    def _io2_di_modules_locked(self) -> list:
        """Return CPX41 8DI modules in physical rack-position order."""
        return self._io2_di_modules_from_cpx(self.io_module_2)

    @staticmethod
    def _valve_module_from_cpx(cpx):
        """Select the VABX manifold explicitly; never guess between writable modules."""
        if cpx is None:
            return None
        writable = [
            mod for mod in cpx.modules
            if (mod.is_function_supported("set_channel")
                and mod.is_function_supported("reset_channel"))
        ]
        named = [m for m in writable if "vabx" in str(getattr(m, 'name', '')).lower()]
        if named:
            return sorted(named, key=lambda mod: int(getattr(mod, 'position', 999)))[0]
        return writable[0] if len(writable) == 1 else None

    def _io1_valve_module_locked(self):
        """Return only the CPX37 VABX valve manifold, never a DI module."""
        return self._valve_module_from_cpx(self.io_module)

    def _io2_valve_module_locked(self):
        """Return only the CPX41 VABX valve manifold, never a DI module."""
        if self.io_module_2 is None:
            return None
        return self._valve_module_from_cpx(self.io_module_2)

    def _read_io2_sensor_channels_locked(self) -> list:
        """Read CPX 254 DI modules only: S17-S24 from first 8DI, S25-S28 from second 8DI."""
        channels: list = []
        di_modules = self._io2_di_modules_locked()
        if len(di_modules) < 2:
            raise RuntimeError(
                f"CPX41 mapping invalid: expected 2x 8DI, found {len(di_modules)}"
            )
        for mod in di_modules[:2]:
            ch = mod.read_channels()
            if isinstance(ch, list):
                channels.extend(ch[:8])
        return channels[:12]

    def _update_sensor_latches_locked(self, channels: list, first_sid: int):
        """Giữ ON ngắn cho sensor để không hụt xung nhanh giữa hai lần polling."""
        latch_s = getattr(self, '_sensor_on_latch_s', 0.0)
        if latch_s <= 0.0:
            return
        until = time.time() + latch_s
        for idx, value in enumerate(channels):
            if value:
                self._sensor_latch_until[first_sid + idx] = until

    def _sensor_latched_locked(self, sid: int, raw: bool, now: Optional[float] = None) -> bool:
        if raw:
            return True
        if sid in getattr(self, '_sensor_no_latch_sids', set()):
            return False
        latch_s = getattr(self, '_sensor_on_latch_s', 0.0)
        if latch_s <= 0.0:
            return False
        if now is None:
            now = time.time()
        return now <= self._sensor_latch_until.get(sid, 0.0)

    # ── Sensor read ───────────────────────────────────────────────

    def _sensor_raw(self, sid: int) -> bool:
        """Luôn đọc sensor phần cứng thực (IO module).
           - S17..S28 → Module 2 (CH sid-17; S23/S24 reserved)
           - S1..S16 → Module 1 (CH sid-1)
        """
        with self._io_bg_lock:
            cache1 = getattr(self, '_io_sensor_cache', [])
            ready1 = getattr(self, '_io_ready', False)
            cache2 = getattr(self, '_io_sensor_cache_2', [])
            ready2 = getattr(self, '_io_ready_2', False)

        if 1 <= sid <= 16:
            if not ready1 or len(cache1) < sid:
                with self._io_bg_lock:
                    return self._sensor_latched_locked(sid, False)
            raw = bool(cache1[sid - 1])
            with self._io_bg_lock:
                return self._sensor_latched_locked(sid, raw)
        elif 17 <= sid <= 28:
            if not ready2 or len(cache2) < (sid - 16):
                with self._io_bg_lock:
                    return self._sensor_latched_locked(sid, False)
            raw = bool(cache2[sid - 17])
            with self._io_bg_lock:
                return self._sensor_latched_locked(sid, raw)
        return False

    def sensor(self, sid: int) -> bool:
        """Đọc sensor thực từ IO module — áp dụng cho cả AUTO/AI/MANUAL.
        Trước đây MANUAL đọc từ _sim_sensors dict (deprecated kể từ feature/manual-real-sensors)."""
        return self._sensor_raw(sid)

    def sensor_real(self, sid: int) -> bool:
        """Alias của sensor() — giữ để backward-compat call site cũ (homing, hardware checks)."""
        return self._sensor_raw(sid)

    def _snap(self, *sids: int) -> tuple:
        """Snapshot nhiều sensor cùng lúc từ IO module (1 lần acquire lock)."""
        with self._io_bg_lock:
            cache1 = getattr(self, '_io_sensor_cache', [])
            ready1 = getattr(self, '_io_ready', False)
            cache2 = getattr(self, '_io_sensor_cache_2', [])
            ready2 = getattr(self, '_io_ready_2', False)

        results = []
        now = time.time()
        for sid in sids:
            try:
                if 1 <= sid <= 16:
                    raw = bool(cache1[sid - 1]) if ready1 and len(cache1) >= sid else False
                elif 17 <= sid <= 28:
                    raw = bool(cache2[sid - 17]) if ready2 and len(cache2) >= (sid - 16) else False
                else:
                    raw = False
            except Exception:
                raw = False
            with self._io_bg_lock:
                results.append(self._sensor_latched_locked(sid, raw, now))
        return tuple(results)

    # ── Non-blocking servo motion ─────────────────────────────────

    def _nb_move(self, servo_id: int, pos_mm: float, vel: int = 30, continuous_update: bool = False) -> bool:
        """
        Gửi lệnh di chuyển tuyệt đối non-blocking đến servo (không chờ đến đích).
        Kiểm tra giới hạn phần mềm (servo_limits) và yêu cầu đã homed trước khi gửi.
        pos_mm: vị trí đích tính từ điểm zero (sau homing), đơn vị mm.
        vel: tốc độ mm/s (mặc định 30).
        Sau khi gửi lệnh: đặt _ignore_arrived_X trong 0.5s để tránh
        target_position_reached() trả True sai ngay khi bắt đầu di chuyển.
        Trả về False nếu vượt giới hạn hoặc chưa homed, True nếu thành công.
        """
        limit = self._conf('servo_limits', {}).get(servo_id, 999.0)
        if pos_mm > limit:
            self.get_logger().error(f"S{servo_id}: {pos_mm}mm > limit {limit}mm")
            return False
        if servo_id not in self.zero_offset and servo_id in self.servos:
            self.get_logger().error(f"S{servo_id}: chua home!")
            return False
        mot = self.servos.get(servo_id)
        if not mot:
            return True
        try:
            offset = int(self.zero_offset.get(servo_id, 0))
            counts = offset + int(pos_mm * COUNTS_PER_MM)
            self._ensure_ready(mot, servo_id)
            # FAS firmware quirk: sau homing, position_task đầu tiên bị drive
            # "nuốt" (accept nhưng không move vật lý). Flush bằng stop_motion_task
            # rồi settle 0.5s trước khi gửi lệnh thật.
            if self._need_warm_up.pop(servo_id, False):
                with self._servo_lock:
                    mot.stop_motion_task()
                time.sleep(0.5)
                self.get_logger().info(f"S{servo_id}: post-homing warm-up flush done")
            with self._servo_lock:
                if continuous_update and hasattr(mot, 'configure_continuous_update'):
                    mot.configure_continuous_update(True)
                mot.position_task(int(counts), int(vel), absolute=True, nonblocking=True)
                if continuous_update and hasattr(mot, 'configure_continuous_update'):
                    mot.configure_continuous_update(False)
            if servo_id == 1: self._inx_moving = True
            elif servo_id == 2: self._iny_moving = True
            now = time.time()
            self._active_servo_moves[servo_id] = (
                float(pos_mm), int(vel), bool(continuous_update))
            setattr(self, f'_ignore_arrived_{servo_id}', now + 0.5)
            self._servo_motion_t[servo_id] = now  # cho _publish_positions đọc tươi
            return True
        except Exception as e:
            self.get_logger().error(f"S{servo_id} nb_move error: {e}")
            return False

    def _arrived(self, servo_id: int) -> bool:
        """
        Kiểm tra servo đã đến đích chưa (target_position_reached từ FAS firmware).
        Trả về False trong 0.5s đầu sau khi gửi lệnh (_ignore_arrived_X)
        để tránh false-positive khi drive chưa kịp bắt đầu di chuyển.
        Nếu servo không kết nối → trả về True (không block flow).
        """
        if time.time() < getattr(self, f'_ignore_arrived_{servo_id}', 0.0):
            return False
        mot = self.servos.get(servo_id)
        if not mot:
            return True
        try:
            with self._servo_lock:
                done = mot.target_position_reached()
            if done:
                active = self._active_servo_moves.get(servo_id)
                physically_at_target = (
                    active is None or self._at_position(servo_id, active[0]))
                if physically_at_target:
                    self._active_servo_moves.pop(servo_id, None)
                    if servo_id == 1: self._inx_moving = False
                    if servo_id == 2: self._iny_moving = False
            return done
        except Exception:
            return False

    def _at_position(self, servo_id: int, target_mm: float, tol: Optional[float] = None) -> bool:
        """
        Verify servo ĐÃ THỰC SỰ tới target bằng cách đọc vị trí encoder (_pos),
        không tin tuyệt đối flag target_position_reached() của drive (có thể lên sớm
        khi servo còn cách target vài mm do "position window" của FAS firmware).

        Args:
            servo_id  : 1=InX, 2=InY, 3=Servo3, 4=OutX, 5=OutY
            target_mm : vị trí target tính bằng mm
            tol       : sai số cho phép (mm). Nếu None → lấy config.position_tolerance.

        Returns:
            True  nếu |_pos() - target_mm| ≤ tol
            False nếu chưa đến hoặc đọc vị trí lỗi
            True  (skip) nếu servo chưa kết nối (cho phép flow tiếp tục trong sim mode)

        Dùng kết hợp với _arrived() để xác nhận servo đến đích THẬT trước khi
        chuyển sub-state nguy hiểm (vd: cylinder extend, InY scan).
        """
        if servo_id not in self.servos:
            return True
        if tol is None:
            tol = getattr(self.config, 'position_tolerance', 1.0)
        pos = self._pos(servo_id)
        if pos is None:
            return False
        return abs(pos - target_mm) <= tol

    def _stop(self, servo_id: int):
        """
        Dừng ngay servo (stop_motion_task) và reset flag đang di chuyển.
        Dùng khi nhấn STOP, ABORT, hoặc timeout bước.
        KHÔNG clear ready cache — stop_motion_task chỉ halt current task,
        drive vẫn enabled+ready, lần JOG kế tiếp phải mượt không re-check.
        """
        mot = self.servos.get(servo_id)
        if mot:
            try:
                with self._servo_lock:
                    mot.stop_motion_task()
            except Exception:
                pass
        self._jog_active.discard(servo_id)
        self._active_servo_moves.pop(servo_id, None)
        if servo_id == 1: self._inx_moving = False
        if servo_id == 2: self._iny_moving = False

    def _stop_immediate(self, servo_id: int):
        """
        Phát STOP tới một servo mà KHÔNG chờ drive báo ``drive_stopped``.

        ``edcon.stop_motion_task()`` phát đúng các bit STOP nhưng sau đó gọi
        ``wait_for_stop()`` không timeout. Nếu dùng hàm đó tuần tự trong callback
        STOP, một drive đang HOME/offline có thể giữ callback vô hạn và các drive
        còn lại sẽ không bao giờ nhận STOP.

        Vì đây là đường safety, ta chỉ thực hiện phần phát telegram STOP của
        edcon rồi trả về ngay. Cố ý không lấy ``_servo_lock``: HOME có thể đang
        giữ lock trong một lệnh Modbus; STOP phải ghi đè telegram đang chạy ngay.
        Mỗi servo có IO thread/telegram riêng nên các trục được broadcast độc lập.
        """
        mot = self.servos.get(servo_id)
        if mot:
            try:
                prepare_stop = getattr(mot, '_prepare_stop_motion_task_bits', None)
                telegram = getattr(mot, 'telegram', None)
                com = getattr(mot, 'com', None)
                send_io = getattr(com, 'send_io', None)
                if not callable(prepare_stop) or telegram is None or not callable(send_io):
                    raise AttributeError("edcon immediate STOP API unavailable")

                # Mirrors the non-blocking half of edcon.stop_motion_task():
                # clear position/JOG/HOME bits and assert the STOP bit on wire.
                prepare_stop()
                stop_bytes = telegram.output_bytes()
                send_io(stop_bytes, nonblocking=True)

                # Match edcon's local post-send state. This value is intentionally
                # not sent now; the just-queued telegram keeps STOP asserted.
                telegram.stw1.do_not_reject_traversing_task = True
            except Exception as exc:
                # Compatibility fallback for an older/different edcon version.
                # It may wait forever, therefore isolate it from the ROS callback
                # so STOP for the other axes and system reset still continue.
                self.get_logger().warn(
                    f"S{servo_id}: immediate STOP fallback ({exc})"
                )

                def _legacy_stop():
                    try:
                        mot.stop_motion_task()
                    except Exception as legacy_exc:
                        self.get_logger().warn(
                            f"S{servo_id}: STOP command failed ({legacy_exc})"
                        )

                threading.Thread(
                    target=_legacy_stop,
                    daemon=True,
                    name=f"servo_stop_s{servo_id}",
                ).start()

        self._jog_active.discard(servo_id)
        self._active_servo_moves.pop(servo_id, None)
        if servo_id == 1: self._inx_moving = False
        if servo_id == 2: self._iny_moving = False

    def _pos_cached(self, servo_id: int) -> Optional[float]:
        """
        Đọc vị trí cached (mm) — KHÔNG gọi Modbus, KHÔNG cần _servo_lock.
        Dùng cho hot path control_loop (20Hz) để tránh chiếm lock liên tục,
        nhường lock cho JOG/STOP/state machine motion commands.

        Cache update mỗi ~100ms bởi _positions_bg_loop._publish_positions().
        Trả None nếu cache chưa populate (lần đầu sau startup, trước khi
        _positions_bg_loop tick đầu tiên hoàn tất).

        ⚠️ KHÔNG dùng cho _at_position() / _arrived() — các check đó cần
        giá trị tươi để verify servo settle đúng vị trí trước sub-state nguy hiểm.
        """
        return self._pos_cache.get(servo_id)

    def _pos(self, servo_id: int) -> Optional[float]:
        """
        Đọc vị trí hiện tại của servo (mm, tính từ zero_offset sau homing).
        Công thức: (encoder_counts - zero_offset) / COUNTS_PER_MM
        Trả về 0.0 nếu servo không kết nối, None nếu có lỗi đọc.
        Đồng thời update _pos_cache để các caller cached có dữ liệu tươi hơn
        ngay sau khi _pos() thực sự được gọi.
        """
        mot = self.servos.get(servo_id)
        if not mot:
            return 0.0
        try:
            with self._servo_lock:
                counts = mot.current_position()
            mm = (counts - self.zero_offset.get(servo_id, 0)) / COUNTS_PER_MM
            self._pos_cache[servo_id] = round(mm, 2)
            return mm
        except Exception as e:
            self.get_logger().warn(f"S{servo_id} _pos() error: {e}")
            return None

    def _jog(self, servo_id: int, forward: bool):
        """
        Phát lệnh JOG liên tục đến servo (duration=0.0 = giữ cho đến khi có lệnh stop).
        forward=True → chiều dương (xa home), forward=False → chiều âm (về home).
        Chỉ dùng trong MANUAL mode khi operator điều khiển từ GUI.
        """
        mot = self.servos.get(servo_id)
        if not mot:
            return

        # Đánh dấu motion timestamp để _publish_positions biết đọc tươi
        self._servo_motion_t[servo_id] = time.time()
        # Đánh dấu JOG active → publish loop bỏ qua idle-skip, đọc tươi liên tục
        self._jog_active.add(servo_id)
        try:
            self._ensure_ready(mot, servo_id)
            with self._servo_lock:
                mot.jog_task(forward, not forward, duration=0.0)
        except Exception as e:
            self._jog_active.discard(servo_id)
            self.get_logger().error(f"S{servo_id} jog error: {e}")

    def _ensure_ready(self, mot, servo_id: int, timeout: float = 3.0):
        """
        Đảm bảo drive sẵn sàng nhận lệnh chuyển động:
          1. acknowledge_faults() — xóa lỗi cũ trên drive (giống bấm ACK trong FAS)
          2. enable_powerstage() — bật nguồn stage
          3. Chờ ready_for_motion() = True trong timeout giây
        Lock được giữ ngắn nhất có thể để STOP có thể acquire _servo_lock ngay.

        Fast path: cache ready_until[sid] — nếu confirm ready trong vòng _ready_ttl
        giây trước, skip toàn bộ Modbus query (giảm 500ms→<50ms latency cho JOG liên tiếp).
        """
        now = time.time()
        if now < self._ready_until.get(servo_id, 0.0):
            return True

        if hasattr(mot, 'ready_for_motion'):
            acquired = self._servo_lock.acquire(timeout=0.1)
            if acquired:
                try:
                    if mot.ready_for_motion():
                        self._ready_until[servo_id] = now + self._ready_ttl
                        return True
                except Exception:
                    pass
                finally:
                    self._servo_lock.release()

        # Short lock holds — tránh race condition trên Modbus socket
        acquired = self._servo_lock.acquire(timeout=1.0)
        if acquired:
            try:
                mot.acknowledge_faults()
                mot.enable_powerstage()
            finally:
                self._servo_lock.release()

        if not hasattr(mot, 'ready_for_motion'):
            return True
        start = time.time()
        while time.time() - start < timeout:
            if self._servo_lock.acquire(timeout=0.5):
                try:
                    ready = mot.ready_for_motion()
                finally:
                    self._servo_lock.release()
                if ready:
                    self._ready_until[servo_id] = time.time() + self._ready_ttl
                    return True
            time.sleep(0.05)
        self.get_logger().warn(f"S{servo_id}: drive not ready after {timeout}s")
        return False

    # ── Cylinders ─────────────────────────────────────────────────

    def _set_do(self, channel: int, state: bool) -> bool:
        """
        Ghi giá trị digital output lên IO Module 1 (set/reset một channel).
        Chọn đích danh manifold VABX, không ghi nhầm module writable khác.
        Trả về False nếu IO module không kết nối.
        """
        if not self.io_module: return False
        with self._io_bg_lock:
            mod = self._io1_valve_module_locked()
            if mod is None:
                self.get_logger().error(
                    "CPX37 valve mapping invalid: VABX writable module not found/ambiguous"
                )
                return False
            try:
                if state: mod.set_channel(channel)
                else: mod.reset_channel(channel)
                return True
            except Exception as e:
                self.get_logger().warn(f"CPX37 valve write ch{channel} failed: {e}")
        return False

    def _set_do_2(self, channel: int, state: bool) -> bool:
        """
        Ghi digital output lên IO Module 2 (CPX 172.16.11.41).
        Mapping van: Cyl2=0/1, Cyl4=2/3, Cyl5=4/5 (EXTEND/RETRACT).
        """
        if not self.io_module_2:
            return False
        with self._io_bg_lock:
            mod = self._io2_valve_module_locked()
            if mod is None:
                self.get_logger().error(
                    "CPX41 valve mapping invalid: VABX writable module not found/ambiguous"
                )
                return False
            try:
                if state:
                    mod.set_channel(channel)
                else:
                    mod.reset_channel(channel)
                return True
            except Exception as e:
                self.get_logger().warn(f"CPX41 valve write ch{channel} failed: {e}")
        return False

    def _cyl1_extend(self) -> bool:
        """
        Đẩy xi lanh khí nén 1 ra (EXTEND): reset kênh retract → set kênh extend.
        Cylinder 1 dùng để giữ/đẩy khay tray trong STATE 1/2.
        Sensor feedback: S9=retracted, S10=extended.
        Channel mặc định: extend=4, retract=5 (cấu hình trong config YAML).
        """
        self.get_logger().info(f"Cyl1 EXTEND (ch{self._conf('cylinder1_extend_channel', 4)})")
        io = self.io_module  # snapshot to avoid TOCTOU race with _io_bg_loop
        if io:
            try:
                self._set_do(self._conf('cylinder1_retract_channel', 5), False)
                self._set_do(self._conf('cylinder1_extend_channel', 4), True)
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl1 extend IO error: {e}")
                return False
        return False

    def _cyl1_retract(self) -> bool:
        """
        Thu xi lanh 1 về (RETRACT): reset kênh extend → set kênh retract.
        Gọi khi kết thúc STATE 1/2, STOP khẩn cấp, hoặc khởi động node.
        Sensor feedback: S9=retracted, S10=extended.
        """
        self.get_logger().info(f"Cyl1 RETRACT (ch{self._conf('cylinder1_retract_channel', 5)})")
        io = self.io_module  # snapshot to avoid TOCTOU race with _io_bg_loop
        if io:
            try:
                self._set_do(self._conf('cylinder1_extend_channel', 4), False)
                self._set_do(self._conf('cylinder1_retract_channel', 5), True)
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl1 retract IO error: {e}")
                return False
        return False

    def _cyl2_extend(self) -> bool:
        """
        Đẩy xi lanh 2 ra (EXTEND) — dùng trong STATE 4 để giữ khay output.
        Cylinder 2 đã chuyển sang IO Module 2 (CPX 172.16.11.41).
        Sensor feedback: S21=retracted, S22=extended.
        Channel mặc định: extend=0, retract=1.
        """
        io = self.io_module_2
        if self._io_ready_2 and io:
            try:
                self._set_do_2(self._conf('cylinder2_retract_channel', 1), False)
                self._set_do_2(self._conf('cylinder2_extend_channel', 0), True)
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl2 extend error: {e}")
                return False
        return False

    def _cyl2_retract(self) -> bool:
        """
        Thu xi lanh 2 về trên IO Module 2: reset extend=0, set retract=1.
        Sensor feedback: S21=retracted, S22=extended.
        """
        io = self.io_module_2
        if self._io_ready_2 and io:
            try:
                self._set_do_2(self._conf('cylinder2_extend_channel', 0), False)
                self._set_do_2(self._conf('cylinder2_retract_channel', 1), True)
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl2 retract error: {e}")
                return False
        return False

    def _cyl3_set_expected(self, target: str):
        """Set expected feedback state cho cyl3 monitor (target = 'extended' | 'retracted')."""
        if self._cyl3_expected != target:
            self._cyl3_expected         = target
            self._cyl3_cmd_time         = time.time()
            self._cyl3_mismatch_warned  = False
            self._cyl3_confirmed_logged = False

    def _cyl3_extend(self) -> bool:
        """
        Đẩy xi lanh 3 ra (EXTEND) — cố định khay tại Tray Pos1 sau khi Cyl1 đã thả.
        Channel mặc định: extend=6, retract=7 (Module 4 valve CPX 253).
        Sensor feedback: S15=retracted, S16=extended.

        Bị CHẶN khi _cyl3_s13s14_latch còn set (S13+S14 cùng OFF → không có khay
        ở cả 2 vị trí output). Caller muốn extend hợp lệ phải clear latch trước:
        manual GUI extend hoặc STATE 2 recheck S6 ON.
        """
        if self._cyl3_s13s14_latch:
            self._log_once(
                "CYL3_EXTEND_BLOCKED_LATCH",
                "[CYL3] EXTEND bị chặn — S13+S14 latch đang giữ RETRACT",
            )
            return False
        io = self.io_module
        if io:
            try:
                self._set_do(self._conf('cylinder3_retract_channel', 7), False)
                self._set_do(self._conf('cylinder3_extend_channel', 6), True)
                self._cyl3_set_expected("extended")
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl3 extend IO error: {e}")
                return False
        return False

    def _cyl3_retract(self) -> bool:
        """
        Thu xi lanh 3 về (RETRACT). Gọi bởi safety watchdog khi S13+S14 cùng OFF.
        Sensor feedback: S15=retracted, S16=extended.
        """
        io = self.io_module
        if io:
            try:
                self._set_do(self._conf('cylinder3_extend_channel', 6), False)
                self._set_do(self._conf('cylinder3_retract_channel', 7), True)
                self._cyl3_set_expected("retracted")
                return True
            except Exception as e:
                self.get_logger().warn(f"Cyl3 retract IO error: {e}")
                return False
        return False

    def _cyl4_extend(self) -> bool:
        """
        Đẩy Cylinder 4 ra để giữ khay trên platform Servo 3.
        Output nằm trên IO Module 2: EXTEND=2, RETRACT=3.
        Sensor feedback: S25=retracted, S26=extended.
        """
        if not self._io_ready_2 or not self.io_module_2:
            return False
        try:
            self._set_do_2(self._conf('cylinder4_retract_channel', 3), False)
            self._set_do_2(self._conf('cylinder4_extend_channel', 2), True)
            return True
        except Exception as e:
            self.get_logger().warn(f"Cyl4 extend IO error: {e}")
            return False

    def _cyl4_retract(self) -> bool:
        """
        Thu Cylinder 4 khi platform không có khay (S17 OFF).
        Output nằm trên IO Module 2: EXTEND=2, RETRACT=3.
        Sensor feedback: S25=retracted, S26=extended.
        """
        if not self._io_ready_2 or not self.io_module_2:
            return False
        try:
            self._set_do_2(self._conf('cylinder4_extend_channel', 2), False)
            self._set_do_2(self._conf('cylinder4_retract_channel', 3), True)
            return True
        except Exception as e:
            self.get_logger().warn(f"Cyl4 retract IO error: {e}")
            return False

    def _cyl5_extend(self) -> bool:
        """
        Đẩy Cylinder 5 ra khi S19 phát hiện khay tại output stack.
        Output IO Module 2: EXTEND=4, RETRACT=5.
        Sensor feedback: S27=retracted, S28=extended.
        """
        if not self._io_ready_2 or not self.io_module_2:
            return False
        try:
            self._set_do_2(self._conf('cylinder5_retract_channel', 5), False)
            self._set_do_2(self._conf('cylinder5_extend_channel', 4), True)
            return True
        except Exception as e:
            self.get_logger().warn(f"Cyl5 extend IO error: {e}")
            return False

    def _cyl5_retract(self) -> bool:
        """
        Thu Cylinder 5 khi S19 OFF.
        Output IO Module 2: EXTEND=4, RETRACT=5.
        Sensor feedback: S27=retracted, S28=extended.
        """
        if not self._io_ready_2 or not self.io_module_2:
            return False
        try:
            self._set_do_2(self._conf('cylinder5_extend_channel', 4), False)
            self._set_do_2(self._conf('cylinder5_retract_channel', 5), True)
            return True
        except Exception as e:
            self.get_logger().warn(f"Cyl5 retract IO error: {e}")
            return False

    # ── Helpers ──────────────────────────────────────────────────

    def _enter(self, next_state: SystemState):
        """
        Chuyển trạng thái GLOBAL (self.state) và log chuyển tiếp.
        Xóa _guide_logged khi đổi state để các log "chỉ in 1 lần" được reset.
        Khi vào ERROR: clear zero_offset → buộc re-home sau khi recovery
        (auto/ai tự enter HOMING ở START; manual chờ operator nhấn HOMING).
        """
        self.get_logger().info(f"[GLOBAL] -> {next_state.name}")
        if next_state != self.state:
            self._guide_logged.clear()
        if next_state == SystemState.ERROR and self.zero_offset:
            self.zero_offset.clear()
            self._homing_ref_offset.clear()
            self._publish_homing_done(False)  # gate vfd_logic AUTO chờ re-home
            self.get_logger().warn("[ERROR] Cleared zero_offset — cần re-home sau khi recovery")
        self.state = next_state

    def _enter_in(self, next_state: SystemState):
        """
        Chuyển trạng thái state machine phụ INPUT (state_in).
        Reset _cmd_sent_in và các biến step-timer để bước tiếp theo bắt đầu sạch.
        Dùng cho toàn bộ luồng STATE 1 và STATE 2.
        """
        self.get_logger().info(f"[IN] -> {next_state.name}")
        self.state_in        = next_state
        self._cmd_sent_in    = False
        self._step_start_in  = 0.0
        self._step_timeout_in = 0.0

    def _enter_s3(self, next_state: SystemState):
        """
        Chuyển trạng thái state machine phụ S3 (state_s3).
        Reset _cmd_sent_s3 và step-timer cho bước tiếp theo.
        Dùng cho toàn bộ luồng STATE 3 (Platform/Servo3 cấp khay vào vị trí feed).
        """
        self.get_logger().info(f"[S3] -> {next_state.name}")
        self.state_s3        = next_state
        self._cmd_sent_s3    = False
        self._step_start_s3  = 0.0
        self._step_timeout_s3 = 0.0

    def _enter_s4(self, next_state: SystemState):
        """
        Chuyển trạng thái state machine phụ S4 (state_s4).
        Reset _cmd_sent_s4 và step-timer cho bước tiếp theo.
        Dùng cho toàn bộ luồng STATE 4 (OutX/OutY thay khay output).
        """
        self.get_logger().info(f"[S4] -> {next_state.name}")
        self.state_s4        = next_state
        self._cmd_sent_s4    = False
        self._step_start_s4  = 0.0
        self._step_timeout_s4 = 0.0

    def _servo3_at_last_tray_position(self) -> bool:
        """True khi Servo3 đang ở vị trí khay cuối của stack output."""
        return self._at_position(3, self.config.servo3_target2, tol=5.0)

    def _s4_hold_cyl4_if_s17_on(self) -> bool:
        """Trong STATE 4, S17 ON nghĩa là Cyl4 phải giữ platform cho tới lúc lấy khay cuối."""
        if not self.sensor(S17_PLATFORM):
            return True
        cyl4_ret, cyl4_ext = self._snap(S25_CYL4_RETRACTED, S26_CYL4_EXTENDED)
        if cyl4_ext and not cyl4_ret:
            return True
        now = time.time()
        if not self._cyl4_retract_sent or now - self._cyl4_retry_t >= 3.0:
            self._cyl4_extend()
            self._cyl4_retract_sent = True
            self._cyl4_retry_t = now
        self._log_once(
            "S4_HOLD_CYL4_S17",
            f"S4 giữ Cyl4 EXTEND khi S17 ON: S25={int(cyl4_ret)} S26={int(cyl4_ext)}"
        )
        return False

    def _error(self, msg: str):
        """
        Chuyển hệ thống sang trạng thái ERROR, log lỗi và gửi thông báo GUI.
        Toàn bộ state machine dừng lại cho đến khi operator nhấn STOP → START.
        """
        self.get_logger().error(f"ERROR: {msg}")
        self._notify('error', 'ERROR', msg)
        self._enter(SystemState.ERROR)

    def _conf(self, key: str, default: Any = 0.0) -> Any:
        """
        Đọc một giá trị config an toàn, trả về default nếu key không tồn tại.
        Tránh AttributeError khi config chưa có key mới (backward compatibility).
        """
        try:
            return getattr(self.config, key, default)
        except Exception:
            return default

    def _notify(self, level: str, title: str, detail: str = "", hint: str = "",
                code: str = ""):
        """
        Gửi thông báo đến GUI qua topic /providesystem/gui_notify (JSON).
        level: 'info' | 'warn' | 'error' | 'silent_ok'
        hint : optional UI hint cho GUI animation (vd 'press_homing', 'press_stop',
               'switch_manual'). GUI subscribe và blink button tương ứng.
        code : optional machine-readable event code cho popup/action chuyên biệt.
        Throttle 0.5s/title để không flood GUI với cùng một thông báo lặp lại.
        """
        now = time.time()
        if now - self._notify_throttle.get(title, 0) < 0.5:
            return
        self._notify_throttle[title] = now
        try:
            msg = String()
            payload = {"level": level, "title": title, "detail": detail}
            if hint:
                payload["hint"] = hint
            if code:
                payload["code"] = code
            msg.data = json.dumps(payload)
            self.pub_gui_notify.publish(msg)
        except Exception:
            pass

    # ── Activity log helpers ──────────────────────────────────────

    def _sensor_label(self, sid: int) -> str:
        """Trả về 'S6 (Check Tray OutP1)' từ config sensors[].gui_label.
        Cache lazy ở lần gọi đầu. Fallback 'S<sid>' nếu không có trong config.
        """
        if not hasattr(self, '_sensor_label_cache') or self._sensor_label_cache is None:
            self._sensor_label_cache = {}
            for s in (getattr(self.config, 'sensors', []) or []):
                lbl = getattr(s, 'gui_label', None) or getattr(s, 'name', None)
                if lbl:
                    self._sensor_label_cache[s.id] = lbl
        lbl = self._sensor_label_cache.get(sid)
        return f"S{sid} ({lbl})" if lbl else f"S{sid}"

    def _notify_step(self, level: str, state: str, step: str, issue: str,
                     check=None, action=None, enum_name: str = "", hint: str = ""):
        """Notify dạng structured cho state machine — giúp operator biết chính xác
        đang ở đâu + cần kiểm tra/làm gì.

        Args:
          level     : 'info' | 'warn' | 'error'
          state     : tên state operator-facing, vd 'STATE 2A', 'HOMING', 'STATE 1'
          step      : tên bước/sub-step, vd 'A2 InX→505.5', 'pre-flight', '' nếu N/A
          issue     : mô tả ngắn vấn đề
          check     : list[str] những thứ cần kiểm tra (sensor, valve, drive...)
          action    : list[str] các bước operator cần làm (nhấn STOP, retract manual...)
          enum_name : optional SystemState.name để dev/log archaeologist trace
          hint      : optional UI hint cho GUI animation (vd 'press_homing',
                      'press_stop', 'switch_manual') — GUI sẽ blink button tương ứng

        Format:
          title  = '<state> • <step>'      (hoặc chỉ '<state>' nếu step rỗng)
          detail = '<issue>'
                 + ' | Step: <enum_name>'  (nếu có)
                 + ' | Kiểm tra: a, b, c'  (nếu có)
                 + ' | Tiếp: x → y'        (nếu có)
        """
        title = f"{state} • {step}" if step else state
        parts = [issue]
        if enum_name:
            parts.append(f"Step: {enum_name}")
        if check:
            parts.append("Kiểm tra: " + ", ".join(check))
        if action:
            parts.append("Tiếp: " + " → ".join(action))
        self._notify(level, title, " | ".join(parts), hint=hint)

    def _log_once(self, key: str, msg: str):
        """
        Log một message chỉ một lần trong mỗi trạng thái (dùng key làm ID).
        _guide_logged bị xóa mỗi khi state thay đổi, nên message sẽ xuất hiện
        lại ở trạng thái mới. Tránh spam log khi hệ thống đang chờ điều kiện.
        """
        if key not in self._guide_logged:
            self._guide_logged.add(key)
            self.get_logger().info(f"[guide] {msg}")

    def _find_nearest_row_abs(self, pos_mm: float, row_dict: dict) -> int:
        """
        Tìm row gần nhất với vị trí pos_mm trong bảng row_dict {row: position_mm}.
        Dùng khi InY scan để quyết định row nào của stack tray đang ở vị trí đó.
        """
        return min(row_dict, key=lambda r: abs(row_dict[r] - pos_mm))

    def _zone_to_row(self, trigger_pos: float, zone_table: dict) -> Optional[int]:
        """
        Ánh xạ vị trí trigger_pos (mm) vào row number theo bảng zone_table.
        zone_table: {row: [biên_1, biên_2, target_mm]} — zone = [min, max] của
        2 biên đầu. Dùng khi S4/S20 bật (cảm biến scan stack) để xác định khay
        đang ở row nào.

        Trả về None nếu không khớp zone nào (trigger rơi khe hở giữa 2 zone
        hoặc nhiễu). Caller PHẢI xử lý None (retry scan / skip edge) — TUYỆT
        ĐỐI không mặc định row. Sự cố 2026-07-03: fallback row1 cũ làm InY
        chạy thẳng lên R1 khi trigger 804.9mm rơi khe 790–805 giữa zone
        row3/row2 → Cyl1 extend sai vị trí, phải STOP khẩn.
        """
        for row, vals in zone_table.items():
            if len(vals) >= 2 and min(vals[0], vals[1]) <= trigger_pos <= max(vals[0], vals[1]):
                return row
        self.get_logger().warn(
            f"[zone_to_row] {trigger_pos:.1f}mm không khớp zone nào → None (caller xử lý retry/skip)"
        )
        return None

    # def _calc_output_target(self, trigger_pos_mm: float) -> tuple:
    #     cfg             = self.config
    #     actual_tray_pos = trigger_pos_mm + cfg.s4_cross_offset_mm
    #     target_pos      = actual_tray_pos - cfg.tray_height_mm
    #     occupied_row    = self._find_nearest_row_abs(actual_tray_pos, cfg.iny_output_stack)
    #     is_full         = target_pos <= cfg.output_min_pos_mm
    #     return target_pos, occupied_row, is_full

    def _iny_safe(self) -> bool:
        """
        Kiểm tra InY đang ở vùng an toàn (≤ iny_safe_zone mm).
        Interlock: InX chỉ được phép di chuyển khi InY < 50mm (safe zone)
        để tránh va chạm giữa hai trục.
        """
        p = self._pos(2)
        return p is not None and p <= self.config.iny_safe_zone

    # ══════════════════════════════════════════════════════════════
    # INVARIANT — S1/S2 MUTEX via S7 (Khay tại Robot) — ÁP DỤNG MỌI MODE
    #   S7 OFF (Robot trống)  → CHỈ S1 trigger (lấy khay mới vào Robot)
    #   S7 ON  (Robot có khay) → CHỈ S2 trigger (lấy khay cũ ra)
    # Enforced ở 4 chỗ: _can_start_s1, _can_start_s2a (auto/ai)
    #                   + _cb_goto_state STATE1/STATE2 (manual button).
    # Đừng nới lỏng — vi phạm = 2 khay cùng vị trí Robot → va chạm cơ khí.
    # ══════════════════════════════════════════════════════════════
    def _can_start_s1(self) -> bool:
        """
        Điều kiện để auto-trigger STATE 1 (chỉ AUTO/AI mode):
          - Đã homing (zero_offset có giá trị)
          - Robot không báo bận (_motion_busy=False)
          - Có khay trên băng tải (S1 OR S2 OR S3 ON)
          - Vị trí cấp khay trống (S7_TRAY_AT_ROBOT=OFF) — xem INVARIANT trên
          - Cylinder 1 đã thu về (S9 ON, S10 OFF)
        """
        if self.operation_mode not in ['auto', 'ai'] or not self.zero_offset or self._motion_busy:
            return False
        has_tray     = self.sensor(S1_BELT_START) or self.sensor(S2_BELT_MID) or self.sensor(S3_BELT_END)
        place_ok     = not self.sensor(S7_TRAY_AT_ROBOT)
        cyl1_ret_ok  = self.sensor(S9_CYL1_RETRACTED)    # S9_CYL1_RETRACTED
        cyl1_ext_ok  = not self.sensor(S10_CYL1_EXTENDED)  # S10_CYL1_EXTENDED must be OFF
        return has_tray and place_ok and cyl1_ret_ok and cyl1_ext_ok

    def _can_start_s2a(self) -> bool:
        """
        Điều kiện để **auto-trigger** STATE 2 (chỉ AUTO/AI mode):
          - operation_mode in ['auto', 'ai'] (MANUAL không auto-chain — operator phải nhấn STATE2)
          - Đã homing
          - Robot đã báo xong khay input (_input_tray_done=True, set bởi _cb_done_tray_input)
          - Khay đang ở vị trí robot (S7_TRAY_AT_ROBOT=ON)
          - Robot không báo bận
        """
        if self.operation_mode not in ['auto', 'ai']:
            return False
        return (bool(self.zero_offset) and getattr(self, '_input_tray_done', False)
                and self.sensor(S7_TRAY_AT_ROBOT) and not self._motion_busy)

    def _can_start_s3(self) -> bool:
        """
        Điều kiện để auto-trigger STATE 3 (chỉ AUTO/AI mode):
          - output_stack_present=True (hardware Servo3 + CPX 254 đã lắp)
          - Đã homing và robot không bận
          - S18_FEED_OK=OFF (vị trí feed đang trống)
          - S18 đã OFF liên tục ít nhất 5 giây (debounce — tránh trigger ngay sau khi
            robot lấy khay, đợi hệ thống ổn định trước khi cấp khay mới)
        """
        if not self._conf('output_stack_present', True):
            return False
        if self.operation_mode not in ['auto', 'ai'] or self._motion_busy:
            return False
            
        # Neu S18 dang OFF va da duoc ghi nhan thoi gian
        s10_off_duration = 0.0
        if not self.sensor(S18_FEED_OK):
            if getattr(self, '_s10_off_time', 0) > 0:
                s10_off_duration = time.time() - self._s10_off_time
            else:
                s10_off_duration = 5.0
                
        # S17 ON = có khay trên Platform sẵn sàng cấp — thiếu check này trước đây
        # khiến auto-trigger fire khi platform trống rồi treo ở S3_WAIT_S17,
        # chiếm busy flag vô ích.
        return (bool(self.zero_offset)
                and self.sensor(S17_PLATFORM)
                and not self.sensor(S18_FEED_OK)
                and s10_off_duration >= 5.0)

    def _can_start_s4(self) -> bool:
        """
        Điều kiện để **auto-trigger** STATE 4 (chỉ AUTO/AI mode — thay khay output):
          - output_stack_present=True (hardware OutX/OutY + CPX 254 đã lắp)
          - operation_mode in ['auto', 'ai'] (MANUAL không auto-chain — operator phải nhấn STATE4)
          - Đã homing
          - _s4_trigger=True (set bởi robot qua /robot/done_tray_output)
          - S18_FEED_OK=ON (có khay ở vị trí feed — đảm bảo output tray đang được dùng)
          - Robot không báo bận
        """
        if not self._conf('output_stack_present', True):
            return False
        if self.operation_mode not in ['auto', 'ai']:
            return False
        return bool(self.zero_offset) and self._s4_trigger and self.sensor(S18_FEED_OK) and not self._motion_busy

    def _pub_cartridge_busy(self, busy: bool):
        """
        Publish trạng thái bận của hệ thống nạp cartridge lên /cartridge/busy.
        Robot node subscribe topic này để biết khi nào hệ thống nạp đang chạy.
        """
        busy = bool(busy)
        self.pub_busy_cartridge.publish(Bool(data=busy))
        self._last_cartridge_busy_published = busy

    def _sync_combined_cartridge_busy(self):
        """Publish one global interlock for every active cartridge state.

        Input, STATE 3 and STATE 4 may run concurrently.  A per-chain COMPLETE
        callback can therefore publish False while another chain is still
        moving.  Recompute the aggregate after every state-machine tick and
        correct that race before the robot may start a new motion.
        """
        homing_busy = self.state in (SystemState.HOMING, SystemState.HOMING_RUNNING)
        combined = (homing_busy
                    or self.state_in != SystemState.IDLE
                    or self.state_s3 != SystemState.IDLE
                    or self.state_s4 != SystemState.IDLE)
        if combined != self._last_cartridge_busy_published:
            self._pub_cartridge_busy(combined)

    def _pub_cartridge_pos2_busy(self, busy: bool):
        """
        Publish trạng thái bận riêng Pos2 (STATE 3 cấp khay thành phẩm /
        STATE 4 thay khay output) lên /cartridge/pos2_busy.
        Robot dùng để block PLACE_TO_OUTPUT khi cụm OutX/OutY đang di chuyển.
        """
        self.pub_busy_cartridge_pos2.publish(Bool(data=busy))

    def _input_belt_empty(self) -> bool:
        """True khi cả S1/S2/S3 đều OFF."""
        return not (
            self.sensor(S1_BELT_START)
            or self.sensor(S2_BELT_MID)
            or self.sensor(S3_BELT_END)
        )

    def _publish_drain(self, active: bool):
        active = bool(active)
        if active != self._drain_state:
            self.get_logger().warn(
                f"[DRAIN] /cartridge/drain = {active} "
                f"(armed_after_s2={self._drain_armed_after_s2}, state_in={self.state_in.name})"
            )
        self._drain_state = active
        self.pub_drain.publish(Bool(data=active))

    def _cb_motion_busy(self, msg: Bool):
        """
        Nhận trạng thái bận của robot từ /robot/motion_busy.
        Cập nhật _motion_busy và timestamp heartbeat.
        Lần đầu nhận → set _robot_connected=True, kích hoạt interlock an toàn.
        """
        self._motion_busy = msg.data
        self._robot_last_seen = time.time()
        if not self._robot_connected:
            self._robot_connected = True
            self.get_logger().info("[ROBOT] Robot node connected — interlock ACTIVE")

    def _cb_done_tray_output(self, msg: Bool):
        """Nhận tín hiệu từ robot báo khay output đã đầy → set cờ _s4_trigger để
        kích hoạt STATE 4 thay khay output trong vòng lặp tiếp theo."""
        """Robot báo khay output đã đầy → trigger State4 thay khay."""
        if msg.data:
            self._s4_trigger = True
            self._notify('info', 'Output tray full', 'Trigger State 4 thay khay')
            self.pub_newtray_output.publish(Bool(data=False))

    def _cb_done_tray_input(self, msg: Bool):
        """
        Robot báo đã xong khay input → set _input_tray_done.
        Cả AUTO lẫn MANUAL đều nhận topic này để trigger STATE 2:
          - AUTO: robot pub topic khi xong khay; sau S2A_COMPLETE auto chuỗi S1 lại.
          - MANUAL: user nhấn nút STATE 2 → QML pub topic; sau S2A_COMPLETE dừng,
            không tự kích S1 (giữ _state1_enabled=False trong manual).
        STATE 2 trigger qua _can_start_s2a() trong _do_idle_input() (check S7).
        """
        if msg.data:
            self._input_tray_done = True
            self._drain_armed_after_s2 = False
            self._publish_drain(False)
            # Chỉ enable auto-chain S1 trong AUTO/AI — MANUAL dừng chain sau S2A
            if self.operation_mode in ('auto', 'ai'):
                self._state1_enabled = True
            self.get_logger().info(
                '[DONE_INPUT] Robot xong khay input → sẵn sàng trigger State2 '
                f'(mode={self.operation_mode})'
            )
            self._notify('info', 'Input tray done', 'Robot xong — chờ State2')
            self.pub_new_tray.publish(Bool(data=False))



    def _outy_safe(self) -> bool:
        """
        Kiểm tra OutY đang ở vùng an toàn (≤ outy_safe_zone mm).
        Điều kiện cần trước khi STATE 4 bắt đầu di chuyển OutX.
        """
        p = self._pos(5)
        return p is not None and p <= self.config.outy_safe_zone

    # ══════════════════════════════════════════════════════════════
    # Homing
    # ══════════════════════════════════════════════════════════════

    def _home_all(self, abort_event: threading.Event) -> bool:
        """
        Thực hiện homing toàn bộ servo theo 2 cụm song song:
          Pos1/Input : InY → InX
          Pos2/Output: OutY → OutX → Platform (Servo3)

        Hai cụm Pos1/Pos2 có thể homing cùng lúc vì độc lập cơ khí, nhưng trong
        từng cụm phải home trục Y xong rồi mới tới trục X. Đặc biệt: OUTY phải
        homing xong mới được homing OUTX, giống rule InY trước InX ở Pos1.

        Logic mỗi servo:
          - Luôn gọi referencing_task() theo phương pháp trong FAS firmware
            (limit switch / encoder index / torque), chờ referenced()=True
          - Kiểm tra drive thực sự di chuyển (Δ > 0.5mm) để phát hiện lỗi cơ học

        Hỗ trợ abort: _homing_abort Event có thể được set bởi _cb_stop() để
        dừng homing ngay lập tức tại bất kỳ bước nào.
        Chạy trong background thread (_do_homing), kết quả báo qua _homing_done_event.
        """
        """[V8] Home pattern — mô phỏng dosing-machine code đã chạy thật.
        Home tuần tự và chỉ chốt zero_offset sau khi drive báo referenced=True.
        Fail loud thay vì silent-assume-home khi drive không sync hardware.
        """
        sequences = {
            "Pos1/Input": [(2, "InY"), (1, "InX")],
            "Pos2/Output": [(5, "OutY"), (4, "OutX"), (3, "Platform")],
        }

        if not hasattr(self, '_servo_homed_once'):
            self._servo_homed_once = {sid: False for sid in self.config.servo_ips}

        def _home_one(sid: int, name: str) -> bool:
            if sid not in self.servos:
                self.get_logger().info(f"  {name} (S{sid}): not connected, skip")
                return True
            mot = self.servos[sid]

            try:
                ready_ok = False
                for ready_attempt in range(1, 4):
                    if abort_event.is_set():
                        self.get_logger().warn(f"  {name}: homing aborted (STOP) before ready")
                        self._stop_immediate(sid)
                        return False
                    if self._ensure_ready(mot, sid, timeout=5.0):
                        ready_ok = True
                        break
                    self._ready_until.pop(sid, None)
                    self.get_logger().warn(
                        f"  {name}: drive not ready before homing "
                        f"(attempt {ready_attempt}/3)"
                    )
                    try:
                        with self._servo_lock:
                            mot.stop_motion_task()
                    except Exception:
                        pass
                    time.sleep(1.0)

                # STOP may arrive while _ensure_ready() is polling Modbus. Check
                # again before any command that can start physical movement.
                if abort_event.is_set():
                    self.get_logger().warn(f"  {name}: homing aborted (STOP) after ready")
                    self._stop_immediate(sid)
                    return False

                if not ready_ok:
                    self._notify_step('error', 'HOMING', f'{name} (S{sid})',
                        'Drive chưa ready nên không gửi lệnh referencing_task',
                        enum_name='HOMING_RUNNING',
                        check=[f'FAS Diagnostic S{sid}: drive enable/ready',
                               'PLC control granted nhưng ready_for_motion phải TRUE',
                               'Powerstage/enable, STO, limit switch, warning trên drive'],
                        action=['ACK lỗi trên FAS rồi HOMING lại',
                                'Nếu vẫn lặp: power-cycle drive hoặc kiểm tra enable/STO'])
                    return False

                # Đánh dấu motion timestamp để _publish_positions đọc tươi trong khi homing
                self._servo_motion_t[sid] = time.time()

                with self._servo_lock:
                    current_pos = mot.current_position()
                self.get_logger().info(f"Homing {name} (S{sid}): current={current_pos}")

                # Luôn reference hardware. Không dùng shortcut "near zero" vì nó có
                # thể set zero_offset cũ trước khi drive sync lại với REF thật.
                self.get_logger().info(
                    f"  {name}: {'first home' if not self._servo_homed_once.get(sid) else 're-home'} "
                    f"→ referencing_task"
                )
                pre_pos = current_pos

                with self._servo_lock:
                    # This check and referencing_task share the command lock.
                    # If STOP won first, HOME cannot restart the old command.
                    if abort_event.is_set():
                        self.get_logger().warn(
                            f"  {name}: homing aborted (STOP) before reference command"
                        )
                        return False
                    mot.referencing_task(nonblocking=True)
                # Lock released immediately — _cb_stop có thể acquire lock ngay

                # 3s interruptible delay: cho drive xóa stale referenced() flag từ NVM
                # và bắt đầu chuyển động vật lý. Chia nhỏ thành nhiều bước 200ms
                # để refresh _servo_motion_t — giữ _publish_positions đọc tươi suốt
                # homing, GUI tracking realtime thay vì freeze sau 2s idle skip.
                delay_start = time.time()
                while time.time() - delay_start < 3.0:
                    if abort_event.is_set():
                        self.get_logger().warn(f"  {name}: homing aborted (STOP) during init delay")
                        self._stop_immediate(sid)
                        return False
                    self._servo_motion_t[sid] = time.time()
                    time.sleep(0.2)

                # Poll referenced() với abort check mỗi iteration + refresh motion
                # timestamp để publish loop tiếp tục đọc tươi vị trí trong khi trục
                # vẫn đang di chuyển về home.
                start = time.time()
                while time.time() - start < self.config.homing_timeout:
                    if abort_event.is_set():
                        self.get_logger().warn(f"  {name}: homing aborted (STOP) during poll")
                        self._stop_immediate(sid)
                        return False
                    self._servo_motion_t[sid] = time.time()
                    with self._servo_lock:
                        ref = mot.referenced()
                    if ref:
                        break
                    time.sleep(0.2)
                else:
                    with self._servo_lock:
                        is_ref = mot.referenced()
                    if not is_ref:
                        self.get_logger().error(
                            f"  {name}: homing TIMEOUT ({self.config.homing_timeout}s) — NOT referenced"
                        )
                        self._notify_step('error', 'HOMING', f'{name} (S{sid})',
                            f'TIMEOUT {self.config.homing_timeout}s — drive không trả referenced=True',
                            enum_name='HOMING_RUNNING',
                            check=[f'FAS Referencing tab method (Encoder/REF cam/Torque)',
                                   'REF cam wiring + position',
                                   f'PNU 11340 (homing velocity) trên drive S{sid}',
                                   'cơ khí trục có bị kẹt không'],
                            action=['Mở FAS Editor → tab Referencing → test home thủ công',
                                    'STOP rồi HOMING lại sau khi fix'])
                        return False

                # Chốt homing: referencing_task(nonblocking) chỉ set bit
                # start_homing_procedure=True và KHÔNG tự clear. Nếu để bit treo
                # HIGH, drive coi lệnh homing còn active → trục về tới reference
                # xong vẫn re-trigger/dò home lại (S4 OutX xa home), home_position_set
                # không latch ổn định → poll timeout 120s. stop_motion_task() clear
                # start_homing_procedure=False để kết thúc homing (giống bản blocking
                # edcon wait_for_referencing_execution).
                if abort_event.is_set():
                    self.get_logger().warn(f"  {name}: homing aborted (STOP) before finalize")
                    self._stop_immediate(sid)
                    return False
                with self._servo_lock:
                    mot.stop_motion_task()

                if abort_event.is_set():
                    self.get_logger().warn(f"  {name}: homing aborted (STOP) after finalize")
                    self._stop_immediate(sid)
                    return False
                with self._servo_lock:
                    post_pos = mot.current_position()

                delta_mm = abs(post_pos - pre_pos) / COUNTS_PER_MM

                if delta_mm < 0.5:
                    with self._servo_lock:
                        is_ref = mot.referenced()
                    if not is_ref:
                        self.get_logger().error(
                            f"  {name}: NOT referenced + no motion (Δ={delta_mm:.2f}mm) "
                            f"→ HOMING FAIL. Check FAS: Referencing tab method/velocity/REF-cam."
                        )
                        self._notify_step('error', 'HOMING', f'{name} (S{sid})',
                            f'Drive KHÔNG di chuyển (Δ={delta_mm:.2f}mm) và không referenced',
                            enum_name='HOMING_RUNNING',
                            check=['FAS Referencing tab method có đúng không',
                                   'REF cam có wire OK không',
                                   'drive có fault không (FAS Diagnostic)',
                                   f'PNU 11340 velocity ≠ 0 trên S{sid}'],
                            action=['FAS Editor → Referencing tab → test',
                                    'Acknowledge faults trên drive',
                                    'STOP rồi HOMING lại'])
                        return False
                    self.get_logger().warn(
                        f"  {name}: moved only {delta_mm:.2f}mm but drive says referenced — "
                        f"may already be at REF position"
                    )

                # Final state commit is serialized with _cb_stop cancellation.
                # Either HOME commits first and STOP subsequently clears it, or
                # STOP sets abort first and this stale thread cannot restore it.
                with self._homing_guard_lock:
                    if abort_event.is_set():
                        self.get_logger().warn(
                            f"  {name}: homing aborted (STOP) before zero commit"
                        )
                        self._stop_immediate(sid)
                        return False
                    self._servo_homed_once[sid] = True
                    self.zero_offset[sid] = post_pos
                    self._need_warm_up[sid] = True  # FAS quirk: flush before first position_task
                self.get_logger().info(f"  {name} = 0mm (homed, Δ={delta_mm:.1f}mm)")
                return True

            except Exception as e:
                self.get_logger().error(f"  {name} home exception: {e}")
                servo_ip = self.config.servo_ips.get(sid, 'IP không xác định')
                self._notify_step('error', 'HOMING', f'{name} (S{sid}) exception',
                    f'Python exception: {str(e)[:80]}',
                    check=['cáp Modbus servo còn cắm không',
                           'drive có cấp điện 24V không',
                           'log cartridge_node.log cho full traceback'],
                    action=[f'Kiểm tra/ping S{sid}: {servo_ip}',
                            'Restart node nếu cần',
                            'Báo dev nếu exception lặp lại'])
                return False

        def _home_sequence(label: str, order: list, results: dict):
            self.get_logger().info(f"[HOMING] {label} sequence start")
            for sid, name in order:
                if abort_event.is_set():
                    results[label] = False
                    self.get_logger().warn(f"[HOMING] {label} aborted before {name}")
                    return
                if not _home_one(sid, name):
                    results[label] = False
                    abort_event.set()
                    self.get_logger().error(f"[HOMING] {label} failed at {name} (S{sid})")
                    return
            results[label] = True
            self.get_logger().info(f"[HOMING] {label} sequence complete")

        results = {}
        threads = [
            threading.Thread(
                target=_home_sequence,
                args=(label, order, results),
                daemon=True,
                name=f"homing_{label.lower().split('/')[0]}",
            )
            for label, order in sequences.items()
        ]

        for t in threads:
            t.start()
        for t in threads:
            t.join()

        return all(results.get(label, False) for label in sequences)

    # ══════════════════════════════════════════════════════════════
    # ROS Callbacks
    # ══════════════════════════════════════════════════════════════

    def _sync_mode_jog(self):
        """
        Publish mode hiện tại lên /providesystem/current_mode để GUI cập nhật.
        Nếu đang trong MANUAL mode và đã homed → publish "jog" (GUI hiển thị nút JOG).
        Ngược lại → publish tên mode thực (auto/manual/ai).
        """
        msg = String()
        # If in manual and homed, we are in 'jog'. Else just the operation_mode.
        if getattr(self, '_jog_mode', False) and self.operation_mode == 'manual':
            msg.data = "jog"
        else:
            msg.data = getattr(self, 'operation_mode', 'manual')
        self.pub_current_mode.publish(msg)

    def _cb_start(self, msg: Bool):
        """
        Xử lý lệnh START từ /system/start_button (GUI hoặc operator nhấn nút).
        - AUTO/AI mode và chưa homed → chuyển sang HOMING tự động
        - AUTO/AI mode và đã homed → chuyển về IDLE, bắt đầu auto-trigger STATE
        - MANUAL mode → giữ nguyên label (manual/jog tùy `_jog_mode` hiện tại),
          chỉ set _system_running để cho phép trigger STATE thủ công. KHÔNG ép
          vào JOG — chỉ vào JOG khi STOP hoặc khi HOMING hoàn tất ở manual.
        Bỏ qua nếu đang trong HOMING_RUNNING để tránh xung đột.
        """
        if not msg.data or self.state == SystemState.HOMING_RUNNING:
            return

        self._system_paused = False
        self._pause_pending = False
        self._paused_servo_moves.clear()
        self._pause_started_at = 0.0
        self._pause_restart_homing = False
        self._system_running = True
        self._inx_moving = self._iny_moving = False
        self._state1_enabled = (self.operation_mode in ['auto', 'ai'])
        self._drain_armed_after_s2 = False
        self._publish_drain(False)
        self._motion_busy = False

        # Auto/AI: tắt JOG flag để GUI hiển thị đúng mode.
        # Manual: KHÔNG động _jog_mode — giữ nguyên trạng thái hiện tại
        # (manual hoặc jog tùy chu trình trước).
        if self.operation_mode != 'manual':
            self._jog_mode = False

        self.get_logger().info(f"[START] System running. Mode: {self.operation_mode}")
        self._sync_mode_jog()
        
        if self.operation_mode in ['auto', 'ai']:
            self._setup_cpx_valves_for_auto_ai()
            if not self.zero_offset:
                self.get_logger().info("[START] Auto/AI mode — not homed → HOMING")
                self._homing_ref_offset.clear()
                # Reset drive warm-up gate — sau homing phải flush lại trước motion đầu tiên
                self._drive_warm_t = -1.0
                self._enter(SystemState.HOMING)
            else:
                self._enter(SystemState.IDLE)
        else:
            self.get_logger().info("[START] Manual mode — keeping current state")

    def _cb_stop(self, msg: Bool):
        """
        Xử lý lệnh STOP khẩn cấp từ /system/stop_button.
        Thực hiện theo thứ tự an toàn:
          1. Set _homing_abort để dừng thread homing ngay lập tức
          2. Dừng tất cả servo (stop_motion_task)
          3. Thu về cả 2 cylinder
          4. Reset toàn bộ state machine về IDLE
          5. Clear zero_offset → BẮT BUỘC re-home khi START tiếp (auto/ai)
          6. Chuyển sang MANUAL mode + JOG sẵn sàng
        Khác với PAUSE: STOP xóa zero_offset, PAUSE giữ nguyên (resume mid-cycle).
        Manual mode không tự auto-home — operator phải nhấn nút HOMING tay.
        Gửi mode MANUAL (code 3) đến robot node để đồng bộ.
        """
        if not msg.data:
            return
        # Cancel BOTH homing paths first:
        #   - START/system homing watches _homing_abort
        #   - Servo-card HOME threads watch _servo_home_abort
        # The guard prevents either stale thread from committing zero_offset
        # concurrently with this cancellation.
        servo_ids = list(self.servos)
        with self._homing_guard_lock:
            self._homing_abort.set()
            self._homing_generation += 1  # invalidate result from stopped run
            self._servo_home_abort.update(servo_ids)
            for sid in servo_ids:
                self._servo_home_generation[sid] = (
                    self._servo_home_generation.get(sid, 0) + 1
                )

        # Broadcast STOP telegrams without waiting for drive_stopped. The edcon
        # public stop_motion_task() can wait forever during HOME; calling it
        # sequentially caused only the first axis to stop.
        for sid in servo_ids:
            self._stop_immediate(sid)
        # KHÔNG retract Cyl1/Cyl2/Cyl3 khi STOP — giữ nguyên trạng thái valve trên CPX
        # (tránh thả khay đang kẹp). Operator can thiệp thủ công nếu cần.
        # Cyl3 safety watchdog (S13+S14 OFF) vẫn chạy độc lập trong _control_loop.
        self._enter(SystemState.IDLE)
        self._enter_in(SystemState.IDLE)
        self._enter_s3(SystemState.IDLE)
        self._enter_s4(SystemState.IDLE)
        self._system_paused  = False
        self._pause_pending  = False
        self._paused_servo_moves.clear()
        self._pause_started_at = 0.0
        self._pause_restart_homing = False
        self._system_running = False
        self._input_tray_done = False
        self._s2_output_full_accept = False
        self._motion_busy = False
        self._state1_enabled = False
        self._drain_armed_after_s2 = False
        self._publish_drain(False)
        self._s4_trigger = False
        # Re-arm drive warm-up gate — phòng khi STOP xảy ra trong tình huống bất thường
        # (drive vẫn có thể ở state lạ); state1 sau START sẽ tự flush.
        self._drive_warm_t = -1.0

        # Clear zero_offset → bắt buộc re-home ở lần START tiếp (auto/ai tự enter
        # HOMING; manual chờ operator nhấn nút HOMING).
        if self.zero_offset:
            self.zero_offset.clear()
            self._homing_ref_offset.clear()
            self._publish_homing_done(False)  # gate vfd_logic AUTO chờ re-home
            self.get_logger().info("[STOP] Cleared zero_offset — START lần sau sẽ phải re-home")

        # Chuyển về MANUAL mode. Không tự bật JOG sub-state ở STOP;
        # nút STOP STATE/STATE MODE trên Control Dashboard là nơi duy nhất
        # chuyển manual <-> jog cho cartridge system.
        self.operation_mode = 'manual'
        self._jog_mode = False
        self._sync_mode_jog()
        robot_msg = Int32()
        robot_msg.data = 3
        self.pub_robot_mode.publish(robot_msg)
        self.get_logger().info("[STOP] Hệ thống dừng - Đã chuyển về MANUAL mode")
        self._sync_mode_jog()
        self.get_logger().info('[STOP] STOP — MANUAL sẵn sàng (cần re-home trước STATE)')
        self._notify('warn', 'STOP', 'Dừng hệ thống — Cần HOMING trước khi chạy STATE')

    def _cb_soft_stop(self, msg: Bool):
        """
        Soft STOP từ /system/soft_stop — dừng motion NGAY, đưa hệ thống về MANUAL
        nhưng GIỮ NGUYÊN trạng thái CPX, zero_offset, và mọi flag tracking.
        Khác _cb_stop:
          - KHÔNG clear zero_offset (không buộc re-home)
          - KHÔNG reset trigger flags (_input_tray_done, _s4_trigger, ...)
          - KHÔNG touch cylinder/gripper/picker valve (giữ nguyên CPX)
        Dùng cho nút STOP nhẹ trên CameraPage — operator muốn pause workflow để
        kiểm tra mà không mất tracking.
        """
        if not msg.data:
            return
        # Stop motion ngay
        servo_ids = list(self.servos)
        with self._homing_guard_lock:
            self._homing_abort.set()
            self._homing_generation += 1  # soft STOP also owns cancellation
            self._servo_home_abort.update(servo_ids)
            for sid in servo_ids:
                self._servo_home_generation[sid] = (
                    self._servo_home_generation.get(sid, 0) + 1
                )
        for sid in servo_ids:
            self._stop_immediate(sid)

        # Reset state machine về IDLE để không tự advance
        # (giữ zero_offset, không clear tracking flags)
        self._enter(SystemState.IDLE)
        self._enter_in(SystemState.IDLE)
        self._enter_s3(SystemState.IDLE)
        self._enter_s4(SystemState.IDLE)

        self._system_paused  = False
        self._pause_pending  = False
        self._paused_servo_moves.clear()
        self._pause_started_at = 0.0
        self._pause_restart_homing = False
        self._system_running = False
        self._motion_busy    = False
        self._state1_enabled = False
        self._drain_armed_after_s2 = False
        self._publish_drain(False)

        # Switch sang MANUAL mode. JOG sub-state chỉ bật qua Control Dashboard.
        self.operation_mode = 'manual'
        self._jog_mode = False
        self._sync_mode_jog()
        robot_msg = Int32()
        robot_msg.data = 3
        self.pub_robot_mode.publish(robot_msg)

        self.get_logger().info("[SOFT_STOP] Dừng motion, giữ nguyên state + CPX, chuyển MANUAL")
        self._notify('warn', 'SOFT STOP', 'Dừng motion — giữ nguyên trạng thái + CPX')

    def _cb_pause(self, msg: Bool):
        """
        PAUSE tức thời: freeze state/sub-state hiện tại, phát STOP telegram tới
        các servo đang chạy và lưu target. RESUME phát lại target đó để step
        tiếp tục từ vị trí vật lý đang dừng, không reset state/counter/CPX.

        Referencing không hỗ trợ Continue giữa chừng; nếu PAUSE trong HOMING,
        dừng toàn bộ trục và RESUME sẽ chạy lại HOMING an toàn từ đầu.
        """
        if not msg.data:
            return
        if self._system_paused:
            return  # idempotent
        self._system_paused = True
        self._pause_pending = False
        self._pause_started_at = time.time()
        self._pause_restart_homing = self.state in (
            SystemState.HOMING, SystemState.HOMING_RUNNING)

        if self._pause_restart_homing:
            # Invalidate thread cũ để nó không commit zero_offset sau PAUSE.
            with self._homing_guard_lock:
                self._homing_abort.set()
                self._homing_generation += 1
            pause_axes = list(self.servos)
            self._paused_servo_moves.clear()
        else:
            # Snapshot trước khi _stop_immediate() xóa active map.
            self._paused_servo_moves = dict(self._active_servo_moves)
            pause_axes = list(self._paused_servo_moves)

        for sid in pause_axes:
            self._stop_immediate(sid)

        self.get_logger().warn(
            f"[PAUSE] Instant stop; state={self.state.name}, "
            f"in={self.state_in.name}, s3={self.state_s3.name}, "
            f"s4={self.state_s4.name}, axes={pause_axes}"
        )
        detail = ('HOMING sẽ chạy lại từ đầu khi RESUME'
                  if self._pause_restart_homing
                  else f'Đã giữ step; dừng servo {pause_axes or "(không có motion)"}')
        self._notify('warn', 'PAUSED', detail)

    def _cb_resume(self, msg: Bool):
        """
        RESUME motion đang giữ rồi mở lại đúng state/sub-state hiện tại.
        Thời gian PAUSE được cộng bù vào các deadline để không tạo timeout giả.
        """
        if not msg.data:
            return
        if not self._system_paused:
            return  # nothing to resume

        paused_for = max(0.0, time.time() - self._pause_started_at)
        restarted_axes = []
        if self._pause_restart_homing:
            # Thread homing cũ đã bị invalidate; khởi động một generation mới.
            self._homing_done_event.clear()
            self._enter(SystemState.HOMING)
        else:
            failed = []
            for sid, (target, vel, continuous) in sorted(self._paused_servo_moves.items()):
                if self._nb_move(sid, target, vel=vel, continuous_update=continuous):
                    restarted_axes.append(sid)
                else:
                    failed.append(sid)
            if failed:
                # Không để vài trục chạy trong khi hệ thống vẫn mang cờ PAUSED.
                for sid in restarted_axes:
                    self._stop_immediate(sid)
                self.get_logger().error(
                    f"[RESUME] Cannot restore servo targets: {failed}; still PAUSED")
                self._notify('error', 'RESUME thất bại',
                             f'Không phát lại được target servo {failed}; hệ thống vẫn PAUSED')
                return

        self._shift_pause_timers(paused_for)
        self._system_paused = False
        self._pause_pending = False
        self._paused_servo_moves.clear()
        self._pause_started_at = 0.0
        restart_homing = self._pause_restart_homing
        self._pause_restart_homing = False
        # Refresh heartbeat để tránh false-positive timeout ngay sau khi resume
        # (pause lâu → _robot_last_seen stale → heartbeat check sẽ fire ngay).
        if self._robot_last_seen > 0:
            self._robot_last_seen = time.time()
        detail = ('Khởi động lại HOMING an toàn'
                  if restart_homing
                  else f'Tiếp tục step hiện tại; servo {restarted_axes or "(không có motion)"}')
        self.get_logger().info(f"[RESUME] {detail}; pause duration={paused_for:.2f}s")
        self._notify('info', 'RESUMED', detail)

    def _shift_pause_timers(self, paused_for: float):
        """Freeze các deadline/state delay trong khoảng PAUSE."""
        if paused_for <= 0.0:
            return
        timer_names = (
            '_drive_warm_t', '_step_start_in', '_step_timeout_in',
            '_30s_timeout', '_tray_robot_check_start', '_s10_warn_t',
            '_cyl_retry_t', '_place_delay_start', '_step_start_s3',
            '_step_timeout_s3', '_cyl4_retry_t', '_step_start_s4',
            '_step_timeout_s4', '_outy_jog_start', '_cyl2_retry_t',
            '_cyl5_retry_t', '_s10_off_time', '_cyl3_cmd_time',
        )
        for name in timer_names:
            value = getattr(self, name, 0.0)
            if isinstance(value, (int, float)) and value > 0.0:
                setattr(self, name, value + paused_for)

    def _can_pause_now(self) -> bool:
        """
        Legacy helper kept for compatibility. Instant PAUSE no longer waits for
        a state boundary, therefore it is always promotable.
        """
        return True

    def _promote_pause(self):
        """Legacy compatibility path; new PAUSE is promoted in _cb_pause."""
        self._system_paused = True
        self._pause_pending = False
        self._notify('warn', 'PAUSED', 'Hệ thống đã tạm dừng — nhấn RESUME để tiếp tục')

    def _cb_gripper_cmd(self, msg: Bool):
        """
        Điều khiển gripper (kẹp) qua IO Module 1 — valve 5/3, drive cả 2 chiều:
          msg.data=True  → Gắp (set ch0, reset ch1)
          msg.data=False → Nhả (reset ch0, set ch1)
        Khác valve 5/2 cũ: cả 2 trạng thái đều drive 1 coil, KHÔNG để both off
        (both off = vị trí center, không tác động cylinder).
        Publish trạng thái lên /robot/gripper_status để robot node biết.
        """
        if not self.io_module: return
        with self._io_bg_lock:
            try:
                valve_mod = self._io1_valve_module_locked()
                if valve_mod is None:
                    raise RuntimeError("CPX37 VABX valve manifold not found/ambiguous")
                if msg.data:  # Gắp
                    valve_mod.reset_channel(1)
                    valve_mod.set_channel(0)
                    self.pub_gripper_status.publish(Bool(data=True))
                else:  # Nhả
                    valve_mod.reset_channel(0)
                    valve_mod.set_channel(1)
                    self.pub_gripper_status.publish(Bool(data=False))
            except Exception as e:
                self.get_logger().error(f"Gripper cmd error: {e}")

    def _cb_picker_cmd(self, msg: Bool):
        """
        Điều khiển picker qua IO Module 1 — valve 5/3, drive cả 2 chiều:
          msg.data=True  → Gắp (set ch2, reset ch3)
          msg.data=False → Nhả (reset ch2, set ch3)
        Khác valve 5/2 cũ: cả 2 trạng thái đều drive 1 coil, KHÔNG để both off.
        Publish trạng thái lên /robot/picker_status.
        """
        if not self.io_module: return
        with self._io_bg_lock:
            try:
                valve_mod = self._io1_valve_module_locked()
                if valve_mod is None:
                    raise RuntimeError("CPX37 VABX valve manifold not found/ambiguous")
                if msg.data:  # Gắp
                    valve_mod.reset_channel(3)
                    valve_mod.set_channel(2)
                    self.pub_picker_status.publish(Bool(data=True))
                else:  # Nhả
                    valve_mod.reset_channel(2)
                    valve_mod.set_channel(3)
                    self.pub_picker_status.publish(Bool(data=False))
            except Exception as e:
                self.get_logger().error(f"Picker cmd error: {e}")

    def _cb_cyl_loadcell_cmd(self, msg: Bool):
        """
        Điều khiển cyl6/loadcell trên CPX 172.16.11.37, channel 8/9:
          (Đã đổi dây khí ngoài thực tế)
          msg.data=True  → Nhả/extend  (reset ch8, set ch9)
          msg.data=False → Kẹp/retract (reset ch9, set ch8)

        Node này đang sở hữu CPX 253, nên xử lý tại đây thay vì
        gripper_festo_node.py (node đó bị tắt trong start_all để tránh ghi
        trùng CPX).
        """
        if not self.io_module:
            self.get_logger().warn("Cyl6 cmd ignored: CPX 253 not connected")
            return
        with self._io_bg_lock:
            try:
                valve_mod = self._io1_valve_module_locked()
                if valve_mod is None:
                    raise RuntimeError("CPX37 VABX valve manifold not found/ambiguous")
                if msg.data:  # Nhả
                    valve_mod.reset_channel(8)
                    valve_mod.set_channel(9)
                    self.pub_cyl_loadcell_status.publish(Bool(data=True))
                    self.get_logger().info("Cyl6/loadcell NHẢ: ch8 reset, ch9 set")
                else:  # Kẹp
                    valve_mod.reset_channel(9)
                    valve_mod.set_channel(8)
                    self.pub_cyl_loadcell_status.publish(Bool(data=False))
                    self.get_logger().info("Cyl6/loadcell KẸP: ch9 reset, ch8 set")
            except Exception as e:
                self.get_logger().error(f"Cyl6/loadcell cmd error: {e}")

    def _cb_cyl_cmd(self, msg: String):
        """
        Điều khiển cylinder 1..5 thủ công từ GUI qua /providesystem/cyl_cmd.
        Định dạng: "<cyl_id> <extend|retract>"  ví dụ "1 extend", "5 retract".

        Mapping feedback:
          Cyl1: S9/S10, Cyl2: S21/S22, Cyl3: S15/S16,
          Cyl4: S25/S26, Cyl5: S27/S28.

        Chỉ cho phép trong MANUAL mode + state IDLE/ERROR + không có state machine
        nào đang chạy — tránh xung đột với logic STATE 1/2/3/4 đang điều khiển
        cylinder theo chu trình.
        """
        parts = msg.data.strip().split()
        if len(parts) < 2:
            return

        if self.operation_mode != 'manual':
            self._notify_step('warn', 'Cylinder', 'manual cmd',
                'Cylinder lock — chỉ điều khiển được trong MANUAL/JOG mode',
                action=['Chuyển GUI sang MANUAL', 'Vào JOG mode (homing xong)'])
            return
        if (self.state not in (SystemState.IDLE, SystemState.ERROR)
                or self.state_in != SystemState.IDLE
                or self.state_s3 != SystemState.IDLE
                or self.state_s4 != SystemState.IDLE):
            self._notify_step('warn', 'Cylinder', 'manual cmd',
                f'Đang chạy state ({self.state_in.name}/{self.state_s3.name}/{self.state_s4.name}) — Cylinder bị khóa',
                enum_name=f'in={self.state_in.name} s3={self.state_s3.name} s4={self.state_s4.name}',
                action=['Nhấn STOP', 'Đợi state machine về IDLE rồi điều khiển Cyl'])
            return

        try:
            cid = int(parts[0])
            act = parts[1].lower()
        except Exception as e:
            self.get_logger().warn(f"[CYL] parse error: {e}")
            return

        try:
            if cid == 1 and act == 'extend':
                self._cyl1_extend()
            elif cid == 1 and act == 'retract':
                self._cyl1_retract()
            elif cid == 2 and act == 'extend':
                self._cyl2_extend()
            elif cid == 2 and act == 'retract':
                self._cyl2_retract()
            elif cid == 3 and act in ('extend', 'retract'):
                if not self._conf('cyl3_present', True):
                    self._notify('warn', 'Cyl3 disabled', 'cyl3_present=false trong config')
                    return
                if act == 'extend':
                    self._cyl3_s13s14_latch = False
                    self._cyl3_extend()
                    self.get_logger().info("[CYL3] Manual EXTEND → S13/S14 latch cleared")
                else:
                    self._cyl3_retract()
            elif cid == 4 and act == 'extend':
                self._cyl4_extend()
            elif cid == 4 and act == 'retract':
                self._cyl4_retract()
            elif cid == 5 and act == 'extend':
                self._cyl5_extend()
            elif cid == 5 and act == 'retract':
                self._cyl5_retract()
            else:
                self.get_logger().warn(f"[CYL] unknown cmd: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"[CYL] exception cid={cid} act={act}: {e}")

    def _cb_gui_confirm(self, msg: String):
        """
        Nhận xác nhận từ operator qua GUI (topic /providesystem/gui_confirm).
        S2_OUTPUT_FULL_ACCEPT chỉ tạo một lần recheck S4 khi State 2
        đang ở S2A_WAIT_OUTPUT_CLEAR. Lệnh riêng này không được phép
        xác nhận nhầm các wait-state cũ.

        Các message khác giữ backward compatibility: set
        _gui_confirmed=True cho S1_WAIT_GUI_CONFIRM / S3_WAIT_GUI_CONFIRM.
        """
        command = str(getattr(msg, 'data', '')).strip().upper()
        if command == 'S2_OUTPUT_FULL_ACCEPT':
            if self.state_in == SystemState.S2A_WAIT_OUTPUT_CLEAR:
                self._s2_output_full_accept = True
                self.get_logger().info(
                    "[S2A] Operator ACCEPT -> request one-shot S4 recheck"
                )
            else:
                self.get_logger().warn(
                    f"[S2A] Ignore stale output-full ACCEPT in {self.state_in.name}"
                )
            return
        self._gui_confirmed = True

    def _cb_robot_mode(self, msg: Int32):
        """
        Đồng bộ mode từ robot node qua /robot/set_mode.
        Mapping cartridge: robot AUTO là AUTO, robot AI là AI, robot MANUAL là MANUAL.
        Khi mode thay đổi: xóa log guide để hiển thị message mới phù hợp mode mới.
        Khi chuyển sang AUTO: reset trigger flags (_input_tray_done, _s4_trigger)
        để bắt buộc chu trình mới từ STATE1/STATE3, không kế thừa state cũ.
        """
        mapping = {1: 'auto', 2: 'ai', 3: 'manual'}
        requested = mapping.get(msg.data, 'manual')

        if requested == 'manual' and self.operation_mode == 'manual' and getattr(self, '_jog_mode', False):
            self.get_logger().info("[SYNC MODE] Robot MANUAL received — giữ JOG sub-mode của cartridge")
            self._drain_armed_after_s2 = False
            self._publish_drain(False)
            self._sync_mode_jog()
            return

        if requested != self.operation_mode:
            old = self.operation_mode
            self.get_logger().info(f"[SYNC MODE] Cập nhật mode từ Robot Node: {requested.upper()}")
            self.operation_mode = requested
            self._jog_mode = False
            self._guide_logged.clear()
            self._sync_mode_jog()
            self._drain_armed_after_s2 = False
            self._publish_drain(False)
            self._reset_auto_triggers_on_mode_enter(old, requested)

    def _reset_auto_triggers_on_mode_enter(self, old_mode: str, new_mode: str):
        """
        Khi chuyển mode: clear zero_offset (bắt buộc re-home) + reset trigger flags.
        - Auto/AI sau khi clear: lần START tiếp theo sẽ tự enter HOMING.
        - Manual sau khi clear: operator phải nhấn nút HOMING tay; STATE trigger
          bị reject cho đến khi homed.
        An toàn: không động state machine / state_in / state_s3 / state_s4.
        """
        if old_mode == new_mode:
            return
        if self.zero_offset:
            self.zero_offset.clear()
            self._homing_ref_offset.clear()
            self._publish_homing_done(False)  # gate vfd_logic AUTO chờ re-home
            self.get_logger().info(
                f"[MODE {old_mode.upper()}→{new_mode.upper()}] Cleared zero_offset — cần re-home"
            )
        if new_mode in ('auto', 'ai'):
            self._input_tray_done = False
            self._s4_trigger = False
            self.get_logger().info(
                f"[MODE→{new_mode.upper()}] Reset _input_tray_done + _s4_trigger — "
                f"bắt đầu chu trình mới từ STATE1/STATE3"
            )

    def _cb_mode(self, msg: String):
        """
        Thay đổi operation mode từ GUI qua /providesystem/set_operation_mode.
        Chỉ cho phép đổi mode khi hệ thống ở IDLE hoặc ERROR (không đang chạy STATE).
        Khi đổi mode: sync GUI và thông báo đến robot node.
        """
        raw_requested = msg.data.strip().lower()
        if raw_requested not in ('auto', 'manual', 'ai', 'camera_ai', 'jog'):
            self._notify('warn', f'Mode khong hop le: {raw_requested}', '')
            return
        if raw_requested == 'auto':
            requested = 'auto'
        elif raw_requested in ('ai', 'camera_ai'):
            requested = 'ai'
        else:
            requested = 'manual'
        jog_requested = (raw_requested == 'jog')
        active_states = {SystemState.IDLE, SystemState.ERROR,
                         SystemState.HOMING, SystemState.HOMING_RUNNING}
        if self.state not in active_states:
            self._notify('warn', 'Khong the doi mode',
                         f'Dang chay ({self.state.name}) — nhan STOP truoc')
            return
        old = self.operation_mode
        self.operation_mode = requested
        self._drain_armed_after_s2 = False
        self._publish_drain(False)

        if jog_requested:
            self._jog_mode = True
            self.get_logger().info("[MODE] User chọn JOG — cartridge MANUAL/JOG, robot MANUAL")
        elif requested == 'manual' and getattr(self, '_jog_mode', False):
            self._jog_mode = False
            self.get_logger().info("[MODE] User chọn MANUAL — thoát JOG sub-state")
        elif requested != 'manual':
            self._jog_mode = False

        # LUÔN echo current_mode về GUI sau mỗi lệnh chọn mode hợp lệ — kể cả khi
        # mode KHÔNG đổi. GUI mặc định current_mode='idle' còn backend default
        # 'manual', nên lần đầu bấm MANUAL (old==requested=='manual') trước đây
        # không publish → GUI kẹt 'idle', mọi nút điều khiển bị khoá. Echo vô điều
        # kiện sửa đúng việc này; _sync_mode_jog idempotent nên an toàn gọi mọi lần.
        self._sync_mode_jog()

        if old != requested or jog_requested:
            self._guide_logged.clear()

            # Sync to robot node
            robot_msg = Int32()
            if requested == 'ai': robot_msg.data = 2
            elif requested == 'auto': robot_msg.data = 1
            else: robot_msg.data = 3
            self.pub_robot_mode.publish(robot_msg)

            self._reset_auto_triggers_on_mode_enter(old, requested)

        # MODE and START use separate ROS topics. START can therefore arrive
        # while operation_mode is still MANUAL, in which case _cb_start cannot
        # begin automatic homing. Converge to the requested running state when
        # the delayed AUTO/AI mode message arrives.
        if self._system_running and requested in ('auto', 'ai'):
            self._jog_mode = False
            self._state1_enabled = True
            self._setup_cpx_valves_for_auto_ai()
            if not self.zero_offset:
                if self.state not in (SystemState.HOMING, SystemState.HOMING_RUNNING):
                    self.get_logger().info(
                        "[MODE] AUTO/AI arrived after START — begin pending HOMING"
                    )
                    self._homing_ref_offset.clear()
                    self._drive_warm_t = -1.0
                    self._enter(SystemState.HOMING)
            elif self.state in (SystemState.IDLE, SystemState.ERROR):
                self.get_logger().info(
                    "[MODE] AUTO/AI arrived after START — homed, resume automatic process"
                )
                self._enter(SystemState.IDLE)

    def _cb_jog(self, msg: String):
        """
        Xử lý lệnh JOG từ GUI qua /providesystem/jog_cmd.
        Định dạng: "<servo_id> <cmd>" | "home <servo_id>" | "clear <servo_id>"

        Lệnh hỗ trợ:
          "clear <sid>"      → Acknowledge faults (bất kỳ mode nào)
          "<sid> stop"       → Dừng servo (bất kỳ mode/state nào)
          "<sid> +"          → JOG chiều dương (chỉ MANUAL+IDLE)
          "<sid> -"          → JOG chiều âm
          "<sid> move <pos>" → Di chuyển đến pos mm
          "home <sid>"       → Homing thủ công 1 servo (thread riêng)

        Các lệnh motion ngoài STOP/CLEAR bị khóa nếu đang AUTO mode hoặc bất kỳ
        state machine nào đang chạy.
        """
        parts = msg.data.strip().split()
        if not parts: return
        cmd0 = parts[0].lower()

        # Cho phép CLEAR lỗi (acknowledge faults giống FAS) bất kỳ lúc nào / mode nào
        if cmd0 == 'clear':
            if len(parts) >= 2:
                try:
                    sid = int(parts[1])
                    mot = self.servos.get(sid)
                    if mot:
                        mot.acknowledge_faults()
                        self._ready_until.pop(sid, None)
                        self.get_logger().info(f"Acknowledge faults cho Servo {sid} (như FAS)")
                except Exception as e:
                    self.get_logger().warn(f"Lỗi khi clear servo: {e}")
            return

        # SAFETY OVERRIDE: STOP từng trục phải luôn chạy được dù đang AUTO,
        # HOMING_RUNNING, STATE 1/2/3/4, hoặc không ở JOG sub-mode. GUI Servo
        # Control gửi "<sid> stop"; xử lý trước mọi interlock để operator luôn
        # dừng được trục ngay trên card servo.
        if len(parts) >= 2 and parts[1].lower() == 'stop':
            try:
                sid = int(parts[0])
                with self._homing_guard_lock:
                    self._servo_home_abort.add(sid)
                    self._servo_home_generation[sid] = (
                        self._servo_home_generation.get(sid, 0) + 1
                    )
                if self.state in (SystemState.HOMING, SystemState.HOMING_RUNNING):
                    self._homing_abort.set()
                    self.get_logger().warn(
                        f"[JOG] Servo S{sid} STOP during global homing -> abort homing thread"
                    )
                self._stop_immediate(sid)
                self.get_logger().info(f"[JOG] Servo S{sid} STOP override")
            except Exception as e:
                self.get_logger().warn(f"[JOG] stop parse/error: {msg.data} ({e})")
            return

        # Mọi lệnh JOG (kể cả RUN/target position, home thủ công, +/-) bắt buộc ở
        # JOG sub-mode (operation_mode == 'manual' AND _jog_mode == True). QML đã
        # gate `jogAllowed = currentMode === "jog"`; backend re-validate để tránh
        # client khác publish trực tiếp /providesystem/jog_cmd trong MANUAL state.
        if self.operation_mode != 'manual' or not getattr(self, '_jog_mode', False):
            self._notify_step('warn', 'JOG', 'cmd',
                'JOG lock — chỉ chạy được trong JOG sub-mode',
                action=['Chuyển sang MANUAL', 'Bấm nút JOG'])
            return

        # 'move' = TARGET POSITION RUN: chức năng phụ kỹ thuật, hoàn toàn ĐỘC LẬP
        # với state machine. Không check state_in/s3/s4, không check HOMING_RUNNING,
        # không trigger state transition. Chỉ gate bởi JOG sub-mode (đã check ở trên)
        # và servo đã homed (_nb_move tự reject nếu thiếu).
        is_move = (len(parts) >= 3 and parts[1].lower() == 'move')

        if not is_move:
            # JOG +/-, stop, home: vẫn cần interlock vì là continuous motion
            # / homing thread, dễ xung đột với state machine nếu đang chạy.
            if (self.state not in (SystemState.IDLE, SystemState.ERROR)
                    or self.state_in != SystemState.IDLE
                    or self.state_s3 != SystemState.IDLE
                    or self.state_s4 != SystemState.IDLE):
                self._notify_step('warn', 'JOG', 'cmd',
                    f'Đang chạy state — JOG bị khóa để tránh xung đột motion',
                    enum_name=f'in={self.state_in.name} s3={self.state_s3.name} s4={self.state_s4.name}',
                    action=['Nhấn STOP', 'Đợi state machine về IDLE rồi JOG'])
                return
        
        if len(parts) < 2:
            return
        try:
            if cmd0 == 'home':
                sid = int(parts[1])
                mot = self.servos.get(sid)
                if mot:
                    # Each Servo-card HOME owns an immutable generation token.
                    # Starting a new HOME must not clear a flag and accidentally
                    # revive an older thread that STOP already cancelled.
                    with self._homing_guard_lock:
                        self._servo_home_abort.discard(sid)
                        self._servo_home_generation[sid] = (
                            self._servo_home_generation.get(sid, 0) + 1
                        )
                        home_generation = self._servo_home_generation[sid]
                        system_generation = self._homing_generation

                    def _do(
                        mot=mot,
                        sid=sid,
                        home_generation=home_generation,
                        system_generation=system_generation,
                    ):
                        def _cancelled():
                            return (
                                sid in self._servo_home_abort
                                or home_generation != self._servo_home_generation.get(sid)
                                or system_generation != self._homing_generation
                            )

                        if not self._ensure_ready(mot, sid, timeout=5.0):
                            self.get_logger().warn(
                                f"S{sid} JOG home rejected: drive not ready"
                            )
                            return
                        # STOP can arrive while _ensure_ready() is polling.
                        if _cancelled():
                            self.get_logger().warn(f"S{sid} JOG home aborted before reference")
                            self._stop_immediate(sid)
                            return
                        with self._servo_lock:
                            if _cancelled():
                                self.get_logger().warn(
                                    f"S{sid} JOG home aborted before reference command"
                                )
                                return
                            mot.referencing_task(nonblocking=True)
                        # Refresh _servo_motion_t mỗi 200ms để _publish_positions
                        # đọc tươi suốt homing → GUI hiển thị vị trí từ từ về 0
                        # thay vì freeze sau 2s idle skip. (Mirror fix fce323e
                        # cho path _do_homing.)
                        delay_start = time.time()
                        while time.time() - delay_start < 3.0:
                            if _cancelled():
                                self.get_logger().warn(f"S{sid} JOG home aborted by servo STOP")
                                self._stop_immediate(sid)
                                return
                            self._servo_motion_t[sid] = time.time()
                            time.sleep(0.2)
                        start = time.time()
                        timeout = self.config.homing_timeout
                        homed_ok = False
                        while time.time() - start < timeout:
                            if _cancelled():
                                self.get_logger().warn(f"S{sid} JOG home aborted by servo STOP")
                                self._stop_immediate(sid)
                                return
                            with self._servo_lock:
                                ref = mot.referenced()
                            if ref:
                                homed_ok = True
                                break
                            self._servo_motion_t[sid] = time.time()
                            time.sleep(0.2)
                        if not homed_ok:
                            # Timeout: KHÔNG set zero_offset (sẽ làm move targets bị off
                            # so với physical zero). Stop motion + notify ERROR rõ ràng.
                            self.get_logger().error(f"S{sid} JOG home TIMEOUT after {timeout}s — KHÔNG set zero_offset")
                            with self._servo_lock:
                                mot.stop_motion_task()
                            self._notify('error', 'HOMING FAIL',
                                f'Servo S{sid} timeout sau {timeout}s — drive không trả referenced=True. '
                                f'Check FAS Referencing tab / REF cam / wiring. RUN target sẽ bị từ chối.')
                            return
                        # Chốt homing: clear bit start_homing_procedure (referencing_task
                        # nonblocking để treo HIGH → drive có thể re-trigger dò home lại).
                        # Mirror nhánh timeout ở trên + bản blocking edcon.
                        if _cancelled():
                            self.get_logger().warn(f"S{sid} JOG home aborted before finalize")
                            self._stop_immediate(sid)
                            return
                        with self._servo_lock:
                            mot.stop_motion_task()

                        if _cancelled():
                            self.get_logger().warn(f"S{sid} JOG home aborted after finalize")
                            self._stop_immediate(sid)
                            return
                        with self._servo_lock:
                            home_position = mot.current_position()

                        # Commit atomically against global/per-servo STOP.
                        with self._homing_guard_lock:
                            if _cancelled():
                                self.get_logger().warn(
                                    f"S{sid} JOG home aborted before zero commit"
                                )
                                return
                            self.zero_offset[sid] = home_position
                            self._need_warm_up[sid] = True  # FAS quirk: flush before first position_task
                            if hasattr(self, '_servo_homed_once'):
                                self._servo_homed_once[sid] = True
                        # Invalidate cache + mark active so _publish_positions reads fresh
                        self._pos_cache.pop(sid, None)
                        self._servo_motion_t[sid] = time.time()
                        self._notify('silent_ok', f'Homing complete', f'Servo {sid}')
                    threading.Thread(target=_do, daemon=True).start()
                return
            sid = int(parts[0])
            d   = parts[1]
            if d == 'stop':   self._stop_immediate(sid)
            elif d == '+':    self._jog(sid, True)
            elif d == '-':    self._jog(sid, False)
            elif d == 'move':
                pos = float(parts[2]) if len(parts) > 2 else 0.0
                if sid == 1:
                    min_val = -322.0
                    max_val = 560.0
                    if pos < min_val or pos > max_val:
                        self.get_logger().error(f"[JOG] S1 (InX) move rejected: {pos} exceeds limits [{min_val}, {max_val}]")
                        self._notify('error', 'LỖI GIỚI HẠN', f'Trục S1 (InX) vượt giới hạn [{min_val}, {max_val}] mm (Nhập: {pos})')
                        return
                elif sid == 2:
                    min_val = -80.0
                    max_val = 1025.0
                    if pos < min_val or pos > max_val:
                        self.get_logger().error(f"[JOG] S2 (InY) move rejected: {pos} exceeds limits [{min_val}, {max_val}]")
                        self._notify('error', 'LỖI GIỚI HẠN', f'Trục S2 (InY) vượt giới hạn [{min_val}, {max_val}] mm (Nhập: {pos})')
                        return
                # _nb_move có 2 reject path silent với GUI (chỉ log ROS):
                #   (a) pos > servo_limits[sid] (positive limit)
                #   (b) servo chưa homed (zero_offset chưa set)
                # Notify để operator thấy lý do RUN không di chuyển hardware.
                ok = self._nb_move(sid, pos)
                if not ok:
                    if sid not in self.zero_offset and sid in self.servos:
                        self._notify('error', 'RUN bị từ chối',
                            f'Servo S{sid} chưa homed — chạy HOMING trước khi RUN target')
                    else:
                        upper = self._conf('servo_limits', {}).get(sid, 999.0)
                        self._notify('error', 'RUN bị từ chối',
                            f'S{sid}: {pos}mm vượt servo_limits ({upper}mm)')
        except Exception as e:
            self.get_logger().error(f"[JOG] exception: {e}")
            self._notify('error', 'JOG exception', str(e))

    def _cb_goto_state(self, msg: String):
        """
        Nhận lệnh chuyển state thủ công từ GUI qua /providesystem/goto_state.
        Lệnh hỗ trợ: HOMING, IDLE, ABORT_TO_JOG, STATE1/S1, STATE2/S2, STATE3/S3, STATE4/S4.
        STATE1 chỉ cho phép trong MANUAL mode (AUTO tự trigger).
        STATE2 cho phép cả hai mode nếu điều kiện cảm biến hợp lệ.
        STATE3/4 cho phép bất kỳ mode nào nếu chưa đang chạy và đã homed.
        """
        cmd = msg.data.strip().upper()
        if self._drop_duplicate_manual_state_cmd(cmd):
            return
        if cmd == 'HOMING':
            if self.state != SystemState.HOMING_RUNNING:
                # Clear stale event nếu thread trước đã set sau STOP nhưng main
                # loop chưa consume (state đã về IDLE).
                self._homing_done_event.clear()
                # Không giữ offset cũ: homing phải sync lại với hardware trước
                # khi zero_offset được set và homing_done được publish.
                self._homing_ref_offset.clear()
                self.zero_offset.clear()
                self._publish_homing_done(False)  # vfd_logic gate AUTO chờ homing mới xong
                self._inx_moving = self._iny_moving = False
                self._enter(SystemState.HOMING)
            return
        if cmd == 'IDLE':
            for sid in list(self.servos):
                self._stop(sid)
            self._enter(SystemState.IDLE)
            self._enter_in(SystemState.IDLE)
            self._enter_s3(SystemState.IDLE)
            self._enter_s4(SystemState.IDLE)
            self._system_running = False
            self.get_logger().info("Got IDLE command, fully reset system states.")
            return

        if cmd == 'ABORT_TO_JOG':
            self._s1_abort("Operator stopped state (ABORT_TO_JOG)")
            return

        # ── STATE 1: chỉ cho MANUAL, operator nhấn tay ──────────
        if cmd in ('STATE1', 'STATE_1', 'S1'):
            if self.operation_mode != 'manual':
                self._notify_step('warn', 'STATE 1', 'trigger',
                    'Chỉ kích thủ công ở MANUAL mode',
                    action=['Chuyển GUI sang MANUAL', 'Nhấn STATE 1 lại'])
                return
            if self.state_in not in (SystemState.IDLE,):
                self._notify_step('warn', 'STATE 1', 'trigger',
                    f'{self.state_in.name} đang chạy — không thể vào STATE 1',
                    enum_name=self.state_in.name,
                    action=['Nhấn STOP', 'Đợi state hiện tại kết thúc rồi retry'],
                    hint='press_stop')
                return
            if not self.zero_offset:
                self._notify_step('warn', 'STATE 1', 'trigger',
                    'Chưa homing — không biết vị trí 0 của servo',
                    action=['Nhấn HOMING trên GUI', 'Đợi homing xong rồi STATE 1'],
                    hint='press_homing')
                return
            if self._motion_busy:
                self.get_logger().warn("Robot đang báo bận (topic motion_busy), vẫn cho phép chạy MANUAL...")
                self._notify('info', 'Robot busy', 'Đang bỏ qua khóa an toàn trong MANUAL mode...')
            # Kiểm tra điều kiện cảm biến THẬT (IO module)
            has_tray    = self.sensor(S1_BELT_START) or self.sensor(S2_BELT_MID) or self.sensor(S3_BELT_END)
            place_ok    = not self.sensor(S7_TRAY_AT_ROBOT)
            cyl1_ret_ok = self.sensor(S9_CYL1_RETRACTED)
            cyl1_ext_ok = not self.sensor(S10_CYL1_EXTENDED)

            if not (has_tray and place_ok and cyl1_ret_ok and cyl1_ext_ok):
                reasons = []
                check = []
                action = []
                if not has_tray:
                    reasons.append(f"{self._sensor_label(1)}/{self._sensor_label(2)}/{self._sensor_label(3)} OFF (belt rỗng)")
                    check.append('khay trên belt input')
                    action.append('Đặt khay lên belt')
                if not place_ok:
                    reasons.append(f"{self._sensor_label(7)} ON (đã có khay ở Robot)")
                    check.append(f'{self._sensor_label(7)} có nhiễu không')
                    action.append('Đợi robot lấy khay xong / chạy STATE 2 trước')
                if not cyl1_ret_ok:
                    reasons.append(f"{self._sensor_label(9)} OFF (Cyl1 chưa retract)")
                    check.append('van Cyl1, áp khí, dây S9')
                    action.append('JOG → Cyl1 RETRACT manual')
                if not cyl1_ext_ok:
                    reasons.append(f"{self._sensor_label(10)} ON (Cyl1 đang extend)")
                    check.append('van Cyl1, dây S10')
                    action.append('JOG → Cyl1 RETRACT manual')
                self._notify_step('warn', 'STATE 1', 'pre-flight',
                    'Điều kiện chưa đủ: ' + '; '.join(reasons),
                    check=check, action=action)
                return
            self._jog_mode = False
            self._cmd_sent_in = False
            self._step_timeout_in = 0.0
            self._sync_mode_jog()
            self._notify('info', 'STATE 1 (manual)', 'Bắt đầu cấp khay')
            self._enter_in(SystemState.S1_CONFIRM_SAFE)
            return

        # ── STATE 2: trong MANUAL operator có thể kích trực tiếp ─
        if cmd in ('STATE2', 'STATE_2', 'S2', 'STATE2A', 'S2A'):
            if self.operation_mode != 'manual':
                self._notify_step('warn', 'STATE 2', 'trigger',
                    'Chỉ kích thủ công ở MANUAL mode',
                    action=['Chuyển GUI sang MANUAL', 'Nhấn STATE 2 lại'])
                return
            if self.state_in not in (SystemState.IDLE, SystemState.S1_COMPLETE):
                self._notify_step('warn', 'STATE 2', 'trigger',
                    f'{self.state_in.name} đang chạy — không thể vào STATE 2',
                    enum_name=self.state_in.name,
                    action=['Nhấn STOP', 'Đợi state hiện tại kết thúc rồi retry'],
                    hint='press_stop')
                return
            if not self.zero_offset:
                self._notify_step('warn', 'STATE 2', 'trigger',
                    'Chưa homing — không biết vị trí 0 của servo',
                    action=['Nhấn HOMING trên GUI', 'Đợi homing xong rồi STATE 2'],
                    hint='press_homing')
                return
            if not self.sensor(S7_TRAY_AT_ROBOT):
                self._notify_step('warn', 'STATE 2', 'pre-flight',
                    f'{self._sensor_label(7)} OFF — chưa có khay ở vị trí robot',
                    check=[f'khay đã được robot đưa về Pos1 chưa', f'{self._sensor_label(7)} có nhiễu / lệch không'],
                    action=['Đợi robot đưa khay về', 'Hoặc đặt khay manual vào vị trí Pos1', 'Sau đó STATE 2 lại'])
                return
            self._input_tray_done = False
            self._jog_mode = False
            self._drive_warm_t = -1.0  # ép drive warm-up flush trước motion đầu tiên (FAS quirk)
            self._sync_mode_jog()
            self._notify('info', 'STATE 2A (manual)', 'S7 ON — bắt đầu lấy khay')
            self._enter_in(SystemState.S2A_CHECK_INTERLOCK)
            return

        # ── STATE 3: chỉ cho MANUAL, operator nhấn nút (AUTO tự trigger qua _can_start_s3) ──────────
        if cmd in ('STATE3', 'STATE_3', 'S3'):
            if not self._conf('output_stack_present', True):
                self._notify_step('warn', 'STATE 3', 'trigger',
                    'STATE 3 disabled — cụm Servo3/OutX/OutY/CPX 254 chưa lắp',
                    check=['YAML cartridge_config.yaml output_stack_present flag'],
                    action=['Khi lắp xong Pos2: đổi output_stack_present: false → true', 'Restart node'])
                return
            if self.operation_mode != 'manual':
                self._notify_step('warn', 'STATE 3', 'trigger',
                    'Chỉ kích thủ công ở MANUAL mode',
                    action=['Chuyển GUI sang MANUAL', 'Nhấn STATE 3 lại'])
                return
            if self.state_s3 not in (SystemState.IDLE,):
                self._notify_step('warn', 'STATE 3', 'trigger',
                    f'{self.state_s3.name} đang chạy — không thể vào STATE 3',
                    enum_name=self.state_s3.name,
                    action=['Nhấn STOP', 'Đợi state hiện tại kết thúc rồi retry'],
                    hint='press_stop')
                return
            if not self.zero_offset:
                self._notify_step('warn', 'STATE 3', 'trigger',
                    'Chưa homing — không biết vị trí 0 của servo',
                    action=['Nhấn HOMING trên GUI', 'Đợi homing xong rồi STATE 3'],
                    hint='press_homing')
                return
            # Check sensor THẬT — STATE3 chỉ chạy khi vị trí cấp output đang trống
            if self.sensor(S18_FEED_OK):
                self._notify_step('warn', 'STATE 3', 'pre-flight',
                    f'{self._sensor_label(18)} ON — vị trí cấp output đang có khay, cần trống',
                    check=[f'khay ở Platform có thật không', f'{self._sensor_label(18)} có nhiễu không'],
                    action=['Đợi robot lấy khay khỏi Platform', 'Hoặc bốc khay manual', 'Sau đó STATE 3 lại'])
                return
            self._jog_mode = False
            self._drive_warm_t = -1.0  # FAS drive warm-up
            self._sync_mode_jog()
            self._s3_force_home = False   # fresh trigger: S17 ON thì giữ vị trí, đẩy tiếp
            self._s3_finish_after_home_check = False
            self._s3_output_loaded = False
            self._notify('info', 'STATE 3 (manual)', 'Bắt đầu cấp khay output')
            self._enter_s3(SystemState.S3_CHECK_OUTXY_SAFE)
            return

        # ── STATE 4: chỉ cho MANUAL, operator nhấn nút (AUTO tự trigger khi _s4_trigger=True) ──────────
        if cmd in ('STATE4', 'STATE_4', 'S4'):
            if not self._conf('output_stack_present', True):
                self._notify_step('warn', 'STATE 4', 'trigger',
                    'STATE 4 disabled — cụm OutX/OutY/CPX 254 chưa lắp',
                    check=['YAML cartridge_config.yaml output_stack_present flag'],
                    action=['Khi lắp xong Pos2: đổi output_stack_present: false → true', 'Restart node'])
                return
            if self.operation_mode != 'manual':
                self._notify_step('warn', 'STATE 4', 'trigger',
                    'Chỉ kích thủ công ở MANUAL mode (AUTO trigger qua /robot/done_tray_output)',
                    action=['Chuyển GUI sang MANUAL', 'Nhấn STATE 4 lại'])
                return
            if self.state_s4 not in (SystemState.IDLE,):
                self._notify_step('warn', 'STATE 4', 'trigger',
                    f'{self.state_s4.name} đang chạy — không thể vào STATE 4',
                    enum_name=self.state_s4.name,
                    action=['Nhấn STOP', 'Đợi state hiện tại kết thúc rồi retry'],
                    hint='press_stop')
                return
            if not self.zero_offset:
                self._notify_step('warn', 'STATE 4', 'trigger',
                    'Chưa homing — không biết vị trí 0 của servo',
                    action=['Nhấn HOMING trên GUI', 'Đợi homing xong rồi STATE 4'],
                    hint='press_homing')
                return
            # Pre-flight sensor check (mirror AUTO trigger):
            #   S18 ON — phải có khay ở vị trí feed cần thay.
            # Cyl2 không reject ở đây: STATE 4 sẽ tự retract và chờ S21 ON/S22 OFF
            # ở bước đầu tiên trước khi OutY/OutX di chuyển.
            s18 = self.sensor(S18_FEED_OK)
            if not s18:
                reasons, check, action = [], [], []
                reasons.append(f"{self._sensor_label(18)} OFF (không có khay ở feed)")
                check.append('khay output có thật ở vị trí feed không')
                action.append('Chạy STATE 3 cấp khay trước')
                self._notify_step('warn', 'STATE 4', 'pre-flight',
                    'Điều kiện chưa đủ: ' + '; '.join(reasons),
                    check=check, action=action)
                return
            self._jog_mode = False
            self._drive_warm_t = -1.0  # FAS drive warm-up
            self._sync_mode_jog()
            self._notify('info', 'STATE 4 (manual)', 'Thay khay output')
            self._enter_s4(SystemState.S4_CYL2_RETRACT_READY)
            return

        if cmd == 'ABORT_TO_JOG':
            self._s1_abort("Operator stopped state (ABORT_TO_JOG)")
            return

        self._notify('warn', f'goto_state: khong biet "{cmd}"', '')

    def _manual_state_cmd_key(self, cmd: str) -> str:
        if cmd in ('STATE1', 'STATE_1', 'S1'):
            return 'STATE1'
        if cmd in ('STATE2', 'STATE_2', 'S2', 'STATE2A', 'S2A'):
            return 'STATE2'
        if cmd in ('STATE3', 'STATE_3', 'S3'):
            return 'STATE3'
        if cmd in ('STATE4', 'STATE_4', 'S4'):
            return 'STATE4'
        if cmd == 'HOMING':
            return 'HOMING'
        return ''

    def _drop_duplicate_manual_state_cmd(self, cmd: str) -> bool:
        """
        Chặn double-click/nhiều message liên tiếp từ GUI. Đây chỉ là debounce
        cho lệnh chọn state; STOP/ABORT/IDLE không bị chặn vì là lệnh an toàn.
        """
        key = self._manual_state_cmd_key(cmd)
        if not key:
            return False
        now = time.monotonic()
        with self._manual_state_cmd_lock:
            if key == self._last_manual_state_cmd and now - self._last_manual_state_cmd_t < 0.8:
                self.get_logger().warn(f"[GOTO_STATE] Drop duplicate {key} command")
                return True
            self._last_manual_state_cmd = key
            self._last_manual_state_cmd_t = now
        return False

    def _s1_abort(self, reason: str):
        """
        Hủy bỏ STATE đang chạy và đưa hệ thống về IDLE an toàn.
        Dùng khi: operator nhấn ABORT_TO_JOG, timeout bước, hoặc lỗi cảm biến.
        Dừng tất cả servo, reset mọi cờ state, bật JOG sẵn sàng.
        KHÔNG retract Cyl1/Cyl2 — giữ nguyên valve state trên CPX
        (tránh thả khay đang kẹp; operator can thiệp thủ công).
        Sau abort chuyển sang MANUAL/JOG để đồng bộ với robot manual và cho phép jog tay.
        """
        self._pub_cartridge_busy(False)
        self._pub_cartridge_pos2_busy(False)
        self._s1_scan_noise_retry = 0
        self._s4_prev_in         = False
        self.get_logger().error(f"S1 ABORT: {reason}")
        self._notify('error', 'Hủy State', reason)
        self._stop(1); self._stop(2); self._stop(3); self._stop(4); self._stop(5)
        self.operation_mode = 'manual'
        self._jog_mode = True
        robot_msg = Int32()
        robot_msg.data = 3
        self.pub_robot_mode.publish(robot_msg)
        self._state1_enabled = False
        self._drain_armed_after_s2 = False
        self._publish_drain(False)
        self._s5_retry = 0; self._s1_retry_count = 0; self._cmd_sent_in = False
        self._input_tray_done = False
        self._s2_output_full_accept = False
        self._s4_trigger = False
        self._enter_in(SystemState.IDLE)
        self._enter_s3(SystemState.IDLE)
        self._enter_s4(SystemState.IDLE)
        self._enter(SystemState.IDLE)
        self._sync_mode_jog()

    def _safety_check_loop(self):
        """
        Luồng kiểm tra an toàn chạy ngầm liên tục (~10Hz).
        Theo dõi tọa độ thực tế của INX và INY để phát hiện sớm các va chạm
        tiềm ẩn, bất kể state machine đang ở trạng thái nào.
        Nếu phát hiện vi phạm Danger Zone hoặc mất kết nối Modbus đột ngột,
        dừng khẩn cấp và chuyển hệ thống sang ERROR.
        """
        while getattr(self, '_system_running', False):
            try:
                time.sleep(0.1)
                
                # Không kiểm tra nếu chưa homed
                if 1 not in self.zero_offset or 2 not in self.zero_offset:
                    continue
                    
                # Đọc vị trí
                inx = self._pos(1)
                iny = self._pos(2)
                
                if inx is None or iny is None:
                    continue
                    
                self._last_inx = inx
                self._last_iny = iny
                    
            except Exception as e:
                self.get_logger().error(f"Safety Loop Error: {e}")
                time.sleep(1.0)

    def _cb_update_config(self, msg: String):
        """
        Cập nhật config runtime từ GUI qua /providesystem/update_config (JSON).
        Payload: {"key": "tên_config", "data": "giá_trị"}
        Hỗ trợ cập nhật số, string, dict (row positions) mà không cần restart node.
        Sau khi cập nhật: lưu vào file YAML (config.save_to_file()).
        Ví dụ: thay đổi tốc độ scan, vị trí row, timeout — có hiệu lực ngay.
        """
        try:
            import json
            self._reload_config_from_disk("before GUI update")
            payload = json.loads(msg.data)
            key = payload.get('key')
            val_str = payload.get('data', '')
            if key and hasattr(self.config, key):
                try:
                    val = json.loads(val_str)
                except ValueError:
                    val = val_str
                if isinstance(val, dict):
                    def parse_val(v):
                        if isinstance(v, list):
                            return [float(x) for x in v]
                        return float(v)
                    val = {int(k) if str(k).isdigit() else k: parse_val(v) for k, v in val.items()}
                elif isinstance(val, str):
                    try: val = float(val)
                    except ValueError: pass
                setattr(self.config, key, val)
                self.config.save_to_file()
                self._config_mtime_ns = self._config_file_mtime_ns()
                self._publish_config_snapshot()
                self.get_logger().info(f"Config updated dynamically: {key} = {val}")
                self._notify('info', 'Config Updated', f'Cập nhật thành công {key}')
            else:
                self.get_logger().warn(f"Config key not found or ignored: {key}")
        except Exception as e:
            self.get_logger().warn(f"update_config error: {e}")

    def _cb_get_config(self, msg: String):
        """
        Phục vụ yêu cầu đọc config từ GUI qua /providesystem/get_config.
        Khi nhận "request": đọc file YAML config, chuyển sang JSON và publish
        lên /providesystem/config_data (latching QoS — GUI mới join vẫn nhận được).
        GUI dùng để hiển thị/chỉnh sửa config trực tiếp trên màn hình.
        """
        if msg.data.strip() == 'request':
            try:
                self._reload_config_from_disk("GUI request")
                self._publish_config_snapshot()
            except Exception as e:
                self.get_logger().warn(f"get_config error: {e}")

    def _cb_set_target_row(self, msg: String):
        """
        Đặt row đích thủ công từ GUI qua /providesystem/set_target_row.
        Ghi đè _current_row để STATE 1 di chuyển InY đến row được chỉ định
        thay vì row được tính tự động từ sensor scan.
        Dùng để debug hoặc vận hành bán tự động khi cần override.
        """
        try:
            row = int(msg.data.strip())
            valid_rows = list(self.config.iny_input_zones.keys()) if hasattr(self.config, 'iny_input_zones') else list(range(1, 6))
            if row in valid_rows:
                self._current_row = row
                self.get_logger().info(f'[SET_TARGET_ROW] row → {row}')
            else:
                self.get_logger().warn(f'[SET_TARGET_ROW] invalid row: {row} (valid: {valid_rows})')
        except ValueError:
            self.get_logger().warn(f'[SET_TARGET_ROW] non-integer value: "{msg.data}"')

    # ══════════════════════════════════════════════════════════════
    # Control Loop
    # ══════════════════════════════════════════════════════════════


    def _cyl3_monitor(self):
        """
        Đối chiếu lệnh cyl3 gần nhất (_cyl3_expected) với feedback S15/S16.
        - Log INFO 1 lần khi sensor confirm đúng kỳ vọng (S16 ON sau extend / S15 ON sau retract).
        - Log WARN 1 lần nếu sau 2s vẫn mismatch (sensor không lên / lên nhầm).
        Chỉ monitor, không block state machine.
        """
        if self._cyl3_expected is None:
            return
        s15, s16 = self._snap(S15_CYL3_RETRACTED, S16_CYL3_EXTENDED)
        elapsed = time.time() - self._cyl3_cmd_time
        if self._cyl3_expected == "extended":
            confirmed = s16 and not s15
        else:  # "retracted"
            confirmed = s15 and not s16
        if confirmed:
            if not self._cyl3_confirmed_logged:
                self.get_logger().info(
                    f"[CYL3] confirmed {self._cyl3_expected.upper()} sau {elapsed:.2f}s"
                )
                self._cyl3_confirmed_logged = True
            self._cyl3_mismatch_warned = False
        elif elapsed >= 2.0 and not self._cyl3_mismatch_warned:
            self.get_logger().warn(
                f"[CYL3] expect {self._cyl3_expected.upper()} nhưng S15={int(s15)} S16={int(s16)} "
                f"sau {elapsed:.2f}s — kiểm tra cảm biến / khí nén"
            )
            self._cyl3_mismatch_warned = True

    def _check_row1_interlock(self):
        """
        Pre-check trước khi đặt khay tại row1 output pos1 (chỉ áp dụng row1 — các
        row khác ở cao hơn, không va với Cyl3).
        HARD blocks:
          1. S15 OFF: Cyl3 chưa retract → tray sẽ va vào piston Cyl3.
          2. S16 ON + S6 OFF: Cyl3 báo extended nhưng không có khay → cơ khí lỗi.
        Exception (vẫn cho qua): S4 scan không trigger nhưng S6 ON (S6 nhận nhầm/
        nhiễu) → fallback row1. Case này pass tự nhiên vì sau scan Cyl3 vẫn retract
        (S16 OFF) nên block #2 không kích hoạt.
        Returns: (ok: bool, reason: str)
        """
        if not self._conf('cyl3_present', True):
            return True, ""
        s15, s16 = self._snap(S15_CYL3_RETRACTED, S16_CYL3_EXTENDED)
        if not s15:
            return False, "S15 OFF (Cyl3 chưa retract)"
        if s16 and not self.sensor(S6_CHECK_TRAY_P1):
            return False, "S16 ON + S6 OFF (Cyl3 extended nhưng không có khay)"
        return True, ""

    def _cyl3_safety_check(self):
        """
        Đồng bộ Cyl3 với cảm biến khay theo policy `S6 ON ↔ S16 ON`:
          • S6 ON  → ép Cyl3 EXTEND (giữ khay)
          • S6 OFF → ép Cyl3 RETRACT (chừa chỗ)

        Chạy mỗi tick (50ms). Edge-trigger log + refresh lệnh 1s khi sensor + actuator
        chưa khớp (S6 ON nhưng S16 OFF, hoặc S6 OFF nhưng S15 OFF). Khi đã khớp
        (S6 ON+S16 ON hoặc S6 OFF+S15 ON) → không gửi lệnh nữa, chỉ log 1 lần.

        LOCK-OUT trong STATE 2A:
        Khi state_in ∈ S2A_*, sync này bị disable — state machine quản lý Cyl3
        theo workflow (extend ở A9b, không retract giữa chừng). Operator không lấy
        được khay output giữa chu trình STATE 2.
        Ngoài STATE 2A (IDLE/S1_*/STATE 3/4) sync hoạt động liên tục.

        _cyl3_safety_active: '' | 'extending' | 'extended' | 'retracting' | 'retracted'
        """
        # [REMOVED CONTINUOUS S6 MIRROR PER USER REQUEST]
        # Cyl3 does not check S6 continuously. It only checks when State 2 is Triggered.
        pass

    def _cyl5_follow_s19(self):
        """
        Giữ quan hệ S19 ↔ Cylinder 5 cả ngoài bước S4_CYL5_SYNC:
          S19 ON  → S28 ON + S27 OFF
          S19 OFF → S27 ON + S28 OFF
        State 4 tự quản lý Cyl5 theo từng bước riêng nên helper này bỏ qua toàn bộ
        lúc state_s4 đang chạy, tránh ghi đè lệnh giữ/nhả khay cuối chu trình.
        """
        if not self._conf('output_stack_present', True):
            return
        # Manual/JOG control cylinder phải giữ theo lệnh operator, không bị S19
        # auto-follow ghi đè ngay sau khi bấm EXTEND/RETRACT.
        if self.operation_mode == 'manual':
            return
        if self.state_s4 != SystemState.IDLE:
            return

        s19 = self.sensor(S19_CHECK_TRAY_P2)
        cyl5_ret, cyl5_ext = self._snap(S27_CYL5_RETRACTED, S28_CYL5_EXTENDED)
        matched = (cyl5_ext and not cyl5_ret) if s19 else (cyl5_ret and not cyl5_ext)
        if matched:
            return

        now = time.time()
        if now - self._cyl5_retry_t < 1.0:
            return
        if s19:
            self._cyl5_extend()
        else:
            self._cyl5_retract()
        self._cyl5_retry_t = now

    def _config_file_mtime_ns(self) -> int:
        path = getattr(self.config, '_config_file', None)
        if not path:
            return 0
        try:
            return os.stat(path).st_mtime_ns
        except OSError:
            return 0

    def _reload_config_from_disk(self, reason: str = "") -> bool:
        """
        Đồng bộ runtime config từ YAML trên đĩa.
        GUI chỉ là view/editor; YAML là nguồn mới nhất khi operator sửa tay.
        """
        path = getattr(self.config, '_config_file', None)
        if not path:
            return False
        try:
            self.config = CartridgeConfig.load(path)
            self._config_mtime_ns = self._config_file_mtime_ns()
            if reason:
                self.get_logger().info(f"[CONFIG] Reloaded YAML ({reason})")
            return True
        except Exception as e:
            self.get_logger().warn(f"[CONFIG] Reload YAML failed ({reason}): {e}")
            return False

    def _publish_config_snapshot(self):
        try:
            if hasattr(self.config, "model_dump"):
                data = self.config.model_dump(exclude_unset=True)
            else:
                data = self.config.dict(exclude_unset=True)
            msg_out = String()
            msg_out.data = json.dumps(data)
            self.pub_config_data.publish(msg_out)
        except Exception as e:
            self.get_logger().warn(f"[CONFIG] publish snapshot error: {e}")

    def _config_file_watchdog(self):
        mtime_ns = self._config_file_mtime_ns()
        if not mtime_ns:
            return
        if mtime_ns != getattr(self, '_config_mtime_ns', 0):
            if self._reload_config_from_disk("file changed"):
                self._publish_config_snapshot()

    def _control_loop(self):
        """
        Vòng lặp điều khiển chính, chạy mỗi 50ms (20Hz) qua ROS timer.
        Thực hiện theo thứ tự:
          1. Cập nhật watchdog tick
          2. Kiểm tra robot heartbeat (timeout 5s → tự xả interlock)
          3. Tracking S18 → ghi timestamp khi S18 OFF (dùng cho auto-trigger S3)
          4. Gọi _process_state() → dispatcher toàn bộ state machine
          5. Publish system state, sensor states
          6. Debounce và publish input_trays_empty (sau 20 tick liên tiếp = 1s)
        Bắt exception trong try/except riêng, chuyển sang ERROR state nếu có lỗi.
        """
        try:
            self._watchdog_last_tick = time.time()
            if self._system_paused:
                return
            # Cyl3 safety watchdog: S13 và S14 cùng OFF -> ép Cyl3 RETRACT và LATCH.
            # Chạy ở MỌI chế độ (auto/ai/manual) và không phụ thuộc _system_running:
            # không có khay ở cả 2 vị trí output thì piston Cyl3 không được thò ra,
            # bất kể operator đang chạy chu trình hay chỉ bật node. Sensor cache do
            # thread _io_bg_loop nuôi độc lập nên đọc luôn tươi.
            # Latch chỉ nhả khi manual GUI extend hoặc STATE 2 rerun recheck S6 ON.
            if self._conf('cyl3_present', True):
                s13 = self.sensor(S13_OUT1_TRAYPOS1)
                s14 = self.sensor(S14_OUT2_TRAYPOS1)
                if not s13 and not s14:
                    self._cyl3_s13s14_latch = True
                    s15 = self.sensor(S15_CYL3_RETRACTED)
                    now = time.time()
                    if not s15 or self._cyl3_expected != "retracted":
                        if now - getattr(self, '_cyl3_watchdog_last_fire', 0.0) >= 1.0:
                            self._cyl3_retract()
                            self._cyl3_watchdog_last_fire = now
                            self.get_logger().info("[CYL3-WATCHDOG] S13+S14 OFF but Cyl3 not retracted → Force Cyl3 RETRACT + LATCH (giữ đến khi manual GUI extend hoặc STATE 2 rerun recheck S6)")

            self._cyl3_safety_check()
            self._cyl3_monitor()
            self._cyl5_follow_s19()

            # S18 tracking logic for S3 auto-feed delay
            feed_ok = self.sensor(S18_FEED_OK)
            if not feed_ok and self._s10_prev:
                self._s10_off_time = time.time()
                self.get_logger().info("[S18] Bị lấy đi -> bắt đầu đếm ngược 5s để cấp tiếp")
            self._s10_prev = feed_ok
            
            # ── Robot heartbeat timeout ──
            # Nếu robot node chưa bao giờ publish hoặc đã ngắt > 5s → tự xả khóa.
            # FREEZE khi _system_paused: operator có thể pause lâu để fix vật lý,
            # không được tự release lock vì "heartbeat chậm".
            if not self._system_paused:
                if self._robot_connected and self._robot_last_seen > 0:
                    if time.time() - self._robot_last_seen > 5.0:
                        self._robot_connected = False
                        self._motion_busy = False
                        self.get_logger().warn("[ROBOT] Robot node timeout (>5s) — interlock RELEASED, cartridge runs standalone")
                elif not self._robot_connected and self._motion_busy:
                    self._motion_busy = False


            self._process_state()
            self._sync_combined_cartridge_busy()
            self._publish_state()
            self._publish_sensors()
            
            # Publish input_trays_empty state to Dobot Robot Node for sensor display/status.
            b_msg = Bool()
            raw_empty = self._input_belt_empty()
            
            if raw_empty:
                self._input_trays_empty_debounce_count += 1
            else:
                self._input_trays_empty_debounce_count = 0
                
            #... rest of control loop
            empty_debounced = bool(self._input_trays_empty_debounce_count >= 20)
            b_msg.data = empty_debounced
            self.pub_input_trays_empty.publish(b_msg)

            drain_active = (
                self.operation_mode in ['auto', 'ai']
                and getattr(self, '_system_running', False)
                and self.state_in == SystemState.IDLE
                and self._state1_enabled
                and self._drain_armed_after_s2
                and empty_debounced
            )
            self._publish_drain(drain_active)
            
        except Exception as e:
            tb = traceback.format_exc()
            self.get_logger().error(f"CRITICAL LOGIC ERROR (Control Loop Killed): {e}\n{tb}")
            self._notify_step('error', 'SYSTEM', 'control_loop crash',
                f'Python exception trong vòng lặp 20Hz: {str(e)[:80]}',
                check=['log cartridge_node.log cho full traceback',
                       f'state hiện tại: global={self.state.name} in={self.state_in.name}'],
                action=['Hệ thống tự chuyển ERROR state',
                        'STOP để reset',
                        'Restart node nếu lặp lại',
                        'Báo dev với traceback từ log'])
            # Transition to ERROR state to ensure safe stop
            if self.state != SystemState.ERROR:
                self._enter(SystemState.ERROR)
            time.sleep(1.0) # Throttle error loop if it keeps failing

    def _publish_sensors(self):
        """Publish chuỗi nhị phân 28 ký tự lên /providesystem/sensors_state.
        Mỗi ký tự = trạng thái cảm biến S1..S28 ('1'=ON, '0'=OFF)."""
        states = "".join("1" if self.sensor(i) else "0" for i in range(1, 29))
        msg = String(); msg.data = states; self.pub_sensors.publish(msg)

    def _publish_state(self):
        """Publish trạng thái state machine dạng chuỗi "<global>|<input>|<s3_or_s4>"
        lên /system_state. GUI parse để hiển thị badge trạng thái theo thời gian thực."""
        s_out = self.state_s4.value if self.state_s4 != SystemState.IDLE else self.state_s3.value
        combined = f"{self.state.value}|{self.state_in.value}|{s_out}"
        msg = String(); msg.data = combined; self.pub_state.publish(msg)

    def _publish_homing_done(self, done: bool):
        """Publish trạng thái homing (latching) lên /cartridge/homing_done.
        Subscriber chính là vfd_logic_node: gate AUTO/AI để băng tải chỉ chạy
        sau khi user nhấn START → homing xong. Gọi True khi homing thành công,
        False khi reset (init, clear zero_offset do STOP/mode-switch/ERROR)."""
        if getattr(self, 'pub_homing_done', None) is None:
            return  # init order guard — gọi từ __init__ trước khi publisher tạo xong
        msg = Bool(); msg.data = bool(done); self.pub_homing_done.publish(msg)

    def _positions_bg_loop(self):
        """
        Thread nền publish vị trí servo mỗi 50ms (20Hz) lên /providesystem/servo_positions.
        Chạy tách biệt khỏi control loop để Modbus read latency không block 20Hz timer.
        Dừng khi _pos_thread_stop=True (set bởi destroy_node).
        """
        while rclpy.ok() and not getattr(self, '_pos_thread_stop', False):
            try:
                self._publish_positions()
            except Exception as e:
                self.get_logger().warn(f"[pos_bg] {e}")
            time.sleep(0.05)

    def _publish_positions(self):
        """Đọc vị trí tất cả servo và publish JSON lên /providesystem/servo_positions.
        JSON chứa {sid: mm, _jog_vel: m/s, _fas_vel: {sid: m/s}}.
        GUI dùng để hiển thị vị trí live và tốc độ JOG hiện tại.

        Tối ưu hot path:
        1. Per-servo: các trục trong _live_position_servos luôn đọc actual
           position. Trục khác nếu time.time() - _servo_motion_t[sid] >
           _idle_skip_modbus_s (mặc định 2s) → servo idle lâu → SKIP Modbus,
           reuse cache. Giảm lock contention khi không có hoạt động.
        2. Khi servo active (JOG/move/home) → đọc tươi Modbus mỗi cycle.
        3. Try-acquire 50ms — nếu lock vẫn bận (state machine giữ) → fallback cache.

        Mục đích: KHÔNG bao giờ chặn JOG vì publish positions, và tự động giảm
        Modbus traffic khi hệ thống idle để JOG cold press phản hồi tức thì.
        """
        try:
            pos = {}
            now = time.time()
            idle_skip = self._idle_skip_modbus_s
            jog_active = self._jog_active
            for sid, mot in self.servos.items():
                cached = self._pos_cache.get(sid)
                # ── (1) Idle skip: servo không có motion command gần đây → reuse cache
                # Bỏ qua nếu servo đang JOG continuous (duration=0.0) — drive di
                # chuyển liên tục mà không có loop refresh _servo_motion_t.
                last_motion = self._servo_motion_t.get(sid, 0.0)
                live_position = sid in getattr(self, '_live_position_servos', set())
                if (not live_position
                        and sid not in jog_active
                        and now - last_motion > idle_skip
                        and cached is not None):
                    pos[str(sid)] = cached
                    continue
                # ── (2/3) Đọc tươi (try-acquire) hoặc fallback cache nếu lock busy
                if not self._servo_lock.acquire(timeout=0.05):
                    if cached is not None:
                        pos[str(sid)] = cached
                    continue
                try:
                    counts = mot.current_position()
                    p = (counts - self.zero_offset.get(sid, 0)) / COUNTS_PER_MM
                except Exception:
                    p = cached
                finally:
                    self._servo_lock.release()
                if p is not None:
                    val = round(p, 2)
                    pos[str(sid)] = val
                    self._pos_cache[sid] = val
            data = pos.copy()
            data['_jog_vel'] = self._jog_velocity_ms
            if self._fas_jog_vel:
                data['_fas_vel'] = {str(k): round(v, 4) for k, v in self._fas_jog_vel.items()}
            msg = String()
            msg.data = json.dumps(data)
            self.pub_servo_pos.publish(msg)
        except Exception:
            pass

    def _watchdog(self):
        """
        Giám sát độ trễ vòng lặp điều khiển (gọi mỗi 5s qua ROS timer riêng).
        Nếu control loop không tick trong >3 giây → cảnh báo log + GUI notify.
        Phát hiện tình trạng loop bị block do Modbus timeout hoặc deadlock.
        """
        gap = time.time() - self._watchdog_last_tick
        if gap > 3.0:
            self.get_logger().error(f"WATCHDOG: loop silent {gap:.1f}s!")
            self._notify_step('error', 'SYSTEM', 'watchdog',
                f'Control loop không tick suốt {gap:.1f}s (mong đợi 50ms/tick)',
                check=['Modbus có timeout/hang không (network spike, drive offline)',
                       'thread trong code có stuck không',
                       'log có warning Modbus retry không'],
                action=['Theo dõi log, đợi loop tự khôi phục',
                        'Nếu lặp lại: STOP + restart node'])

    # ══════════════════════════════════════════════════════════════
    # State Dispatcher
    # ══════════════════════════════════════════════════════════════

    def _process_state(self):
        """
        Điểm vào của toàn bộ state machine, gọi mỗi tick từ _control_loop().
        Ưu tiên xử lý:
          1. HOMING / HOMING_RUNNING (global state)
          2. ERROR (global state)
          3. Nếu bình thường → gọi song song 3 dispatcher:
               _dispatch_input() — STATE 1 + STATE 2 (input side)
               _dispatch_s3()    — STATE 3 (platform/servo3 feed)
               _dispatch_s4()    — STATE 4 (output side thay khay)
        Ba dispatcher chạy độc lập, tránh xung đột qua interlock cảm biến.
        """
        s = self.state
        # Graceful PAUSE: nếu user đã yêu cầu pause và state hiện tại đã hoàn tất
        # (mọi sub-chain về IDLE và không trong HOMING), promote ngay tại đây
        # TRƯỚC khi dispatcher kéo chain mới từ IDLE.
        if self._pause_pending and self._can_pause_now():
            self._promote_pause()
            return
        if s == SystemState.HOMING:           self._do_homing(); return
        if s == SystemState.HOMING_RUNNING:
            # Check if bg homing thread finished (Bug #6 fix: state transition on main thread)
            if self._homing_done_event.is_set():
                self._homing_done_event.clear()
                if self._homing_result:
                    self.get_logger().info("Homing complete (main thread transition)")
                    self._homing_ref_offset.clear()
                    self._publish_homing_done(True)
                    # Manual mode: KHÔNG auto-set _jog_mode — giữ nguyên label "manual"
                    # để user chủ động chọn STATE chạy. JOG chỉ kích hoạt khi STOP state.
                    if self.operation_mode == 'manual':
                        self._sync_mode_jog()
                        self._notify('info', 'Homing xong', 'Chọn STATE để chạy')
                    else:
                        self._notify('info', 'Homing xong', '')
                    self.state_in  = SystemState.IDLE
                    self.state_s3  = SystemState.IDLE
                    self.state_s4  = SystemState.IDLE
                    self._system_running = True
                    self._enter(SystemState.IDLE)
                else:
                    if self._homing_abort.is_set():
                        self.get_logger().info("Homing cancelled by STOP — về IDLE")
                        self._enter(SystemState.IDLE)
                        # _cb_stop đã set _jog_mode=True nếu user bấm STOP, không cần set lại.
                    else:
                        # Fail không do abort → ERROR (đã transition trong _error)
                        self._error("Homing that bai")
            return
        if s == SystemState.ERROR:            self._do_error(); return
        self._dispatch_input()
        self._dispatch_s3()
        self._dispatch_s4()

    def _dispatch_input(self):
        """
        Dispatcher cho state machine phụ INPUT (state_in).
        Xử lý toàn bộ STATE 1 (cấp khay input từ băng tải vào stack)
        và STATE 2 (rút khay đã dùng ra, đưa sang side output để xử lý tiếp).
        Gọi đúng handler function tương ứng với state hiện tại.
        """
        s = self.state_in
        if   s == SystemState.IDLE:                self._do_idle_input()
        elif s == SystemState.S1_CONFIRM_SAFE:     self._s1_confirm_safe()
        elif s == SystemState.S1_INX_MOVE_POS_PICK:         self._s1_inx_move_pos_pick()
        elif s == SystemState.S1_WAIT_ARRIVE:      self._s1_wait_arrive()
        elif s == SystemState.S1_INY_SCAN:         self._s1_iny_scan()
        elif s == SystemState.S1_WAIT_STOP_S4:     self._s1_wait_stop_s4()
        elif s == SystemState.S1_INY_TO_ROW:       self._s1_iny_to_row()
        elif s == SystemState.S1_CHECK_S5:         self._s1_check_s5()
        elif s == SystemState.S1_FALLBACK_RETRACT: self._s1_fallback_retract()
        elif s == SystemState.S1_FALLBACK_WAIT_INY: self._s1_fallback_wait_iny()
        elif s == SystemState.S1_WAIT_GUI_CONFIRM: self._s1_wait_gui_confirm()
        elif s == SystemState.S1_INX_TRY_POS_PICK:    self._s1_inx_try_pos_pick()
        elif s == SystemState.S1_RETRY_JOG:        self._s1_retry_jog()
        elif s == SystemState.S1_CYL1_EXTEND:      self._s1_cyl1_extend()
        elif s == SystemState.S1_INY_PICK_TRAY_UP:           self._s1_iny_pick_tray_up()
        elif s == SystemState.S1_INY_PLACE_TRAY_ROBOT:          self._s1_iny_place_tray_robot()
        elif s == SystemState.S1_WAIT_RELEASE:     self._s1_wait_release()
        elif s == SystemState.S1_WAIT_S7:         self._s1_wait_s7()
        elif s == SystemState.S1_INX_WAIT_TRAY_DONE:           self._s1_inx_wait_tray_done()
        elif s == SystemState.S1_COMPLETE:         self._s1_complete()
        elif s == SystemState.S1_RETRY_SCAN_HOME:  self._s1_retry_scan_home()
        elif s == SystemState.S2A_CHECK_INTERLOCK:   self._s2a_check_interlock()
        elif s == SystemState.S2A_INX_MOVE_POS_PICK:           self._s2a_inx_move_pos_pick()
        elif s == SystemState.S2A_POS_PLACE_TRAY_ROBOT_CYL1:      self._s2a_pos_place_tray_robot_cyl1()
        elif s == SystemState.S2A_WAIT_OUTPUT_CLEAR:    self._s2a_wait_output_clear()
        elif s == SystemState.S2A_WAIT_CYL_EXT:          self._s2a_wait_cyl_ext()
        elif s == SystemState.S2A_INY_HOME:            self._s2a_iny_home()
        elif s == SystemState.S2A_INX_PLACE_TRAY_OUT_POS1:            self._s2a_inx_place_tray_out_pos1()
        elif s == SystemState.S2A_INY_JOG_OUTPUT:    self._s2a_iny_jog_output()
        elif s == SystemState.S2A_INY_TARGETROW:    self._s2a_iny_targetrow()
        elif s == SystemState.S2A_WAIT_CYL1_RET:          self._s2a_wait_cyl1_ret()
        elif s == SystemState.S2A_CYL3_EXTEND:           self._s2a_cyl3_extend()
        elif s == SystemState.S2A_INY_FINAL:      self._s2a_iny_final()
        elif s == SystemState.S2A_WAIT_NEW_TRAY:            self._s2a_wait_new_tray()
        elif s == SystemState.S2A_RETRY_SCAN_HOME:   self._s2a_retry_scan_home()
        elif s == SystemState.S2A_COMPLETE:          self._s2a_complete()

    def _dispatch_s3(self):
        """
        Dispatcher cho STATE 3: Platform/Servo3 đẩy khay từ stack ra vị trí feed (S18).
        Chạy độc lập với _dispatch_input — có thể chạy xen kẽ STATE 1/2 nếu cần.
        """
        s = self.state_s3
        if   s == SystemState.IDLE:                self._do_idle_s3()
        elif s == SystemState.S3_CHECK_OUTXY_SAFE: self._s3_check_outxy_safe()
        elif s == SystemState.S3_OUTX_WAIT:        self._s3_outx_wait()
        elif s == SystemState.S3_SERVO3_TARGET1:   self._s3_servo3_target1()
        elif s == SystemState.S3_CHECK_S17:        self._s3_check_s17()
        elif s == SystemState.S3_WAIT_S17:         self._s3_wait_s17()
        elif s == SystemState.S3_CYL4_EXTEND:      self._s3_cyl4_extend()
        elif s == SystemState.S3_WAIT_GUI_CONFIRM: self._s3_wait_gui_confirm()
        elif s == SystemState.S3_SERVO3_FEED:      self._s3_servo3_feed()
        elif s == SystemState.S3_WAIT_S18:         self._s3_wait_s18()
        elif s == SystemState.S3_COMPLETE:         self._s3_complete()

    def _dispatch_s4(self):
        """
        Dispatcher cho STATE 4: OutX/OutY thay khay output khi khay đầy.
        Chạy độc lập hoàn toàn với STATE 1/2/3 — dùng servo riêng (OutX S4, OutY S5).
        """
        s = self.state_s4
        if   s == SystemState.IDLE:                self._do_idle_s4()
        elif s == SystemState.S4_CYL2_RETRACT_READY: self._s4_cyl2_retract_ready()
        elif s == SystemState.S4_CHECK_OUTY_SAFE:  self._s4_check_outy_safe()
        elif s == SystemState.S4_OUTX_TARGET2:     self._s4_outx_target2()
        elif s == SystemState.S4_OUTY_PICK:        self._s4_outy_pick()
        elif s == SystemState.S4_CYL2_EXTEND:      self._s4_cyl2_extend()
        elif s == SystemState.S4_CYL4_RETRACT:     self._s4_cyl4_retract()
        elif s == SystemState.S4_OUTY_TARGET1:     self._s4_outy_target1()
        elif s == SystemState.S4_OUTX_TARGET3:     self._s4_outx_target3()
        elif s == SystemState.S4_CHECK_S19:         self._s4_check_s19()
        elif s == SystemState.S4_CYL5_SYNC:        self._s4_cyl5_sync()
        elif s == SystemState.S4_OUTY_ROW1:        self._s4_outy_row1()
        elif s == SystemState.S4_OUTY_SCAN_S20:    self._s4_outy_scan_s20()
        elif s == SystemState.S4_OUTY_DROP:        self._s4_outy_drop()
        elif s == SystemState.S4_CYL2_RETRACT:     self._s4_cyl2_retract_state()
        elif s == SystemState.S4_CYL5_EXTEND_HOLD: self._s4_cyl5_extend_hold()
        elif s == SystemState.S4_OUTY_OUTX_HOME:   self._s4_outy_outx_home()
        elif s == SystemState.S4_CYL5_RETRACT_RELEASE: self._s4_cyl5_retract_release()
        elif s == SystemState.S4_OUTX_HOME:         self._s4_outx_home()
        elif s == SystemState.S4_RETRY_SCAN_HOME:  self._s4_retry_scan_home()
        elif s == SystemState.S4_COMPLETE:         self._s4_complete()

    # ── IDLE ─────────────────────────────────────────────────────

    def _do_idle_input(self):
        """
        Trạng thái IDLE của state machine INPUT — chờ điều kiện để trigger STATE 1 hoặc 2.
        Ưu tiên: STATE 2 (robot đã báo xong) trước STATE 1 (cấp khay mới).
        AUTO/AI: STATE 1 auto-trigger qua _can_start_s1().
        MANUAL: STATE 1 chỉ trigger khi operator nhấn nút (_state1_enabled set bởi GUI).
        Log lý do đang chờ (không có khay / robot bận / vị trí cấp đang có khay).
        """
        if not self.zero_offset or not getattr(self, '_system_running', False):
            if not self.zero_offset:
                self._log_once("IDLE_IN_NOT_HOMED", "IDLE-IN: Chua home")
            return

        # S2A: chỉ auto-trigger ở AUTO/AI mode (_can_start_s2a đã gate operation_mode).
        # Manual mode: operator phải nhấn STATE2 qua GUI (_cb_goto_state route riêng).
        if self._can_start_s2a():
            self._input_tray_done = False
            mode_str = self.operation_mode.upper()
            self.get_logger().info(f"[IN-IDLE] Robot done → STATE 2 ({mode_str})")
            self._enter_in(SystemState.S2A_CHECK_INTERLOCK)
            return

        # ── AUTO/MANUAL: auto-trigger S1 ───────────────
        if self._state1_enabled and self._can_start_s1():
            self.get_logger().info(f"[IN-IDLE] Đủ điều kiện → STATE 1 ({self.operation_mode.upper()})")
            self._guide_logged.discard("IDLE_IN_S7")
            self._drain_armed_after_s2 = False
            self._publish_drain(False)
            self._enter_in(SystemState.S1_CONFIRM_SAFE)
            return
        
        # Log lý do chờ
        if not (self.sensor(S1_BELT_START) or self.sensor(S2_BELT_MID) or self.sensor(S3_BELT_END)):
            self._log_once("IDLE_IN_NO_TRAY", "[IN-IDLE] S1/S2/S3 OFF — hết khay input")
        elif self.sensor(S7_TRAY_AT_ROBOT):
            self._log_once("IDLE_IN_NO_PLACE", "[IN-IDLE] S7 ON — vị trí cấp đang có khay (chờ xử lý xong để chạy State 2)")
        elif self._motion_busy:
            self._log_once("IDLE_IN_BUSY", f"[IN-IDLE] Robot đang báo BẬN (/robot/motion_busy={self._motion_busy}) — chờ")

    def _do_idle_s3(self):
        """Chờ điều kiện để auto-trigger STATE 3: S18 OFF liên tục 5s (vị trí feed trống)."""
        if not self.zero_offset or not getattr(self, '_system_running', False):
            return
        if self._can_start_s3():
            self.get_logger().info("[S3-IDLE] S17 ON + S18 OFF >= 5s → STATE 3")
            self._guide_logged.discard("S3_IDLE_S17_OFF")
            self._s3_force_home = False   # fresh trigger: S17 ON thì giữ vị trí, đẩy tiếp
            self._s3_finish_after_home_check = False
            self._s3_output_loaded = False
            self._enter_s3(SystemState.S3_CHECK_OUTXY_SAFE)
            return

        if (self._conf('output_stack_present', True)
                and self.operation_mode in ['auto', 'ai']
                and not self._motion_busy
                and not self.sensor(S18_FEED_OK)):
            s18_off_duration = 0.0
            if getattr(self, '_s10_off_time', 0) > 0:
                s18_off_duration = time.time() - self._s10_off_time
            else:
                s18_off_duration = 5.0
            if s18_off_duration >= 5.0 and not self.sensor(S17_PLATFORM):
                key = "S3_IDLE_S17_OFF"
                if key not in self._guide_logged:
                    self._guide_logged.add(key)
                    self.get_logger().warn(
                        "[S3-IDLE] S18 empty but S17 OFF — hết khay/chưa cấp khay Platform"
                    )
                    self._notify_step(
                        'warn',
                        'STATE 3',
                        'S17 Platform',
                        f'{self._sensor_label(S17_PLATFORM)} OFF — hết khay hoặc chưa cấp khay lên Platform',
                        check=['khay trên Platform Servo3', self._sensor_label(S17_PLATFORM)],
                        action=['Cấp khay lên Platform', 'Khi S17 ON hệ thống sẽ tự chạy STATE 3']
                    )

    def _do_idle_s4(self):
        """Chờ điều kiện để trigger STATE 4: _s4_trigger=True (robot báo khay output đầy)."""
        if not self.zero_offset or not getattr(self, '_system_running', False):
            return
        if self._can_start_s4():
            self._s4_trigger = False
            self.get_logger().info("[S4-IDLE] Output full → STATE 4")
            self._enter_s4(SystemState.S4_CYL2_RETRACT_READY)

    def _do_homing(self):
        """
        Khởi động quá trình homing trong background thread để không block control loop.
        Chuyển sang HOMING_RUNNING ngay lập tức, rồi spawn thread chạy _home_all().
        Kết quả được báo về main thread qua _homing_done_event (threading.Event)
        để tránh race condition khi mutate state từ thread phụ.
        """
        self._enter(SystemState.HOMING_RUNNING)
        self._homing_done_event.clear()
        # Each run owns an immutable cancellation Event.  STOP sets the current
        # event; a later START creates a different one instead of clear()ing the
        # old event and accidentally reviving the previous homing threads.
        self._homing_generation += 1
        generation = self._homing_generation
        abort_event = threading.Event()
        self._homing_abort = abort_event

        def _bg():
            ok = self._home_all(abort_event)
            if generation != self._homing_generation:
                self.get_logger().info(
                    f"[HOMING] Ignore stale result generation {generation}"
                )
                return
            # Signal result to main thread — do NOT mutate state_in/s3/s4 here
            self._homing_result = ok
            self._homing_done_event.set()
        self._homing_thread = threading.Thread(
            target=_bg, daemon=True, name=f"homing_generation_{generation}")
        self._homing_thread.start()

    def _do_error(self):
        """Trạng thái ERROR: log hướng dẫn phục hồi (STOP → START) mỗi lần vào state."""
        self._log_once("ERROR_STATE", "ERROR — kiem tra loi roi nhan STOP -> START")

    # ══════════════════════════════════════════════════════════════
    # STATE 1: Cap khay Input  (giữ nguyên logic gốc)
    # ══════════════════════════════════════════════════════════════

    def _s1_confirm_safe(self):
        """
        Bước đầu STATE 1: đảm bảo InY về vị trí home (≤ iny_home + 2mm) trước khi
        InX di chuyển ra `inx_target2`. Nếu InY đã ở home → chuyển ngay sang
        S1_INX_MOVE_POS_PICK. Ngược lại ra lệnh `_nb_move(2, iny_home)` rồi chờ arrived.

        Pre-condition: không có sub-state khác đang dùng servo (kiểm tra
        _inx_moving / _iny_moving).
        Next: S1_INX_MOVE_POS_PICK khi InY về home thật sự (verify bằng _at_position).

        Drive warm-up (FAS firmware quirk): sau referencing_task, drive có thể
        accept position_task nhưng không physically move (target_position_reached
        về True sai sớm). Mimic STOP+START fix: gọi stop_motion_task() trên tất
        cả servo để flush internal queue, đợi 1s settle, rồi mới cho luồng tiếp.
        """
        # ── Drive warm-up gate ───────────────────────────────────────
        if self._drive_warm_t < 0.0:
            # Pending → trigger flush
            for sid in list(self.servos):
                self._stop(sid)
            self._drive_warm_t = time.time() + 1.0
            self.get_logger().info("S1: drive warm-up (stop_motion_task + 1s settle)")
            return
        if self._drive_warm_t > 0.0:
            if time.time() < self._drive_warm_t:
                return  # đang chờ settle
            self._drive_warm_t = 0.0  # done
        # ─────────────────────────────────────────────────────────────

        if not self._cmd_sent_in and not self._inx_moving and not self._iny_moving:
            self._pub_cartridge_busy(True)
            if self._input_tray_done:
                self._input_tray_done = False
        iny = self._pos(2)
        if iny is None:
            return
        if iny <= self.config.iny_home + 2.0:
            self._enter_in(SystemState.S1_INX_MOVE_POS_PICK)
        else:
            if not self._cmd_sent_in:
                ok = self._nb_move(2, self.config.iny_home)
                if not ok:
                    self._log_once("S1_INY_HOME_FAIL", "S1_SAFE: INY home fail")
                    return
                self._cmd_sent_in     = True
                self._step_timeout_in = time.time() + self.config.move_timeout
            else:
                if time.time() > self._step_timeout_in:
                    self._cmd_sent_in = False
                # [BLOCKING-FIX] verify _at_position(2, iny_home)
                elif self._arrived(2) and self._at_position(2, self.config.iny_home):
                    self._enter_in(SystemState.S1_INX_MOVE_POS_PICK)

    def _s1_inx_move_pos_pick(self):
        """
        Gửi lệnh InX → inx_target2 (505.5mm). KHÔNG chờ arrived ở đây — chỉ kích lệnh
        và chuyển ngay sang S1_WAIT_ARRIVE (state đó chịu trách nhiệm verify).

        Pre-condition:
          - Phải có khay trên băng tải (S1 ∨ S2 ∨ S3 ON) — nếu không, log + notify GUI.
          - InY phải ≤ iny_safe_zone (interlock: tránh va chạm 2 trục).

        Next: S1_WAIT_ARRIVE (sau khi gửi lệnh InX).
        """
        if not (self.sensor(S1_BELT_START) or self.sensor(S2_BELT_MID) or self.sensor(S3_BELT_END)):
            self._log_once("S1_WAIT_BELT", "Step2: Cho S1/S2/S3")
            self._notify('info', 'Cho khay (S1/S2/S3)', 'Dat khay len bang tai.')
            return
        if not self._iny_safe():
            self._log_once("S1_INY_SAFE", "Step2: INY chua safe")
            return
        if not self._cmd_sent_in:
            ok = self._nb_move(1, self.config.inx_target2, vel=200)
            if not ok:
                self._log_once("S1_INX_FAIL", "S1 Step2: INX fail")
                return
            self._cmd_sent_in    = True
            self._inx_arrived = False
            self._30s_timeout = 0.0
            self._cyl1_warn_t  = 0.0
            self.get_logger().info(f"Step2: INX -> {self.config.inx_target2}mm")
            self._enter_in(SystemState.S1_WAIT_ARRIVE)

    def _s1_wait_arrive(self):
        """
        Chờ InX tới target_2 (505.5mm) THẬT SỰ trước khi cho phép sang S1_INY_SCAN.
        Cơ chế block 2 lớp:
          1. _arrived(1) = drive flag target_position_reached (có thể lên sớm)
          2. _at_position(1, target, position_tolerance=1.0mm) = đọc vị trí encoder
             qua _pos() để xác nhận sai số ≤ 1mm.
        Nếu drive báo arrived nhưng vị trí lệch quá 1mm → resend lệnh + chờ tiếp,
        TUYỆT ĐỐI không cho phép chuyển sang InY scan để tránh va chạm cơ khí.
        """
        if not self._inx_arrived:
            self._inx_arrived = self._arrived(1)
            if not self._inx_arrived:
                self._log_once("S1_INX_MOVING", "INX dang di chuyen")
                return
            # Verify INX actually reached target position (tolerance từ config, mặc định 1mm)
            if not self._at_position(1, self.config.inx_target2):
                inx_curr = self._pos(1)
                tol      = self.config.position_tolerance
                self.get_logger().warn(
                    f"INX arrived=True but pos="
                    f"{inx_curr if inx_curr is None else f'{inx_curr:.2f}'}mm "
                    f"!= target {self.config.inx_target2}mm (tol={tol}mm) — resend"
                )
                self._inx_arrived = False
                self._nb_move(1, self.config.inx_target2, vel=150)
                return
            inx_curr = self._pos(1)
            self._30s_timeout = time.time() + 50.0
            self.get_logger().info(
                f"INX dung tai {inx_curr:.2f}mm (target {self.config.inx_target2}mm, "
                f"tol={self.config.position_tolerance}mm) — check S3+S9 (50s)"
            )

        belt_end, cyl1_ret = self._snap(S3_BELT_END, S9_CYL1_RETRACTED)

        if belt_end and cyl1_ret:
            self.get_logger().info("S3+S9 ON -> scan INY")
            self._s4_armed = False
            self._enter_in(SystemState.S1_INY_SCAN)
            return

        if not belt_end:
            if time.time() > self._30s_timeout:
                self.get_logger().info("S3 khong ON sau 50s — INX ve safe (-60mm), retry")
                self._nb_move(1, self.config.inx_safe)
                self._s1_scan_noise_retry = 0
                self._s4_prev_in = False
                self._enter_in(SystemState.S1_CONFIRM_SAFE)
                return
            remain = self._30s_timeout - time.time()
            self._log_once("S1_WAIT_S3", f"Cho S3 ON (con {remain:.0f}s)")
        else:
            if self._cyl1_warn_t == 0:
                self._cyl1_warn_t = time.time()
            elif time.time() - self._cyl1_warn_t >= 5.0:
                self._notify_step('warn', 'STATE 1', 'wait Cyl1 retract',
                    f'{self._sensor_label(3)} ON (khay đến cuối belt) nhưng {self._sensor_label(9)} OFF >5s',
                    enum_name='S1_*',
                    check=['van Cyl1 retract', 'áp khí', f'dây {self._sensor_label(9)}'],
                    action=['JOG → Cyl1 RETRACT manual', 'Hoặc kiểm tra van solenoid'])
                self._cyl1_warn_t = time.time()
            self._log_once("S1_WAIT_S13", "S3 ON, chờ S9 ON")

    def _s1_iny_scan(self):
        """
        Quét InY từ home → target_scaninp1 (970mm) để phát hiện stack khay đầu vào.
        Logic chống nhiễu S4 (xem RULES.md RULE 3):
          1. S4 ON khi đã armed → xét zone ngay (không phụ thuộc rising edge)
          2. Gate S4 armed theo 2 điều kiện AND, re-evaluate mỗi tick:
             - InX vẫn tại inx_target2 ± position_tolerance (_at_position)
             - InY ∈ [iny_scan_valid_min_mm, iny_scan_valid_max_mm]
          3. Disarm + log warn nếu InX drift khỏi target giữa lúc scan.

        S4 ON hợp lệ → tính row từ _zone_to_row → chuyển S1_INY_TO_ROW.
        S4 ON NGOÀI mọi zone (_zone_to_row → None, trigger rơi khe hở
        giữa 2 row / calib lệch): xử lý như noise — cùng nhánh retry/error
        bên dưới, KHÔNG fallback row nào (sự cố 2026-07-03 @ 804.9mm).
        Hết hành trình mà không có S4 ON hợp lệ:
          - Retry < limit (s1_scan_noise_retry_limit, default 1): → S1_RETRY_SCAN_HOME
          - Hết retry: notify error, về home + InX về inx_noise_recovery_mm, vào IDLE.
        """
        iny = self._pos(2)
        if iny is None:
            return

        row1_zone = self.config.iny_input_zones.get(1, [])
        row1_scan_max = max(row1_zone[0], row1_zone[1]) if len(row1_zone) >= 2 else self.config.target_scaninp1
        scan_target = max(self.config.target_scaninp1, row1_scan_max)

        if not self._cmd_sent_in:
            ok = self._nb_move(2, scan_target,
                               vel=self.config.iny_scan_vel)
            if not ok:
                self._log_once("S1_SCAN_FAIL", "S1 INY scan: nb_move fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
            self._s4_armed        = False
            self._s4_prev_in      = self.sensor(S4_SCAN_STACK_P1)
            self._s1_s4_on_logged = False
            self.get_logger().info(
                f"[S1 SCAN] INY -> {scan_target:.0f}mm "
                f"vel={self.config.iny_scan_vel}"
            )

        # [BLOCKING-FIX D+E] Gate S4 theo 2 điều kiện re-check mỗi tick:
        #   1. InX vẫn TẠI inx_target2 (verify _pos, tolerance position_tolerance)
        #      → tránh nhận S4 nếu InX bị nudge/trôi khỏi 505.5mm giữa lúc scan.
        #   2. InY trong vùng quét hợp lệ:
        #      iny_scan_valid_min_mm ≤ iny ≤ iny_scan_valid_max_mm (vd 70–970mm)
        #      → cắt cả 2 đầu (chống trigger dưới 70mm và quá 970mm).
        # Nếu InX trôi khỏi target → disarm S4 ngay + log warn.
        inx_at_target = self._at_position(1, self.config.inx_target2)
        valid_max = max(self.config.iny_scan_valid_max_mm, scan_target)
        iny_in_valid  = (self.config.iny_scan_valid_min_mm
                         <= iny <=
                         valid_max)
        new_armed     = inx_at_target and iny_in_valid

        # Disarm + warn nếu InX trôi giữa chừng (đã armed mà giờ inx_at_target=False)
        if self._s4_armed and not inx_at_target:
            inx_now = self._pos(1)
            self._log_once(
                "S1_SCAN_INX_DRIFT",
                f"[S1 SCAN] InX troi khoi target "
                f"({inx_now if inx_now is None else f'{inx_now:.2f}'}mm "
                f"vs {self.config.inx_target2}mm, tol={self.config.position_tolerance}mm) "
                f"— DISARM S4"
            )
        if new_armed and not self._s4_armed:
            self._s4_prev_in = self.sensor(S4_SCAN_STACK_P1)
        self._s4_armed = new_armed

        s4_now = self.sensor(S4_SCAN_STACK_P1)
        # Khi armed, S4 ON trong zone là đủ để chốt row. Không chỉ phụ thuộc
        # rising edge, vì sensor có thể đã ON trước lúc phần mềm armed.
        s4_rising = self._s4_armed and (not self._s4_prev_in) and s4_now
        if self._s4_armed:
            self._s4_prev_in = s4_now

        invalid_trigger_pos = None
        forced_row = None
        if self._s4_armed and s4_now:
            trigger_pos = self._pos(2) or iny

            row = self._zone_to_row(trigger_pos, self.config.iny_input_zones)
            if not getattr(self, '_s1_s4_on_logged', False):
                zone_msg = f"row{row}" if row is not None else "NO_ZONE"
                self.get_logger().warn(
                    f"[S1 SCAN] S4 ON detect @ InY={trigger_pos:.1f}mm ({zone_msg})"
                )
                # Gửi sang Log Activity của GUI, không bật notification banner.
                # Mỗi lượt scan chỉ ghi một lần nhờ _s1_s4_on_logged.
                if row is not None:
                    target_mm = self.config.iny_input_zones[row][2]
                    self._notify(
                        'silent_ok',
                        'S4 ON — DETECT',
                        f'InY={trigger_pos:.1f} mm | Row {row} | Target={target_mm:.1f} mm'
                    )
                else:
                    self._notify(
                        'silent_info',
                        'S4 ON — NGOÀI ZONE',
                        f'InY={trigger_pos:.1f} mm | Không khớp row nào'
                    )
                self._s1_s4_on_logged = True
            if row is None:
                # Trigger rơi khe hở giữa 2 zone (calib lệch / nhiễu) → rơi
                # xuống nhánh fail bên dưới NGAY tick này: stop + retry scan,
                # KHÔNG đoán row (sự cố 2026-07-03: 804.9mm hụt biên dưới
                # row2 0.1mm → cũ fallback row1 → chạy thẳng R1).
                invalid_trigger_pos = trigger_pos
            else:
                target_mm         = self.config.iny_input_zones[row][2]
                self._current_row = row

                self.get_logger().info(
                    f"[S1 SCAN] S4 ON @ {trigger_pos:.1f}mm → row{row} ({target_mm:.0f}mm) - Moving directly"
                )
                self._s5_retry = 0
                self._enter_in(SystemState.S1_INY_TO_ROW)
                return

        timed_out = time.time() > self._step_timeout_in
        at_target = self._arrived(2) or iny >= scan_target - 2.0
        if at_target and s4_now:
            row_at_scan_end = self._zone_to_row(iny, self.config.iny_input_zones)
            if row_at_scan_end is not None:
                forced_row = row_at_scan_end
        if timed_out or at_target or invalid_trigger_pos is not None:
            if invalid_trigger_pos is not None:
                self.get_logger().warn(
                    f"[S1 SCAN] S4 trigger @ {invalid_trigger_pos:.1f}mm NGOÀI mọi zone "
                    f"— xử lý như nhiễu (kiểm tra calib iny_input_zones)"
                )
            else:
                self.get_logger().warn("[S1 SCAN] Hết hạn scan mà không có rising edge hợp lệ của S4")
            self._stop(2)

            if forced_row is not None:
                target_mm = self.config.iny_input_zones[forced_row][2]
                self._current_row = forced_row
                self._s5_retry = 0
                self.get_logger().warn(
                    f"[S1 SCAN] Reached row{forced_row} zone @ {iny:.1f}mm with S4 ON "
                    f"but no new rising edge → row{forced_row} ({target_mm:.0f}mm)"
                )
                self._enter_in(SystemState.S1_INY_TO_ROW)
                return

            if self._s1_scan_noise_retry < self._conf('s1_scan_noise_retry_limit', 1):
                self._s1_scan_noise_retry += 1
                self._notify(
                    'warn',
                    'S4 trigger ngoài zone' if invalid_trigger_pos is not None else 'Không bắt được S4 hợp lệ',
                    f'Retry scan lần {self._s1_scan_noise_retry}'
                )
                self._enter_in(SystemState.S1_RETRY_SCAN_HOME)
                return

            self._notify(
                'error',
                'Scan S4 thất bại',
                f'INY về {self.config.iny_home:.0f}mm, INX về {self._conf("inx_noise_recovery_mm", 10.0):.0f}mm'
            )
            self._nb_move(2, self.config.iny_home)
            self._nb_move(1, self._conf("inx_noise_recovery_mm", 10.0))
            self._jog_mode = True
            self._sync_mode_jog()
            self._enter_in(SystemState.IDLE)
            return

        self._log_once("S1_SCANNING",
                       f"[S1 SCAN] INY {iny:.0f}mm arm={'OK' if self._s4_armed else 'NO'} S4={'ON' if s4_now else 'OFF'}")

    def _s1_retry_scan_home(self):
        """
        Retry sau khi scan thất bại lần 1: đưa InY về home rồi quay lại S1_INY_SCAN.
        Có timeout (move_timeout). Nếu InY không về được home → vào IDLE,
        notify error. Nếu OK → reset _s4_armed, _s4_prev_in rồi vào S1_INY_SCAN.
        """
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_home)
            if not ok:
                self._notify('error', 'Retry scan fail', 'INY không về được home')
                self._nb_move(1, self._conf("inx_noise_recovery_mm", 10.0))
                self._enter_in(SystemState.IDLE)
                return
            self._cmd_sent_in = True
            self._step_timeout_in = time.time() + self.config.move_timeout
            return

        if time.time() > self._step_timeout_in:
            self._notify('error', 'Retry scan timeout', 'INY về home timeout')
            self._nb_move(1, self._conf("inx_noise_recovery_mm", 10.0))
            self._enter_in(SystemState.IDLE)
            return

        # [BLOCKING-FIX] verify _at_position(2, iny_home) — đảm bảo InY về home
        # thật sự trước khi retry scan, tránh scan từ vị trí giữa zone
        if self._arrived(2) and self._at_position(2, self.config.iny_home):
            self._cmd_sent_in = False
            self._step_timeout_in = 0.0
            self._s4_armed = False
            self._s4_prev_in = self.sensor(S4_SCAN_STACK_P1)
            self.get_logger().info("[S1 SCAN] Retry lại từ INY home")
            self._enter_in(SystemState.S1_INY_SCAN)

    def _s1_iny_to_row(self):
        """
        Sau khi S4 rising edge phát hiện row N: di chuyển InY đến target
        `iny_input_zones[N][2]` (chỉ số 2 = robot pickup position của row đó).
        Vận tốc chậm = iny_row_vel (docking).

        CAO RỦI RO: Cyl1 sẽ kẹp khay tại row này. Vị trí sai = kẹp sai height
        → va chạm hoặc kẹt. Verify _at_position(2, target) trước khi sang S1_CHECK_S5.
        """
        zone = self.config.iny_input_zones.get(self._current_row)
        target = zone[2] if zone else None
        if target is None:
            valid = sorted(self.config.iny_input_zones.keys())
            self._current_row = max(valid[0], min(valid[-1], self._current_row))
            target = self.config.iny_input_zones[self._current_row][2]
        if not self._cmd_sent_in:
            ok = self._nb_move(2, target, vel=self.config.iny_row_vel, continuous_update=True)
            if not ok:
                self._log_once("S1_ROW_FAIL", f"S1: INY -> {target}mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            # [BLOCKING-FIX] CAO — Cyl1 sẽ kẹp tại row, sai vị trí = va chạm
            elif self._arrived(2) and self._at_position(2, target):
                self._step_start_in = time.time()
                self._enter_in(SystemState.S1_CHECK_S5)

    def _s1_check_s5(self):
        """
        Sau khi InY đến row: chờ S5 (Output Detect) ON xác nhận có khay đúng vị trí.
        S5 ON → kích `_cyl1_extend()` → S1_CYL1_EXTEND.
        S5 OFF quá OUTPUT_DETECT_WAIT_S: retry 1 lần (về S1_CHECK_S5 với _s5_retry=1);
        retry lần 2 vẫn fail → S1_FALLBACK_RETRACT.
        """
        if not self._cmd_sent_in:
            self._step_start_in = time.time()
            self._cmd_sent_in   = True

        if self.sensor(S5_OUTPUT_DETECT):
            self.get_logger().info(f"S5 ON (lan {self._s5_retry+1}) — extend Cyl1")
            self._cyl1_extend()
            self._enter_in(SystemState.S1_CYL1_EXTEND)
            return

        elapsed = time.time() - self._step_start_in
        self._log_once("S1_WAIT_S5",
                       f"Cho S5 ON (con {max(0.0, OUTPUT_DETECT_WAIT_S - elapsed):.1f}s, "
                       f"lan {self._s5_retry+1}/2) row {self._current_row}")

        if elapsed >= OUTPUT_DETECT_WAIT_S:
            if self._s5_retry == 0:
                self._s5_retry = 1
                self._cmd_sent_in = False
                self._guide_logged.discard("S1_WAIT_S5")
                self.get_logger().warn(f"S5 OFF lan 1 tai row {self._current_row} — thu lai lan 2")
            else:
                self.get_logger().warn(f"S5 OFF 2 lan tai row {self._current_row} — reset")
                self._notify_step('warn', 'STATE 1', f'check {self._sensor_label(5)} (row {self._current_row})',
                    f'{self._sensor_label(5)} OFF 2 lần tại row {self._current_row} — khay không xác nhận tới vị trí cấp',
                    check=[f'{self._sensor_label(5)} thẳng hàng/sạch không',
                           'khay đã được đẩy hết bằng Cyl1 chưa',
                           'cơ khí kẹp khay'],
                    action=['Hệ thống tự fallback retract Cyl1', 'Nếu lặp lại: STOP rồi kiểm tra sensor S5'])
                self._s5_retry = 0
                self._enter_in(SystemState.S1_FALLBACK_RETRACT)

    def _s1_fallback_retract(self):
        """
        Recovery khi S5 fail 2 lần: retract Cyl1 + chờ S9 ON + S10 OFF (cross-check
        cylinder thực sự thu về). Khi an toàn → ra lệnh InY về home → S1_FALLBACK_WAIT_INY.
        Retry _cyl1_retract() mỗi 3s nếu sensor chưa đúng (chưa có max attempts ở đây).
        """
        cyl1_ret, cyl1_ext = self._snap(S9_CYL1_RETRACTED, S10_CYL1_EXTENDED)
        if not self._cmd_sent_in:
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in = True

        if cyl1_ret and not cyl1_ext:
            self.get_logger().info("Fallback: S9 ON + S10 OFF -> INY ve home")
            self._nb_move(2, self.config.iny_home)
            self._cmd_sent_in = True
            self._step_timeout_in = time.time() + self.config.move_timeout
            self._enter_in(SystemState.S1_FALLBACK_WAIT_INY)
            return

        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Fallback: Retry Cyl1 retract")
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S1_FB_RETRACT", "Fallback: cho S9 ON, S10 OFF")

    def _s1_fallback_wait_iny(self):
        """
        Chờ InY về home sau fallback retract. Khi InY ≤ iny_home + 2mm (verify
        bằng _pos): ra lệnh InX về `inx_safe` (-60mm) rồi vào S1_CONFIRM_SAFE
        để bắt đầu lại STATE 1 từ đầu. Timeout → ERROR.
        """
        if time.time() > self._step_timeout_in:
            self.get_logger().error("Timeout iny ve home")
            self._error("Fallback INY timeout")
            return
        iny = self._pos(2)
        if self._arrived(2) or (iny is not None and iny <= self.config.iny_home + 2.0):
            self.get_logger().info("INY ve home safe -> INX ve safe (-60mm)")
            self._nb_move(1, self.config.inx_safe,vel=200)
            self._enter_in(SystemState.S1_CONFIRM_SAFE)

    def _go_gui_confirm(self):
        """
        Helper escalate khi sub-state INPUT (S1/S2A) timeout hoặc gặp lỗi nguy hiểm.
        Hành động: ra lệnh InY về home + chuyển sang S1_WAIT_GUI_CONFIRM + notify GUI
        cấp độ 'error' để operator kiểm tra cơ khí/sensor rồi nhấn XÁC NHẬN.
        Khác `_error()` (vào ERROR state) — `_go_gui_confirm` cho phép phục hồi
        không cần restart node.
        """
        self._nb_move(2, self.config.iny_home)
        self._step_timeout_in  = time.time() + self.config.move_timeout
        self._gui_confirmed = False
        self._enter_in(SystemState.S1_WAIT_GUI_CONFIRM)
        self._notify_step('error', 'STATE 1', f'wait {self._sensor_label(4)}/{self._sensor_label(5)}',
            f'Cảm biến không phát hiện khay — kẹt/lỗi sensor',
            check=[f'{self._sensor_label(4)} (scan stack) sạch + thẳng hàng',
                   f'{self._sensor_label(5)} (output detect) đúng vị trí',
                   'cơ khí stack có lệch khay không'],
            action=['Khắc phục cơ khí/sensor', 'Nhấn XÁC NHẬN trên GUI để tiếp tục', 'Hoặc STOP để abort'])

    def _s1_wait_gui_confirm(self):
        """
        Đợi operator nhấn XÁC NHẬN trên GUI (set `_gui_confirmed=True`).
        Trong khi chờ: ra lệnh InX về `inx_safe` (-60mm) nếu InY đã safe.
        Khi xác nhận → stop cả 2 trục + sang S1_RETRY_JOG.
        """
        if not self._cmd_sent_in:
            if self._iny_safe():
                ok = self._nb_move(1, self.config.inx_safe)
                if ok:
                    self._cmd_sent_in = True
        if self._gui_confirmed:
            self._gui_confirmed = False
            self._stop(1); self._stop(2)
            self._enter_in(SystemState.S1_RETRY_JOG)
        self._log_once("S1_GUI_WAIT", "Cho nhan XAC NHAN tren GUI")

    def _s1_retry_jog(self):
        """
        Sau khi operator XÁC NHẬN từ GUI: tiếp tục STATE 1 nếu có khay.
        - Không có khay (S1/S2/S3 OFF): notify "Het khay" và chờ.
        - Có khay + InX gần inx_target2 (sai số <15mm): trực tiếp vào S1_INY_SCAN.
        - Có khay + InX ở xa: chuyển sang S1_INX_TRY_POS_PICK để di chuyển InX trước.
        """
        if not (self.sensor(S1_BELT_START) or self.sensor(S2_BELT_MID) or self.sensor(S3_BELT_END)):
            self._log_once("S1_NO_TRAY", "Het khay — S1/S2/S3 OFF")
            self._notify_step('warn', 'STATE 1', 'wait input tray',
                f'Belt input rỗng: {self._sensor_label(1)} + {self._sensor_label(2)} + {self._sensor_label(3)} đều OFF',
                check=['còn khay nào trên belt không',
                       f'{self._sensor_label(1)}/{self._sensor_label(2)}/{self._sensor_label(3)} có nhiễu / lệch không'],
                action=['Nạp khay lên belt input', 'Hệ thống tự tiếp tục khi sensor ON'])
            return
        inx = self._pos(1)
        if inx is None:
            return
        if abs(inx - self.config.inx_target2) < 15.0:
            self._s4_armed = False; self._s5_retry = 0
            self._enter_in(SystemState.S1_INY_SCAN)
        else:
            self._enter_in(SystemState.S1_INX_TRY_POS_PICK)

    def _s1_inx_try_pos_pick(self):
        """
        Retry đưa InX về inx_target2 (505.5mm) sau khi operator XÁC NHẬN.
        Pre-condition: InY phải safe (≤ iny_safe_zone) — interlock rule.
        Verify _at_position(1, inx_target2) trước khi sang S1_INY_SCAN.
        """
        if not self._cmd_sent_in:
            if not self._iny_safe():
                self._log_once("S1_RETRY_INY", "Cho INY safe truoc INX 500mm")
                return
            ok = self._nb_move(1, self.config.inx_target2)
            if not ok:
                self._log_once("S1_RETRY_INX", "S1 Retry: INX 500mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            # [BLOCKING-FIX] verify _at_position(1, inx_target2)
            elif self._arrived(1) and self._at_position(1, self.config.inx_target2):
                self._s4_armed = False; self._s5_retry = 0
                self._enter_in(SystemState.S1_INY_SCAN)

    def _s1_cyl1_extend(self):
        """
        Kích Cyl1 EXTEND để kẹp khay tại row. Chờ S10 (Cyl1 Extended) ON.
        Retry _cyl1_extend() mỗi 3s nếu S10 chưa ON (chưa có max attempts).
        BỎ check S5 trong state này vì quá trình đẩy rulo có thể làm rung S5.
        Next: S1_INY_PICK_TRAY_UP (rút InY về home để chuẩn bị đẩy khay đi).
        """
        cyl1_ext, = self._snap(S10_CYL1_EXTENDED)
        # BO CHECK S5 o day vi qua trinh day rulo co the lam rung S5
        if not self._cmd_sent_in:
            self._cyl1_extend()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in    = True
        if cyl1_ext:
            self.get_logger().info("S10 ON — gap khay OK -> INY ve 50mm")
            self._enter_in(SystemState.S1_INY_PICK_TRAY_UP)
            return
        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Retry Cyl1 extend")
            self._cyl1_extend()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S1_WAIT_S14", "Cho S10 ON (Cyl1 extend)")

    def _s1_iny_pick_tray_up(self):
        """
        Sau khi Cyl1 kẹp khay: rút InY về home (0mm) với vận tốc 250mm/s để
        kéo khay khỏi stack. Tên hàm là "iny_50" vì lịch sử trước đây dừng ở
        50mm; hiện target là `iny_home` = 0mm.
        Next: S1_INY_PLACE_TRAY_ROBOT (đẩy InY về iny_target2 để đặt khay vào vị trí robot).
        """
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_home,vel=250)
            if not ok:
                self.get_logger().warn("S1: INY -> home (0mm) fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            # [BLOCKING-FIX] verify _at_position(2, iny_home)
            elif self._arrived(2) and self._at_position(2, self.config.iny_home):
                self._enter_in(SystemState.S1_INY_PLACE_TRAY_ROBOT)

    def _s1_iny_place_tray_robot(self):
        """
        Di chuyển InY → iny_target2 (87mm) để đặt khay vào vị trí robot pickup.
        Khay đang được Cyl1 kẹp suốt quá trình này.
        Next: S1_WAIT_RELEASE (nhả Cyl1 sau khi đến vị trí).
        """
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_target2, vel=120)
            if not ok:
                self.get_logger().warn("S1: INY -> 200mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            # [BLOCKING-FIX] verify _at_position(2, iny_target2)
            elif self._arrived(2) and self._at_position(2, self.config.iny_target2):
                self._enter_in(SystemState.S1_WAIT_RELEASE)

    def _s1_wait_release(self):
        """
        Sau khi InY tại iny_target2: retract Cyl1 (nhả khay). Cross-check 2 sensor:
        S9 ON (Cyl1 retracted) AND S10 OFF (Cyl1 NOT extended) — KHÔNG chỉ check 1
        (xem RULE 9). Retry _cyl1_retract() mỗi 3s nếu chưa đạt cross-check.
        Next: S1_WAIT_S7 (chờ sensor xác nhận khay tại robot).
        """
        cyl1_ret, cyl1_ext = self._snap(S9_CYL1_RETRACTED, S10_CYL1_EXTENDED)
        if not self._cmd_sent_in:
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in    = True
        if cyl1_ret and not cyl1_ext:
            self.get_logger().info("S9 ON + S10 OFF — Cyl1 nha xong -> cho S7")
            self._enter_in(SystemState.S1_WAIT_S7)
            return
        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Retry Cyl1 retract")
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S1_WAIT_REL", f"Cho S9 ON + S10 OFF | S9={cyl1_ret} S10={cyl1_ext}")

    def _s1_wait_s7(self):
        """
        Chờ S7 (Tray At Robot) ON xác nhận khay đã ở vị trí robot pickup
        sau khi Cyl1 nhả. Khi ON → S1_INX_WAIT_TRAY_DONE (rút InX về safe).
        Hàm này không có timeout — phụ thuộc robot hoặc operator confirm.
        """
        tray_robot, = self._snap(S7_TRAY_AT_ROBOT)
        if tray_robot:
            self.get_logger().info("S7 ON — Khay o vi tri robot -> INX ve safe")
            self._enter_in(SystemState.S1_INX_WAIT_TRAY_DONE)
            return
        self._log_once("S1_WAIT_S7", "Cho S7 ON")

    def _s1_inx_wait_tray_done(self):
        """
        Đưa InX về `inx_safe` (-60mm) để giải phóng workspace cho robot.
        Tên hàm "inx_10" là lịch sử (trước đây target=10mm); hiện target=inx_safe.
        Pre-condition: InY safe (interlock).
        Next: S1_COMPLETE.
        """
        if not self._iny_safe():
            self._log_once("S1_INX_WAIT_INY", f"Cho INY <= {self.config.iny_safe_zone}mm truoc INX ve home")
            return
        if not self._cmd_sent_in:
            ok = self._nb_move(1, self.config.inx_safe,vel=200)
            if not ok:
                self.get_logger().warn(f"S1: INX -> {self.config.inx_safe}mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            # [BLOCKING-FIX] verify _at_position(1, inx_safe)
            elif self._arrived(1) and self._at_position(1, self.config.inx_safe):
                self._enter_in(SystemState.S1_COMPLETE)

    def _s1_complete(self):
        """
        Publish new_tray_loaded. Check S7 với timeout 3s.
        Xong thì về IDLE. IDLE sẽ chờ robot done_tray_input rồi kích S2A.
        """
        self._pub_cartridge_busy(False)
        self._s1_scan_noise_retry = 0
        self._s4_prev_in         = False
        if not self._cmd_sent_in:
            self.stack_row_index  = self._current_row
            self._cmd_sent_in     = True
            self._tray_robot_check_start = 0.0

        if self.sensor(S7_TRAY_AT_ROBOT):
            if self._tray_robot_check_start >= 0.0:
                self.pub_new_tray.publish(Bool(data=True))
                self.get_logger().info("Published: new_tray_loaded = True (S7 ON)")
                self._notify('info', 'STATE 1 COMPLETE', 'Khay đã ở robot')
                self._drain_armed_after_s2 = False
                self._publish_drain(False)
                self._tray_robot_check_start = -1.0

            self._state1_enabled = False
            self._enter_in(SystemState.IDLE)
        else:
            if self._tray_robot_check_start == 0.0:
                self._tray_robot_check_start = time.time()
                self.get_logger().warn("S7 chua ON — cho 3s")
            elapsed = time.time() - self._tray_robot_check_start
            if elapsed >= TRAY_ROBOT_CHECK_TIMEOUT_S:
                self.get_logger().error(f"S7 OFF sau {TRAY_ROBOT_CHECK_TIMEOUT_S:.0f}s — caution")
                self._notify_step('warn', 'STATE 1', 'verify tray at robot',
                    f'{self._sensor_label(7)} OFF sau {TRAY_ROBOT_CHECK_TIMEOUT_S:.0f}s — khay không xác nhận tại Robot',
                    check=[f'khay có thật ở vị trí Robot không',
                           f'{self._sensor_label(7)} thẳng hàng + sạch',
                           'cơ khí đẩy khay (Cyl1) hoạt động đủ chưa'],
                    action=['Đẩy khay manual nếu cần', 'Kiểm tra wiring S7', 'Hệ thống quay về IDLE để retry'])
                self.pub_new_tray.publish(Bool(data=True))
                self._drain_armed_after_s2 = False
                self._publish_drain(False)
                self._state1_enabled = False
                self._enter_in(SystemState.IDLE)
            else:
                self._log_once("S1C_S18_WAIT",
                               f"Cho S7 ON (con {TRAY_ROBOT_CHECK_TIMEOUT_S - elapsed:.1f}s)")

    # ══════════════════════════════════════════════════════════════
    # STATE 2: Thay khay Input  (giữ nguyên logic gốc)
    # ══════════════════════════════════════════════════════════════

    def _s2a_preflight_scan(self):
        """
        Snapshot toàn bộ sensor liên quan tại thời điểm trigger STATE 2A (gọi từ A1).
        Log structured + set các snapshot cho workflow sau dùng:
          • _s6_snapshot       — S6 (tray at outP1) → quyết định path scan ở A7
          • _s13_snapshot      — S13 limit switch tại Pos1 (output)
          • _s14_snapshot      — S14 limit switch tại Pos1 (output)
          • _s15_snapshot      — S15 (Cyl3 retracted) — informational
          • _s16_snapshot      — S16 (Cyl3 extended) — informational

        Cyl1 sensor (S9/S10) đọc nhưng KHÔNG snapshot — workflow sau dùng live (state
        machine kích extend/retract liên tục, snapshot sẽ stale ngay).

        Cảnh báo (không block) khi sensor state không khớp expected:
          • Cyl1 unknown (S9 OFF + S10 OFF) hoặc xung đột (S9 ON + S10 ON)
          • Cyl3 unknown (S15 OFF + S16 OFF) hoặc xung đột
        """
        s6, s7 = self._snap(S6_CHECK_TRAY_P1, S7_TRAY_AT_ROBOT)
        s9, s10 = self._snap(S9_CYL1_RETRACTED, S10_CYL1_EXTENDED)
        s13, s14 = self._snap(S13_OUT1_TRAYPOS1, S14_OUT2_TRAYPOS1)
        s15, s16 = self._snap(S15_CYL3_RETRACTED, S16_CYL3_EXTENDED)
        inx = self._pos_cached(1)
        iny = self._pos_cached(2)
        inx_str = f"{inx:.1f}mm" if inx is not None else "?"
        iny_str = f"{iny:.1f}mm" if iny is not None else "?"

        # Lưu snapshot dùng cho workflow sau
        self._s6_snapshot  = s6
        self._s13_snapshot = s13
        self._s14_snapshot = s14
        self._s15_snapshot = s15
        self._s16_snapshot = s16

        self.get_logger().info(
            f"[S2A PRE-FLIGHT] S6={int(s6)} S7={int(s7)} "
            f"| Cyl1 S9={int(s9)} S10={int(s10)} "
            f"| Cyl3 S15={int(s15)} S16={int(s16)} "
            f"| OutLim S13={int(s13)} S14={int(s14)} "
            f"| InX={inx_str} InY={iny_str}"
        )
        self.get_logger().info(
            f"[S2A PRE-FLIGHT] decision: S6={s6} → "
            f"{'jog do S4' if s6 else 'thẳng row1'}"
        )

        # Sanity warnings (không block — chỉ log để operator biết)
        if s9 and s10:
            self.get_logger().warn(
                "[S2A PRE-FLIGHT] Cyl1 CONFLICT: S9+S10 cùng ON — cảm biến/cơ khí lỗi"
            )
        elif not s9 and not s10:
            self.get_logger().warn(
                "[S2A PRE-FLIGHT] Cyl1 UNKNOWN: S9+S10 cùng OFF — piston ở giữa hành trình"
            )
        if s15 and s16:
            self.get_logger().warn(
                "[S2A PRE-FLIGHT] Cyl3 CONFLICT: S15+S16 cùng ON — cảm biến/cơ khí lỗi"
            )
        elif not s15 and not s16:
            self.get_logger().warn(
                "[S2A PRE-FLIGHT] Cyl3 UNKNOWN: S15+S16 cùng OFF — piston ở giữa hành trình"
            )

    def _s2a_check_interlock(self):
        """
        STATE 2A bước A1 (entry point): chuẩn bị state cho luồng thay khay input.

        Pre-condition: state_in vừa chuyển từ IDLE → S2A_CHECK_INTERLOCK (qua nút
        STATE 2 trong manual hoặc qua _can_start_s2a trong auto).

        Khi vào A1 (first tick) gọi `_s2a_preflight_scan()` để snapshot toàn bộ
        sensor liên quan + log structured. Snapshots dùng cho workflow sau:
        S6 cho A7 path decision, S13-S16 cho monitor/debug.

        Drive warm-up (FAS firmware quirk): tương tự _s1_confirm_safe — sau homing
        (referencing_task) drive accept position_task nhưng không chuyển động vật lý.
        Phải gọi stop_motion_task() để flush queue, đợi 1s settle, rồi mới gửi move.

        Trước khi chạy State 2, giữ đúng wait posture cuối State 1:
        InX=inx_safe (-60mm), InY=iny_target2 (87mm). Chỉ tại posture này
        mới scan S4. S4 ON thì khóa State 2 và chờ operator ACCEPT.

        Next: S2A_WAIT_OUTPUT_CLEAR hoặc S2A_INX_MOVE_POS_PICK.
        """
        # ── Drive warm-up gate (FAS firmware quirk) ──────────────────
        if self._drive_warm_t < 0.0:
            for sid in list(self.servos):
                self._stop(sid)
            self._drive_warm_t = time.time() + 1.0
            self.get_logger().info("S2A: drive warm-up (stop_motion_task + 1s settle)")
            return
        if self._drive_warm_t > 0.0:
            if time.time() < self._drive_warm_t:
                return  # đang chờ settle
            self._drive_warm_t = 0.0
        # ─────────────────────────────────────────────────────────────

        # STATE 1 normally leaves exactly this posture. Re-establish it before
        # reading S4 if an abnormal/manual move caused either axis to drift.
        # InX must never travel toward the pick target while S4 is unresolved.
        if not self._at_position(2, self.config.iny_target2):
            if not self._iny_moving:
                ok = self._nb_move(2, self.config.iny_target2, vel=150)
                if not ok:
                    self.get_logger().warn(
                        f"S2A gate: INY -> {self.config.iny_target2}mm fail"
                    )
            self._log_once(
                "S2A_GATE_INY_87",
                f"S2A gate: cho InY tai {self.config.iny_target2}mm de scan S4",
            )
            return

        if not self._at_position(1, self.config.inx_safe):
            if not self._inx_moving:
                ok = self._nb_move(1, self.config.inx_safe, vel=200)
                if not ok:
                    self.get_logger().warn(
                        f"S2A gate: INX -> safe {self.config.inx_safe}mm fail"
                    )
            self._log_once(
                "S2A_GATE_INX_SAFE",
                f"S2A gate: cho InX tai safe {self.config.inx_safe}mm",
            )
            return

        if not self._cmd_sent_in:
            self._s2_output_full_accept = False
            self._pub_cartridge_busy(True)

            # Gate before State 2 performs its normal preflight/actuator work.
            if self.sensor(S4_SCAN_STACK_P1):
                self._block_s2_for_output_full()
                return

            self._s2a_preflight_scan()

            # --- CYL3 S6 CHECK ON STATE 2 TRIGGER ---
            if self._conf('cyl3_present', True):
                s6 = self.sensor(S6_CHECK_TRAY_P1)
                if s6:
                    self._cyl3_s13s14_latch = False
                    self._cyl3_extend()
                    self.get_logger().info("[S2A TRIGGER] S6 ON → Clear Latch + Extend Cyl3")
                else:
                    self._cyl3_retract()
                    self.get_logger().info("[S2A TRIGGER] S6 OFF → Retract Cyl3")
            # ----------------------------------------

            self._s4_armed_out      = False
            self._output_row        = 0
            self._output_target_pos = 0.0
            self._cmd_sent_in = True
        self._enter_in(SystemState.S2A_INX_MOVE_POS_PICK)

    def _block_s2_for_output_full(self):
        """Khóa State 2 khi InX=-60mm, InY=87mm; không cho pick motion."""
        self._s2_output_full_accept = False
        self.get_logger().warn(
            f"[S2A] S4 ON -> block State 2; hold InX={self.config.inx_safe}mm, "
            f"InY={self.config.iny_target2}mm (pick InX={self.config.inx_target2}mm)"
        )
        self._notify_s2_output_full()
        self._enter_in(SystemState.S2A_WAIT_OUTPUT_CLEAR)

    def _s2a_inx_move_pos_pick(self):
        """
        STATE 2A bước A2: InX → inx_target2 để lấy khay cũ từ robot về.
        Pre-condition:
          - InY safe (interlock).
          - S4 OFF ngay trước khi gửi lệnh InX rời inx_safe.
          - S9 (Cyl1 RETRACTED) ON — interlock S2 mới: tránh InX di chuyển khi Cyl1
            còn extended trong workspace.
        Next: S2A_POS_PLACE_TRAY_ROBOT_CYL1.
        """
        if not self._iny_safe():
            self._log_once("S2A_INY2", "A2: INY chua safe")
            return
        if not self._cmd_sent_in and self.sensor(S4_SCAN_STACK_P1):
            # Last-moment gate: cover a S4 transition between A1 and A2 ticks.
            # No pick command has been sent yet, so InX is still at inx_safe.
            self._block_s2_for_output_full()
            return
        if not self.sensor(S9_CYL1_RETRACTED):
            self._log_once("S2A_S13_ILK", "INTERLOCK S2: S9=OFF -> INX bi khoa")
            self._notify_step('warn', 'STATE 2A',
                f'A2 InX→{self.config.inx_target2:g}',
                f'INTERLOCK: {self._sensor_label(9)} OFF (Cyl1 chưa retract) — InX bị khóa',
                enum_name='S2A_INX_MOVE_POS_PICK',
                check=['van Cyl1 retract', 'áp khí', f'dây {self._sensor_label(9)}', f'có thể đồng thời {self._sensor_label(10)} ON?'],
                action=['JOG → Cyl1 RETRACT manual đến khi S9 ON', 'Hoặc STOP rồi check cơ khí/van'])
            return
        if not self._cmd_sent_in:
            ok = self._nb_move(1, self.config.inx_target2, vel=200)
            if not ok:
                self._log_once(
                    "S2A_A2_FAIL",
                    f"S2A A2: INX {self.config.inx_target2:g}mm fail",
                )
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            # [BLOCKING-FIX] verify _at_position(1, inx_target2)
            elif self._arrived(1) and self._at_position(1, self.config.inx_target2):
                self._enter_in(SystemState.S2A_POS_PLACE_TRAY_ROBOT_CYL1)

    def _s2a_pos_place_tray_robot_cyl1(self):
        """
        STATE 2A bước A3: InY → iny_target2 (87mm) tới vị trí kẹp khay cũ, sau đó
        kích Cyl1 EXTEND để kẹp. S4 đã được gate khi InX còn ở
        inx_safe và InY ở iny_target2, trước bước InX pick motion.
        CAO RỦI RO: verify _at_position(2, iny_target2) trước khi Cyl1 extend
        để tránh kẹp sai vị trí.
        Next: S2A_WAIT_CYL_EXT.
        """
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_target2, vel=150)
            if not ok:
                self._log_once("S2A_A3_FAIL", "S2A A3: INY 87mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            # [BLOCKING-FIX] CAO — Cyl1 sẽ extend kẹp khay; sai vị trí = va chạm
            elif self._arrived(2) and self._at_position(2, self.config.iny_target2):
                self.get_logger().info(
                    f"A3: INY tai {self.config.iny_target2}mm -> Cyl1 EXTEND"
                )
                self._cyl1_extend()
                self._enter_in(SystemState.S2A_WAIT_CYL_EXT)

    def _notify_s2_output_full(self, recheck: bool = False):
        """Hiện popup chuyên biệt; GUI chỉ gửi yêu cầu recheck S4."""
        prefix = "Kiểm tra lại: S4 vẫn ON. " if recheck else ""
        self._notify(
            'warn',
            'OUTPUT TRAY FULL',
            prefix
            + f'S4 đang ON tại vị trí check tray. InX được giữ tại '
              f'{self.config.inx_safe:g} mm, InY giữ tại '
              f'{self.config.iny_target2:g} mm; InX không di chuyển tới vị trí '
              f'lấy khay {self.config.inx_target2:g} mm. Dọn khay Output rồi '
              'nhấn ACCEPT để kiểm tra lại S4.',
            code='s2_output_full',
        )

    def _s2a_wait_output_clear(self):
        """
        Giữ wait posture InX=inx_safe, InY=iny_target2 khi output stack đầy.

        S4 tự OFF không làm flow tiếp tục. Chỉ khi operator nhấn
        ACCEPT, callback đặt one-shot flag và state này mới recheck S4.
        S4 còn ON -> tiếp tục chờ; S4 OFF -> chạy lại interlock A1
        rồi mới cho InX đi tới vị trí lấy khay.
        """
        # While waiting, issue no motion and no Modbus position reads. The
        # servo drives keep the verified -60/87 posture, avoiding unnecessary
        # bus load during a potentially long operator wait.
        if not self._s2_output_full_accept:
            return

        # Consume every ACCEPT exactly once. Posture is verified before S4;
        # if either axis drifted, restore it safely and require a new ACCEPT.
        self._s2_output_full_accept = False
        if not self._at_position(2, self.config.iny_target2):
            if not self._iny_moving:
                self._nb_move(2, self.config.iny_target2, vel=150)
            self._log_once(
                "S2A_OUTPUT_FULL_INY_87",
                f"S4 ON: giu InY tai {self.config.iny_target2}mm de check tray",
            )
            return

        if not self._at_position(1, self.config.inx_safe):
            if not self._inx_moving:
                self._nb_move(1, self.config.inx_safe, vel=200)
            self._log_once(
                "S2A_OUTPUT_FULL_INX_SAFE",
                f"S4 ON: giu InX tai safe {self.config.inx_safe}mm",
            )
            return

        if self.sensor(S4_SCAN_STACK_P1):
            self.get_logger().warn(
                "[S2A] ACCEPT recheck: S4 still ON -> keep State 2 blocked"
            )
            self._notify_s2_output_full(recheck=True)
            return

        self.get_logger().info(
            "[S2A] ACCEPT recheck: S4 OFF, output clear -> restart A1 interlocks"
        )
        self._notify(
            'info',
            'OUTPUT TRAY CLEAR',
            'S4 đã OFF. STATE 2 tiếp tục từ InX=-60 mm, InY=87 mm.',
            code='s2_output_clear',
        )
        self._enter_in(SystemState.S2A_CHECK_INTERLOCK)

    def _s2a_wait_cyl_ext(self):
        """
        STATE 2A bước A4: chờ S10 (Cyl1 Extended) ON xác nhận đã kẹp khay cũ.
        Retry _cyl1_extend() mỗi 3s.
        Next: S2A_INY_HOME (rút InY về home với khay đang kẹp).
        """
        cyl1_ext, = self._snap(S10_CYL1_EXTENDED)
        if not self._cmd_sent_in:
            self._cyl1_extend()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in    = True
        if cyl1_ext:
            self.get_logger().info("A4: S10 ON — gap khay cu OK")
            self._enter_in(SystemState.S2A_INY_HOME)
            return
        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Retry Cyl1 extend")
            self._cyl1_extend()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S2A_S14", "A4: Cho S10 ON")

    def _s2a_iny_home(self):
        """
        STATE 2A bước A5: kéo InY về iny_home (0mm) cùng khay cũ (Cyl1 đang kẹp).
        Next: S2A_INX_PLACE_TRAY_OUT_POS1 (đưa InX ra vị trí output stack).
        """
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_home, vel=150)
            if not ok:
                self._log_once("S2A_A5_FAIL", "S2A A5: INY 10mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            # [BLOCKING-FIX] verify _at_position(2, iny_home)
            elif self._arrived(2) and self._at_position(2, self.config.iny_home):
                self._enter_in(SystemState.S2A_INX_PLACE_TRAY_OUT_POS1)

    def _s2a_inx_place_tray_out_pos1(self):
        """
        STATE 2A bước A6: InX → inx_output_stack (100mm) — vị trí thả khay vào
        output stack. Pre-condition: InY safe.
        Reset _s4_armed_out = False trước khi vào jog output.
        Next: S2A_INY_JOG_OUTPUT.
        """
        if not self._iny_safe():
            self._log_once("S2A_INX10_WAIT", "A6: INY chua safe")
            return
        if not self._cmd_sent_in:
            ok = self._nb_move(1, self.config.inx_output_stack, vel=200)
            if not ok:
                self._log_once("S2A_A6_FAIL", "S2A A6: INX output pos fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            # [BLOCKING-FIX] verify _at_position(1, inx_output_stack)
            elif self._arrived(1) and self._at_position(1, self.config.inx_output_stack):
                self._s4_armed_out = False
                self._enter_in(SystemState.S2A_INY_JOG_OUTPUT)

    def _s2a_iny_jog_output(self):
        """
        STATE 2A bước A7: scan InY để xác định vị trí thả khay cũ vào output stack.

        Logic dùng chung với _s1_iny_scan — CHỈ KHÁC tọa độ InX target:
          - S1 scan: InX tại inx_target2 (505.5mm — vị trí lấy khay từ robot)
          - S2A scan: InX tại inx_output_stack (-300mm — vị trí output stack)
        Đều scan InY → target_scaninp1 (970mm) @ iny_scan_vel.

        S6 (pre-check stack):
          - S6 OFF (stack rỗng): bỏ qua S4 detect — 2-phase move trực tiếp đến row 1:
              phase 1: InY → 950mm @ vel=150 (fast pre-position, dưới row1 zone min 990)
              phase 2: InY → iny_output_zones[1][2] @ vel=50 (slow final approach)
              → enter S2A_INY_TARGETROW.
            Lý do: stack rỗng nên không có khay nào để S4 trigger → đi thẳng row 1.
          - S6 ON (stack có khay): scan với S4 rising-edge + armed-gate (giống S1):
              * inx_at_target (InX tại inx_output_stack)
              * iny in [iny_scan_valid_min_mm, iny_scan_valid_max_mm]
            S4 ON hợp lệ → _zone_to_row(iny_output_zones) → target row →
            enter S2A_INY_TARGETROW. Trigger NGOÀI mọi zone (None) → bỏ qua
            edge, tiếp tục scan.
            Hết hành trình mà S4 không trigger (hoặc mọi trigger đều ngoài
            zone) → fallback row 1 + warn + pause sau khi đặt.

        Next: S2A_INY_TARGETROW (di chuyển đến row target @ iny_row_vel + nhả Cyl1).
        """
        # SENSOR-FIRST: sau khi scan đã armed, kiểm tra S4 trước mọi lần đọc
        # encoder Modbus. Feedback encoder trên FAS cập nhật chậm; nếu đọc vị trí
        # trước rồi mới đọc S4, InY có thể đi thêm ~20mm ở 50mm/s.
        if (self._cmd_sent_in
                and self._s6_snapshot
                and self._s4_armed_out
                and self.sensor(S4_SCAN_STACK_P1)):
            detect_t = time.time()
            self._stop(2)  # ưu tiên dừng cơ khí, không chờ lấy vị trí để log
            trigger_pos = self._pos(2)
            if trigger_pos is None:
                trigger_pos = self._pos_cached(2)

            row = (self._zone_to_row(trigger_pos, self.config.iny_output_zones)
                   if trigger_pos is not None else None)
            pos_text = 'N/A' if trigger_pos is None else f'{trigger_pos:.1f}'
            zone_msg = f'row{row}' if row is not None else 'NO_ZONE'
            self.get_logger().warn(
                f'[S2A SENSOR-FIRST] S4 ON -> STOP @ InY={pos_text}mm ({zone_msg})'
            )

            if row is not None:
                target_mm = self.config.iny_output_zones[row][2]
                occupied = ' | Row 1 đang occupied' if row == 1 and self._row1_occupied else ''
                self._notify(
                    'silent_ok', 'OUTPUT ZONE — DETECT/STOP',
                    f'InY stop={pos_text} mm | Row {row} | Target={target_mm:.1f} mm'
                    f'{occupied} | Phản ứng={time.time() - detect_t:.3f}s'
                )
            else:
                self._notify(
                    'silent_info', 'OUTPUT ZONE — DETECT/STOP NGOÀI ZONE',
                    f'InY stop={pos_text} mm | Chuyển retry scan an toàn'
                )

            self._s2_s4_on_logged = True
            if row is not None and not (row == 1 and self._row1_occupied):
                self._output_target_pos = target_mm
                self._output_row = row
                self._enter_in(SystemState.S2A_INY_TARGETROW)
            else:
                # Không tiếp tục chạy mù sau khi đã STOP; về home rồi scan lại.
                self._enter_in(SystemState.S2A_RETRY_SCAN_HOME)
            return

        iny = self._pos(2)
        if iny is None:
            return
        row1_zone = self.config.iny_output_zones.get(1, [])
        row1_scan_max = max(row1_zone[0], row1_zone[1]) if len(row1_zone) >= 2 else self.config.target_scaninp1
        scan_target = max(self.config.target_scaninp1, row1_scan_max)

        if not self._cmd_sent_in:
            # REFRESH S6 trước khi quyết định path scan — A1 snapshot có thể stale
            # (A1 → A7 cách ~5-10s, hardware có thể đổi nếu STATE 2A vừa bắt đầu).
            # Trong STATE 2A đã có lock-out S13/S14 watchdog → S6 chỉ đổi nếu sensor
            # nhiễu / hardware fault. Refresh để placement decision dùng giá trị thật.
            fresh_s6 = self.sensor(S6_CHECK_TRAY_P1)
            if fresh_s6 != self._s6_snapshot:
                self.get_logger().info(
                    f"[S2A SCAN] S6 thay đổi từ A1: {self._s6_snapshot}→{fresh_s6} — refresh"
                )
            self._s6_snapshot = fresh_s6
            self._s4_armed_out    = False
            self._s4_prev_out     = self.sensor(S4_SCAN_STACK_P1)
            self._s2_s4_on_logged = False
            # Snapshot row1 occupancy: nếu Cyl3 đang giữ khay tại row1 (S16 ON + S6 ON)
            # → loại trừ row1 khỏi detection trong scan này.
            s16_snap, s6_snap = self._snap(S16_CYL3_EXTENDED, S6_CHECK_TRAY_P1)
            self._row1_occupied = bool(s16_snap and s6_snap)

            if not self._s6_snapshot:
                # S6 OFF: stack rỗng → phase 1 fast vel=150 đến 950mm (dưới row1 zone)
                ok = self._nb_move(2, 950.0, vel=150)
                if not ok:
                    self._log_once("S2A_SCAN_FAIL", "S2A scan S6 OFF phase 1: nb_move fail")
                    return
                self._s2a_s6off_phase = 1
                self.get_logger().info("[S2A SCAN] S6=OFF stack rỗng — fast InY → 950mm @ vel=150 (phase 1)")
            else:
                # S6 ON: scan vel=50 đến hết vùng row1 với S4 detection
                ok = self._nb_move(2, scan_target, vel=50)
                if not ok:
                    self._log_once("S2A_SCAN_FAIL", "S2A scan: nb_move fail")
                    return
                self._s2a_s6off_phase = 0
                self.get_logger().info(
                    f"[S2A SCAN] S6=ON — scan InY → {scan_target:.0f}mm @ vel=50 "
                    f"row1_occupied={self._row1_occupied}"
                )
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout

        # ── S6 OFF: 2-phase fast→slow trực tiếp tới row 1 target ──
        if not self._s6_snapshot:
            target_r1 = self.config.iny_output_zones[1][2]
            if self._s2a_s6off_phase == 1:
                if self._arrived(2) or iny >= 950.0 - 2.0:
                    ok = self._nb_move(2, target_r1, vel=50)
                    if not ok:
                        self._log_once("S2A_SCAN_FAIL", "S2A scan S6 OFF phase 2: nb_move fail")
                        return
                    self._s2a_s6off_phase = 2
                    self._step_timeout_in = time.time() + self.config.move_timeout
                    self.get_logger().info(
                        f"[S2A SCAN] S6=OFF arrived 950mm — slow → {target_r1:.0f}mm @ vel=50 (phase 2)"
                    )
                elif time.time() > self._step_timeout_in:
                    self._stop(2)
                    self.get_logger().warn("[S2A SCAN] S6=OFF phase 1 timeout")
                    self._enter_in(SystemState.ERROR)
                return

            if self._s2a_s6off_phase == 2:
                if self._arrived(2) or iny >= target_r1 - 2.0:
                    self._stop(2)
                    self._output_target_pos = target_r1
                    self._output_row = 1
                    self.get_logger().info(
                        f"[S2A SCAN] S6=OFF arrived row1 target ({target_r1:.0f}mm) → TARGETROW"
                    )
                    self._enter_in(SystemState.S2A_INY_TARGETROW)
                elif time.time() > self._step_timeout_in:
                    self._stop(2)
                    self.get_logger().warn(
                        f"[S2A SCAN] S6=OFF phase 2 timeout — fallback row1 ({target_r1:.0f}mm)"
                    )
                    self._output_target_pos = target_r1
                    self._output_row = 1
                    self._enter_in(SystemState.S2A_INY_TARGETROW)
                return

        # ── S6 ON: detect S4 (giống S1 rising-edge + armed-gate) ────
        if self._s6_snapshot:
            inx_at_target = self._at_position(1, self.config.inx_output_stack)
            valid_max = max(self.config.iny_scan_valid_max_mm, scan_target)
            iny_in_valid  = (self.config.iny_scan_valid_min_mm
                             <= iny <=
                             valid_max)
            new_armed     = inx_at_target and iny_in_valid

            if self._s4_armed_out and not inx_at_target:
                inx_now = self._pos(1)
                self._log_once(
                    "S2A_SCAN_INX_DRIFT",
                    f"[S2A SCAN] InX trôi khỏi {self.config.inx_output_stack}mm "
                    f"({inx_now if inx_now is None else f'{inx_now:.2f}'}mm, "
                    f"tol={self.config.position_tolerance}mm) — DISARM S4"
                )
            if new_armed and not self._s4_armed_out:
                self._s4_prev_out = self.sensor(S4_SCAN_STACK_P1)
            self._s4_armed_out = new_armed

            s4_now = self.sensor(S4_SCAN_STACK_P1)
            # S4 là NO: chạm khay tồn → rising edge (OFF → ON)
            s4_rising = self._s4_armed_out and (not self._s4_prev_out) and s4_now
            if self._s4_armed_out:
                self._s4_prev_out = s4_now

            if self._s4_armed_out and s4_now:
                trigger_pos = self._pos(2) or iny
                row = self._zone_to_row(trigger_pos, self.config.iny_output_zones)
                if not getattr(self, '_s2_s4_on_logged', False):
                    zone_msg = f"row{row}" if row is not None else "NO_ZONE"
                    self.get_logger().warn(
                        f"[S2A SCAN] S4 ON detect @ InY={trigger_pos:.1f}mm ({zone_msg})"
                    )
                    if row is not None:
                        target_mm = self.config.iny_output_zones[row][2]
                        occupied = ' | Row 1 đang occupied' if row == 1 and self._row1_occupied else ''
                        self._notify(
                            'silent_ok',
                            'OUTPUT ZONE — DETECT',
                            f'InY={trigger_pos:.1f} mm | Row {row} | '
                            f'Target={target_mm:.1f} mm{occupied}'
                        )
                    else:
                        self._notify(
                            'silent_info',
                            'OUTPUT ZONE — NGOÀI ZONE',
                            f'InY={trigger_pos:.1f} mm | Không khớp row nào'
                        )
                    self._s2_s4_on_logged = True
                if row is None:
                    # Trigger rơi khe hở giữa 2 zone / nhiễu → bỏ qua edge này,
                    # tiếp tục scan. Hết hành trình sẽ vào nhánh fallback row1
                    # + notify + pause sẵn có bên dưới (đang giữ khay nên vẫn
                    # phải đặt được — khác S1 là error về home).
                    self._log_once(
                        "S2A_SCAN_ZONE_MISS",
                        f"[S2A SCAN] S4 trigger @ {trigger_pos:.1f}mm NGOÀI mọi zone "
                        f"— bỏ qua, tiếp tục scan (kiểm tra calib iny_output_zones)"
                    )
                # Loại trừ row1 nếu đã biết Cyl3 giữ khay tại row1 → tiếp tục scan
                elif row == 1 and self._row1_occupied:
                    self._log_once(
                        "S2A_SCAN_SKIP_ROW1",
                        f"[S2A SCAN] S4 trigger @ {trigger_pos:.1f}mm trùng row1 đang occupied "
                        f"→ bỏ qua, tiếp tục scan tìm row 2+"
                    )
                else:
                    target_mm = self.config.iny_output_zones[row][2]
                    self._output_target_pos = target_mm
                    self._output_row = row
                    self.get_logger().info(
                        f"[S2A SCAN] S4 ON @ {trigger_pos:.1f}mm → "
                        f"row{row} ({target_mm:.0f}mm)"
                    )
                    self._stop(2)
                    self._enter_in(SystemState.S2A_INY_TARGETROW)
                    return

        # ── S6 ON fallback: arrived scan_target mà S4 không trigger row nào ─
        # (S6 OFF không vào nhánh này — đã return ở khối 2-phase phía trên)
        timed_out = time.time() > self._step_timeout_in
        at_target = self._arrived(2) or iny >= scan_target - 2.0
        if timed_out or at_target:
            self._stop(2)
            if self.sensor(S4_SCAN_STACK_P1):
                row_at_scan_end = self._zone_to_row(iny, self.config.iny_output_zones)
                if row_at_scan_end is not None and not (row_at_scan_end == 1 and self._row1_occupied):
                    target_mm = self.config.iny_output_zones[row_at_scan_end][2]
                    self._output_target_pos = target_mm
                    self._output_row = row_at_scan_end
                    self.get_logger().warn(
                        f"[S2A SCAN] Reached row{row_at_scan_end} zone @ {iny:.1f}mm with S4 ON "
                        f"but no new rising edge → row{row_at_scan_end} ({target_mm:.0f}mm)"
                    )
                    self._enter_in(SystemState.S2A_INY_TARGETROW)
                    return
            target_mm = self.config.iny_output_zones[1][2]
            self.get_logger().warn(
                f"[S2A SCAN] S4 không trigger trong S6=ON → fallback row1 ({target_mm:.0f}mm) "
                "— sẽ pause sau khi hoàn tất để kiểm tra S4/S6"
            )
            self._s4s6_mismatch_seen = True
            self._notify(
                'warn',
                'CẢNH BÁO: S4/S6 mismatch',
                'Stack báo có khay (S6 ON) nhưng scan không thấy row nào — sẽ STOP sau lần đặt này để kiểm tra cảm biến S4 hoặc S6.'
            )
            self._output_target_pos = target_mm
            self._output_row = 1
            self._enter_in(SystemState.S2A_INY_TARGETROW)
            return

        self._log_once(
            "S2A_SCAN",
            f"[S2A SCAN] InY {iny:.0f}mm S4={'ON' if self.sensor(S4_SCAN_STACK_P1) else 'OFF'} "
            f"armed={getattr(self, '_s4_armed_out', False)} S6={self._s6_snapshot}"
        )

    def _s2a_retry_scan_home(self):
        """
        Recovery khi S2A scan thất bại: đưa InY về home rồi quay lại jog output.
        Có timeout (move_timeout) → ERROR nếu InY không về được home.
        Verify _at_position(2, iny_home) hoặc _pos đọc về ≤ iny_home+2 trước
        khi chuyển state.
        """
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_home)
            if not ok:
                self._enter_in(SystemState.ERROR)
            self._cmd_sent_in = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._error("Homing INY sau lỗi timeout")
            elif self._arrived(2) or (self._pos(2) <= self.config.iny_home + 2.0):
                self._cmd_sent_in = False
                self._enter_in(SystemState.S2A_INY_JOG_OUTPUT)

    def _s2a_iny_targetrow(self):
        """
        STATE 2A bước A8: di chuyển InY đến `_output_target_pos` (đã xác định ở
        bước jog_output) với vận tốc iny_row_vel. Khi đến đích → retract Cyl1
        (thả khay cũ vào output stack tại row trống).
        CAO RỦI RO: verify _at_position(2, target) trước khi nhả — sai vị trí
        = thả khay sai row, có thể đè khay khác hoặc rơi ngoài stack.
        Pre-check (row1 only): _check_row1_interlock() — chặn nếu Cyl3 chưa retract.
        Next: S2A_WAIT_CYL1_RET.
        """
        if not self._cmd_sent_in:
            # ROW1 INTERLOCK — chỉ cho phép move khi Cyl3 đã retract (S15 ON)
            # và không có inconsistent state (S16 ON + S6 OFF).
            if self._output_row == 1:
                ok_il, reason = self._check_row1_interlock()
                if not ok_il:
                    self._log_once("S2A_ROW1_IL", f"[S2A] BLOCK row1: {reason} — chờ điều kiện")
                    return
            # Chỉ vào đây qua path S6=OFF (target đã set, chưa gửi move)
            target = self._output_target_pos
            if target <= 0:
                self.get_logger().error("output_target_pos chưa set → skip")
                self._enter_in(SystemState.S2A_WAIT_CYL1_RET)
                return
            ok = self._nb_move(2, target, vel=self.config.iny_row_vel, continuous_update=True)
            if not ok:
                self.get_logger().warn(f"S2A output_row: move {target:.1f}mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout

        if time.time() > self._step_timeout_in:
            self._cmd_sent_in = False
        # [BLOCKING-FIX] CAO — Cyl1 retract sẽ thả khay; sai row = thả sai vị trí
        elif self._arrived(2) and self._at_position(2, self._output_target_pos):
            self.get_logger().info(
                f"[S2A] InY tới row{self._output_row} "
                f"({self._output_target_pos:.0f}mm) → Cyl1 RETRACT"
            )
            self._cyl1_retract()
            self._enter_in(SystemState.S2A_WAIT_CYL1_RET)

    def _s2a_wait_cyl1_ret(self):
        """
        STATE 2A bước A9: chờ S9 (Cyl1 Retracted) ON xác nhận đã thả khay vào stack.
        Retry _cyl1_retract() mỗi 3s nếu sensor chưa đúng.
        Next: S2A_CYL3_EXTEND (cố định khay bằng Cyl3 trước khi InY rút).
        """
        cyl1_ret, = self._snap(S9_CYL1_RETRACTED)
        if not self._cmd_sent_in:
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
            self._cmd_sent_in    = True
        if cyl1_ret:
            self.get_logger().info("A9: S9 ON — da tha khay")
            self._enter_in(SystemState.S2A_CYL3_EXTEND)
            return
        if time.time() > self._cyl_retry_t:
            self.get_logger().info("Retry Cyl1 retract")
            self._cyl1_retract()
            self._cyl_retry_t = time.time() + 3.0
        self._log_once("S2A_S13", "A9: Cho S9 ON")

    def _s2a_cyl3_extend(self):
        """
        STATE 2A bước A9b: extend Cyl3 (ch6=ON, ch7=OFF) cố định khay tại Tray Pos1.
        Pre-condition: S6_CHECK_TRAY_P1 ON (xác nhận khay đã sit).
        Bắn lệnh extend rồi chuyển NGAY sang S2A_INY_FINAL — InY về home song song với Cyl3 đang đẩy ra.
        Nếu cyl3_present=False (hardware chưa đấu) → skip thẳng sang S2A_INY_FINAL.
        """
        if not self._conf('cyl3_present', True):
            self.get_logger().info("[S2A] Cyl3 disabled (cyl3_present=false) → skip extend, vào S2A_INY_FINAL")
            self._enter_in(SystemState.S2A_INY_FINAL)
            return
        if not self.sensor(S6_CHECK_TRAY_P1):
            self._log_once("S2A_A9B_WAIT_S6", "A9b: cho S6 ON truoc khi extend Cyl3")
            return
        # State 2 recheck S6 ON → clear S13/S14 latch trước khi extend.
        # Why: latch chỉ release khi (a) manual GUI extend hoặc (b) STATE 2 rerun
        # tới đây với S6 ON xác nhận có khay cần giữ.
        if self._cyl3_s13s14_latch:
            self._cyl3_s13s14_latch = False
            self.get_logger().info("[S2A] S6 ON → S13/S14 latch cleared (State 2 recheck)")
        self._cyl3_extend()
        self.get_logger().info("[S2A] Cyl3 EXTEND (cố định khay) → InY về home song song")
        self._enter_in(SystemState.S2A_INY_FINAL)

    def _s2a_iny_final(self):
        """
        STATE 2A bước A10: đưa InY về iny_home để chuẩn bị InX rút.
        Next: S2A_WAIT_NEW_TRAY.
        """
        if not self._cmd_sent_in:
            ok = self._nb_move(2, self.config.iny_home, vel=200)
            if not ok:
                self._log_once("S2A_A10_FAIL", "S2A A10: INY 10mm fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            # [BLOCKING-FIX] verify _at_position(2, iny_home)
            elif self._arrived(2) and self._at_position(2, self.config.iny_home):
                self._enter_in(SystemState.S2A_WAIT_NEW_TRAY)

    def _s2a_wait_new_tray(self):
        """
        STATE 2A bước A11: InX về inx_safe (-60mm) để giải phóng workspace.
        Pre-condition: InY safe.
        Next: S2A_COMPLETE.
        """
        if not self._iny_safe():
            self._log_once("S2A_INX20", "A11: Cho INY safe")
            return
        if not self._cmd_sent_in:
            ok = self._nb_move(1, self.config.inx_safe, vel=200)
            if not ok:
                self._log_once("S2A_A11_FAIL", "S2A A11: INX safe fail")
                return
            self._cmd_sent_in     = True
            self._step_timeout_in = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_in:
                self._cmd_sent_in = False
            # [BLOCKING-FIX] verify _at_position(1, inx_safe)
            elif self._arrived(1) and self._at_position(1, self.config.inx_safe):
                self._enter_in(SystemState.S2A_COMPLETE)

    def _s2a_complete(self):
        if not self._cmd_sent_in:
            self._input_tray_done = False
            self._s2_output_full_accept = False
            self._pub_cartridge_busy(False)
            self._notify('info', 'STATE 2 COMPLETE', 'Da rut khay ra')
            self._cmd_sent_in = True

        # Nếu lần STATE 2 này dùng fallback row1 do S4/S6 mismatch → PAUSE buộc operator kiểm tra
        if self._s4s6_mismatch_seen:
            self._s4s6_mismatch_seen = False  # clear để lần kế tiếp không re-pause
            self._system_paused = True
            self.get_logger().warn("[S2A] PAUSE: S4/S6 mismatch — operator phải kiểm tra cảm biến rồi RESUME")
            self._notify(
                'error',
                'STOP: Kiểm tra cảm biến S4/S6',
                'STATE 2 đã hoàn tất nhưng phát hiện S4/S6 mismatch trong scan — hệ thống PAUSE. Kiểm tra cảm biến S4 (scan stack) hoặc S6 (check tray pos1) rồi nhấn RESUME.'
            )

        self._drain_armed_after_s2 = (
            self.operation_mode in ('auto', 'ai')
            and getattr(self, '_system_running', False)
            and self._state1_enabled
        )
        self.get_logger().info(
            "State2 done -> IDLE (cho check S123 de chay State 1 cap khay tiep, "
            f"drain_armed={self._drain_armed_after_s2})"
        )
        self._enter_in(SystemState.IDLE)

    # ══════════════════════════════════════════════════════════════
    # STATE 3 — Cấp khay thành phẩm  (giữ nguyên logic gốc)
    # ══════════════════════════════════════════════════════════════

    # ══════════════════════════════════════════════════════════════
    # WARNING — Pos2 INTERLOCK (S3 prerequisite)
    #   Servo3 (Platform) KHÔNG được phép chạy khi cụm OutX/OutY còn ở
    #   vùng workspace. Phải verify OutX ≤ outx_home+5 AND OutY ≤
    #   outy_target1+5 trước khi Servo3 di chuyển — tránh va chạm
    #   trục Servo3 với cụm Y.
    #   Đừng nới lỏng tolerance hoặc skip _at_position verify.
    # ══════════════════════════════════════════════════════════════
    def _s3_check_outxy_safe(self):
        """
        STATE 3 bước đầu: đảm bảo OutX về `outx_home` AND OutY về `outy_target1`
        trước khi Servo3 di chuyển — tránh va chạm trục Z với cụm output.
        Verify cả 2 servo bằng _arrived AND _at_position.
        Timeout fallback: vẫn cho tiếp tục (warn log) — vì OutX/OutY ở "near home"
        thường an toàn cho Servo3.
        Drive warm-up (FAS quirk): flush stop_motion_task + đợi 1s trước motion đầu.
        Next: S3_CHECK_S17.
        """
        # ── Drive warm-up gate (FAS firmware quirk) ──────────────────
        if self._drive_warm_t < 0.0:
            for sid in list(self.servos):
                self._stop(sid)
            self._drive_warm_t = time.time() + 1.0
            self.get_logger().info("S3: drive warm-up (stop_motion_task + 1s settle)")
            return
        if self._drive_warm_t > 0.0:
            if time.time() < self._drive_warm_t:
                return
            self._drive_warm_t = 0.0
        # ─────────────────────────────────────────────────────────────

        # busy flag publish 1 lần khi entry
        if not self._cmd_sent_s3 and not getattr(self, '_s3_outy_safe_done', False):
            self._pub_cartridge_busy(True)
            self._pub_cartridge_pos2_busy(True)
        cfg = self.config
        ox = self._pos(4)
        oy = self._pos(5)

        # Nếu State 4 đang chạy:
        if self.state_s4 != SystemState.IDLE:
            # Cho phép chạy song song State 3 nếu OutX đã ra xa khu vực feed (ox <= 150mm)
            if ox is not None and ox <= 150.0:
                self.get_logger().info("[S3] State 4 đang hoạt động nhưng OutX ở vị trí an toàn (<=150mm) -> Chạy song song, bỏ qua OutX/Y standby")
                self._s3_outy_safe_done = False
                self._enter_s3(SystemState.S3_SERVO3_TARGET1)
                return
            else:
                self._log_once("S3_WAIT_S4_SAFE", "S3 chờ State 4 di chuyển OutX ra xa vùng feed (>150mm)")
                return

        ox_safe = ox is None or ox <= cfg.outx_home + 5.0
        oy_safe = oy is None or oy <= cfg.outy_target1 + 5.0

        # ── SEQUENTIAL: OutY safe TRƯỚC, sau đó OutX home ──
        # Rule "tuần tự không chạy cùng lúc OUTX/OUTY" (giống InX/InY ở S1).
        # _s3_outy_safe_done = flag nội bộ giữa 2 step để dùng chung _cmd_sent_s3.
        if not oy_safe and not getattr(self, '_s3_outy_safe_done', False):
            # Step 1: OutY → outy_target1
            if not self._cmd_sent_s3:
                if not self._nb_move(5, cfg.outy_target1):
                    return
                self._cmd_sent_s3     = True
                self._step_timeout_s3 = time.time() + self.config.move_timeout
            else:
                if time.time() > self._step_timeout_s3:
                    # timeout → bỏ qua step 1, sang step 2 (preserve lenient
                    # behavior cũ — outx_home thường vẫn an toàn cho Servo3)
                    self._s3_outy_safe_done = True
                    self._cmd_sent_s3 = False
                elif self._arrived(5) and self._at_position(5, cfg.outy_target1):
                    self._s3_outy_safe_done = True
                    self._cmd_sent_s3 = False
                else:
                    self._log_once("S3_OUTY", "Step1: cho OutY ve target1 truoc")
            return

        if not ox_safe:
            # Step 2: OutX → outx_home (OutY đã safe hoặc skip do timeout)
            if not self._cmd_sent_s3:
                if not self._nb_move(4, cfg.outx_home):
                    return
                self._cmd_sent_s3     = True
                self._step_timeout_s3 = time.time() + self.config.move_timeout
            else:
                if time.time() > self._step_timeout_s3:
                    self._s3_outy_safe_done = False
                    self._enter_s3(SystemState.S3_OUTX_WAIT)
                elif self._arrived(4) and self._at_position(4, cfg.outx_home):
                    self._s3_outy_safe_done = False
                    self._enter_s3(SystemState.S3_OUTX_WAIT)
                else:
                    self._log_once("S3_OUTX", "Step2: cho OutX ve home")
            return

        # Both safe → tới OutX wait 100mm (idle waitdone tray)
        self._s3_outy_safe_done = False
        self._enter_s3(SystemState.S3_OUTX_WAIT)

    def _s3_outx_wait(self):
        """
        STATE 3: OutX → outx_target1 (100mm) — vị trí idle waitdone tray.
        Verify _at_position(4, outx_target1) trước khi sang S3_SERVO3_TARGET1.
        """
        cfg = self.config
        if not self._cmd_sent_s3:
            ok = self._nb_move(4, cfg.outx_target1, vel=140)
            if not ok:
                self._log_once("S3_OXW_FAIL", "S3 outx_wait fail")
                return
            self._cmd_sent_s3     = True
            self._step_timeout_s3 = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_s3:
                self.get_logger().warn("[S3] OUTX_WAIT timeout — resend lệnh")
                self._cmd_sent_s3 = False
            elif self._arrived(4) and self._at_position(4, cfg.outx_target1):
                self._enter_s3(SystemState.S3_SERVO3_TARGET1)
    
    def _s3_servo3_target1(self):
        """
        STATE 3 bước 2: quyết định vị trí Servo3 trước khi feed, theo S17 (Platform):
          - S17 ON và KHÔNG bị ép về home (_s3_force_home=False): GIỮ NGUYÊN vị trí
            Servo3 hiện tại → đi thẳng S3_CHECK_S17 → feed tiếp tới servo3_target2
            (cấp khay kế trong cùng chồng, KHÔNG reset về 0 — 1 chồng tới 10 khay).
          - S17 OFF hoặc _s3_force_home=True: đưa Servo3 về servo3_home (0mm).
            Nếu _s3_finish_after_home_check=True thì đọc lại S17 rồi complete,
            không chờ khay mới.
        Verify _arrived AND _at_position(3, servo3_home) trước transition.
        Timeout → resend (KHÔNG vào ERROR — tránh clear zero_offset / re-home cả 5 servo).
        """
        cfg = self.config
        # S17 ON + không ép về home → giữ vị trí Servo3, đẩy tiếp (continue feed)
        if self.sensor(S17_PLATFORM) and not self._s3_force_home:
            self._cmd_sent_s3 = False
            self._enter_s3(SystemState.S3_CHECK_S17)
            return

        # S17 OFF hoặc overshoot 850 → đưa Servo3 về home 0mm để nhận/xét lại chồng
        if not self._cmd_sent_s3:
            ok = self._nb_move(3, cfg.servo3_home)
            if not ok:
                self._log_once("S3_T1_FAIL", "S3 target1 fail")
                return
            self._cmd_sent_s3     = True
            self._step_timeout_s3 = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_s3:
                # Mirror pattern S1/S2: resend thay vì ERROR (ERROR clear zero_offset
                # → ép re-home cả 5 servo, quá khắc nghiệt khi commissioning Pos2)
                self.get_logger().warn("[S3] SERVO3_TARGET1 timeout — resend lệnh")
                self._cmd_sent_s3 = False
            # [BLOCKING-FIX] verify _at_position(3, servo3_home)
            elif self._arrived(3) and self._at_position(3, cfg.servo3_home):
                self._cmd_sent_s3   = False
                self._s3_force_home = False   # đã về home, clear cờ overshoot
                if self._s3_finish_after_home_check:
                    self._s3_finish_after_home_check = False
                    s17_now = self.sensor(S17_PLATFORM)
                    if s17_now:
                        self._cyl4_extend()
                    else:
                        self._cyl4_retract()
                    self.get_logger().info(
                        f"[S3] Servo3 returned home after target2 empty; S17={'ON' if s17_now else 'OFF'}"
                    )
                    self._enter_s3(SystemState.S3_COMPLETE)
                    return
                self._enter_s3(SystemState.S3_CHECK_S17)

    def _s3_check_s17(self):
        """
        Check S17 (Platform Tray) — có khay trên Platform của Servo3 chưa.
        - S17 OFF: chuyển sang S3_WAIT_S17 (chờ operator/băng tải cấp khay).
        - S17 ON: confirm 2s rồi sang S3_CYL4_EXTEND.
        Cylinder 4 phải xác nhận S26 ON + S25 OFF trước khi Servo 3 feed.
        """
        if not self.sensor(S17_PLATFORM):
            key = "S3_CHECK_S17_OFF"
            if key not in self._guide_logged:
                self._guide_logged.add(key)
                self.get_logger().warn(
                    "[S3] S17 OFF at check — hết khay/chưa cấp khay Platform, chờ S17 ON"
                )
                self._notify_step(
                    'warn',
                    'STATE 3',
                    'Check S17',
                    f'{self._sensor_label(S17_PLATFORM)} OFF — hết khay hoặc chưa cấp khay lên Platform',
                    check=['khay trên Platform Servo3', self._sensor_label(S17_PLATFORM)],
                    action=['Cấp khay lên Platform', 'Khi S17 ON hệ thống sẽ tiếp tục STATE 3']
                )
            self._cyl4_retract()
            self._enter_s3(SystemState.S3_WAIT_S17)
            return

        self._guide_logged.discard("S3_IDLE_S17_OFF")
        self._guide_logged.discard("S3_CHECK_S17_OFF")

        if not getattr(self, '_step_start_s3', 0.0):
            self._step_start_s3 = time.time()
            self._log_once("S3_CHECK_S17", "S17 ON -> Confirming 2s...")

        if time.time() - self._step_start_s3 >= 2.0:
            self.get_logger().info("[S3] S17 confirmed for 2s -> Extend Cyl4")
            self._enter_s3(SystemState.S3_CYL4_EXTEND)

    def _s3_wait_s17(self):
        """
        Chờ S17 ON (có khay trên Platform). Khi ON → S3_CHECK_S17 (confirm 2s).
        Nếu S17 OFF mà Servo 3 ở vị trí khác home (0mm) -> quay về home chờ.
        Không có timeout — chờ vô hạn cho đến khi operator cấp khay.
        """
        cfg = self.config
        p3 = self._pos(3)
        if self.sensor(S17_PLATFORM):
            self._guide_logged.discard("S3_IDLE_S17_OFF")
            self._guide_logged.discard("S3_CHECK_S17_OFF")
            self._notify('info', 'Da phat hien khay', 'S17 ON — confirm 2s')
            self._enter_s3(SystemState.S3_CHECK_S17)
            return

        # Platform trống: giữ Cylinder 4 ở trạng thái retract. Cross-check
        # S25 ON + S26 OFF; nếu chưa đạt thì retry mỗi 3 giây.
        cyl4_ret, cyl4_ext = self._snap(S25_CYL4_RETRACTED, S26_CYL4_EXTENDED)
        now = time.time()
        if not (cyl4_ret and not cyl4_ext):
            if not self._cyl4_retract_sent or now - self._cyl4_retry_t >= 3.0:
                self._cyl4_retract()
                self._cyl4_retract_sent = True
                self._cyl4_retry_t = now
            self._log_once(
                "S3_WAIT_CYL4_RET",
                f"Cho Cyl4 RETRACT: S25={int(cyl4_ret)} S26={int(cyl4_ext)}"
            )
        else:
            self._cyl4_retract_sent = False

        # Nếu S17 OFF mà Servo 3 khác vị trí home (0mm) -> quay về home chờ
        if p3 is not None and not self._at_position(3, cfg.servo3_home, tol=5.0):
            if not self._cmd_sent_s3:
                self.get_logger().warn(f"[S3] S17 OFF nhưng Servo3 @ {p3:.1f}mm khác home {cfg.servo3_home}mm -> lùi về chờ")
                if self._nb_move(3, cfg.servo3_home):
                    self._cmd_sent_s3 = True
            else:
                if self._arrived(3) and self._at_position(3, cfg.servo3_home):
                    self._cmd_sent_s3 = False
        else:
            if self._cmd_sent_s3:
                self._cmd_sent_s3 = False
            self._log_once("S3_WAIT_S17", "Cho S17 ON — cap khay len Platform")

    def _s3_cyl4_extend(self):
        """
        STATE 3: S17 đã ổn định → extend Cylinder 4 giữ khay.
        Chỉ cho Servo 3 feed khi cross-check S26 ON + S25 OFF.
        Nếu S17 mất trước khi xác nhận xong, retract Cyl4 và quay lại chờ khay.
        Retry lệnh EXTEND mỗi 3 giây nếu feedback chưa đúng.
        """
        if not self.sensor(S17_PLATFORM):
            self.get_logger().warn("[S3] S17 OFF trước khi Cyl4 extend xong -> RETRACT và chờ lại")
            self._cyl4_retract()
            self._enter_s3(SystemState.S3_WAIT_S17)
            return

        cyl4_ret, cyl4_ext = self._snap(S25_CYL4_RETRACTED, S26_CYL4_EXTENDED)
        if cyl4_ext and not cyl4_ret:
            self._cyl4_retract_sent = False
            self.get_logger().info("[S3] Cyl4 confirmed EXTENDED: S26=ON S25=OFF -> Servo3 FEED")
            self._enter_s3(SystemState.S3_SERVO3_FEED)
            return

        now = time.time()
        if not self._cmd_sent_s3 or now - self._cyl4_retry_t >= 3.0:
            if self._cyl4_extend():
                self._cmd_sent_s3 = True
                self._cyl4_retry_t = now
                if not self._step_start_s3:
                    self._step_start_s3 = now
            else:
                self._log_once("S3_CYL4_IO", "Cyl4 EXTEND chưa gửi được — chờ IO Module 2")

        if (self._cmd_sent_s3
                and self._step_start_s3
                and now - self._step_start_s3 > CYLINDER_TIMEOUT_S):
            self._notify_step('warn', 'STATE 3', 'Cyl4 extend',
                f'{self._sensor_label(26)} chưa ON hoặc {self._sensor_label(25)} chưa OFF '
                f'sau {CYLINDER_TIMEOUT_S:.0f}s — vẫn đang retry mỗi 3s',
                check=['van Cyl4 extend', 'áp khí',
                       f'dây {self._sensor_label(25)}/{self._sensor_label(26)}',
                       'mapping output Cyl4 trên CPX 41'],
                action=['Kiểm tra Cyl4 extend',
                        'Nhấn STOP nếu cần abort State 3'])
            self.get_logger().warn(
                f"[S3] Cyl4 EXTEND timeout (S25={int(cyl4_ret)} S26={int(cyl4_ext)}) "
                "— keep waiting/retrying"
            )
            self._step_start_s3 = now

        self._log_once(
            "S3_CYL4_EXTEND",
            f"Cho Cyl4 EXTEND: S25={int(cyl4_ret)} S26={int(cyl4_ext)}"
        )

    def _s3_wait_gui_confirm(self):
        self._enter_s3(SystemState.S3_SERVO3_FEED)

    def _s3_servo3_feed(self):
        """
        STATE 3 bước feed: Servo3 → servo3_target2 (400mm) đẩy khay vào robot.
        Vận tốc = servo3_feed_velocity. Vừa đi vừa monitor S18 (Feed OK).
        2 nhánh thoát:
          1. S18 ON sớm → STOP servo3 + S3_WAIT_S18 (confirm 5s).
          2. Servo3 tới giới hạn (target2) + S18 vẫn OFF → khay kẹt → quay về
             servo3_home thử lại. Verify _at_position(3, target2) trước fallback.
        Timeout move_timeout → _error().
        """
        cfg = self.config
        if not self._cmd_sent_s3:
            ok = self._nb_move(3, cfg.servo3_target2, vel=int(cfg.servo3_feed_velocity))
            if not ok:
                # _nb_move reject (pos > servo_limits[3] HOẶC Servo3 chưa homed) chỉ
                # log ROS — notify GUI để operator thấy ngay thay vì state đứng im.
                # Gate bằng _guide_logged (như _log_once) để fire đúng 1 lần, không spam 20Hz.
                if "S3_FEED_FAIL" not in self._guide_logged:
                    limit = self._conf('servo_limits', {}).get(3, 999.0)
                    self._notify_step('error', 'STATE 3', 'Servo3 FEED',
                        f'Lệnh đẩy khay bị từ chối — Servo3 không di chuyển '
                        f'(đích servo3_target2={cfg.servo3_target2}mm)',
                        check=[f'servo_limits[3]={limit}mm có ≥ servo3_target2={cfg.servo3_target2}mm không',
                               'Servo3 (S3) đã HOMING chưa (zero_offset)'],
                        action=['Sửa servo_limits[3] ≥ servo3_target2 trong cartridge_config.yaml rồi restart node',
                                'Hoặc HOMING lại Servo3 (S3)'])
                self._log_once("S3_FEED_FAIL", "S3 feed fail")
                return
            self._cmd_sent_s3     = True
            self._step_start_s3   = time.time()
            self._step_timeout_s3 = time.time() + self.config.move_timeout
            self.get_logger().info(f"[S3] Servo3 pushing to {cfg.servo3_target2}mm max until S18 ON")
        else:
            if self.sensor(S18_FEED_OK):
                self._stop(3)
                self._s3_output_loaded = True
                self.get_logger().info("[S3] S18 ON -> Dung Servo 3 som")
                self._enter_s3(SystemState.S3_WAIT_S18)
                return

            if time.time() > self._step_timeout_s3:
                self.get_logger().warn("[S3] SERVO3_FEED timeout — resend lệnh")
                self._cmd_sent_s3 = False
            # [BLOCKING-FIX] verify _at_position(3, servo3_target2). Nhánh này chỉ
            # vào khi Servo3 đã tới giới hạn target2 mà S18 chưa ON → cần verify thật
            # đã ở target2, tránh đọc nhầm drive flag rồi quay về 10mm vô lý.
            elif (self._arrived(3) and self._at_position(3, cfg.servo3_target2)
                  and time.time() - self._step_start_s3 > 0.5):
                self._s3_output_loaded = False
                self._s3_finish_after_home_check = True
                self.get_logger().info("[S3] Servo3 tới target2 nhưng S18 OFF -> hết khay tại vị trí cuối, về home check S17")
                self._notify('info', 'State 3', 'Servo3 tới vị trí khay cuối nhưng S18 OFF — về home check S17')
                self._s3_force_home = True
                self._enter_s3(SystemState.S3_SERVO3_TARGET1)

    def _s3_wait_s18(self):
        """
        Confirm S18 ON liên tục trong 2s sau khi servo3 dừng. Nếu S18 OFF
        giữa chừng → quay lại S3_SERVO3_FEED (đẩy thêm). Đủ 2s S18 ON → S3_COMPLETE.
        """
        if not self.sensor(S18_FEED_OK):
            self.get_logger().warn("[S3] S18 OFF during confirm -> Resume FEED")
            self._enter_s3(SystemState.S3_SERVO3_FEED)
            return

        if not getattr(self, '_step_start_s3', 0.0):
            self._step_start_s3 = time.time()
            self._log_once("S3_WAIT_S18", "S18 ON -> Confirming 2s...")

        if time.time() - self._step_start_s3 >= 2.0:
            self._s3_output_loaded = True
            self.get_logger().info("[S3] S18 confirmed 2s -> Feed Success")
            self._enter_s3(SystemState.S3_COMPLETE)
            return

    def _s3_complete(self):
        self._pub_cartridge_busy(False)
        self._pub_cartridge_pos2_busy(False)
        if self._s3_output_loaded:
            self.pub_newtray_output.publish(Bool(data=True))
            self._notify('info', 'State 3 done', 'Cap khay thanh pham thanh cong')
            self.get_logger().info("[S3] COMPLETE — pub new_trayoutput_loaded")
        else:
            self.pub_newtray_output.publish(Bool(data=False))
            self._notify('info', 'State 3 done', 'Khong co khay tai vi tri cap — da ve home check S17')
            self.get_logger().info("[S3] COMPLETE — no tray loaded at S18")
        self._s3_output_loaded = False
        self._s3_finish_after_home_check = False
        self._enter_s3(SystemState.IDLE)

    # ══════════════════════════════════════════════════════════════
    # STATE 4 — Thay khay output  (giữ nguyên logic gốc)
    # ══════════════════════════════════════════════════════════════
    # WARNING — Pos2 INTERLOCK (S4 prerequisite)
    #   OutX KHÔNG được phép di chuyển khi OutY > outy_safe_zone+5
    #   (15mm). Tương tự rule InX/InY (S1/S2): OutY phải nâng cao
    #   tới safe zone trước khi OutX dịch ngang — tránh va 2 trục
    #   output.
    # ══════════════════════════════════════════════════════════════

    def _s4_check_outy_safe(self):
        """
        STATE 4 bước đầu: đảm bảo OutY về `outy_target1` (10mm = safe clearance)
        trước khi OutX di chuyển vào (tránh va chạm 2 trục output).
        Verify _at_position(5, outy_target1) bằng _at_position helper.
        Drive warm-up (FAS quirk): flush stop_motion_task + đợi 1s trước motion đầu.
        Next: S4_OUTX_TARGET2.
        """
        # ── Drive warm-up gate (FAS firmware quirk) ──────────────────
        if self._drive_warm_t < 0.0:
            for sid in list(self.servos):
                self._stop(sid)
            self._drive_warm_t = time.time() + 1.0
            self.get_logger().info("S4: drive warm-up (stop_motion_task + 1s settle)")
            return
        if self._drive_warm_t > 0.0:
            if time.time() < self._drive_warm_t:
                return
            self._drive_warm_t = 0.0
        # ─────────────────────────────────────────────────────────────

        if not self._s4_hold_cyl4_if_s17_on():
            return

        if not self._cmd_sent_s4:
            self._pub_cartridge_busy(True)
            self._pub_cartridge_pos2_busy(True)
        cfg = self.config
        oy = self._pos(5)
        if oy is not None and oy <= cfg.outy_safe_zone + 5:
            self._enter_s4(SystemState.S4_OUTX_TARGET2)
        else:
            if not self._cmd_sent_s4:
                ok = self._nb_move(5, cfg.outy_target1)
                if not ok:
                    self._log_once("S4_SAFE_FAIL", "S4 outy home fail")
                    return
                self._cmd_sent_s4     = True
                self._step_timeout_s4 = time.time() + self.config.move_timeout
            else:
                if time.time() > self._step_timeout_s4:
                    self._cmd_sent_s4 = False
                # [BLOCKING-FIX] verify _at_position(5, outy_target1)
                elif self._arrived(5) and self._at_position(5, cfg.outy_target1):
                    self._enter_s4(SystemState.S4_OUTX_TARGET2)
            self._log_once("S4_SAFE", "Cho OutY ve safe zone")

    def _s4_outx_target2(self):
        """
        STATE 4: OutX → outx_target2 (400mm) — vị trí lấy khay thành phẩm từ Pos 1.
        Gate _outy_safe() re-check MỖI TICK (mirror rule _iny_safe của S1/S2 —
        không chỉ tin thứ tự bước, phòng OutY drift giữa chừng).
        Timeout → resend (pattern S1/S2). Verify _at_position trước khi sang
        S4_OUTY_PICK (Cyl2 đã được retract ở bước đầu STATE 4).
        """
        cfg = self.config
        if not self._s4_hold_cyl4_if_s17_on():
            return
        if not self._outy_safe():
            self._log_once("S4_OX2_OYSAFE", "[S4] OutY chưa safe — khóa OutX")
            return
        if not self._cmd_sent_s4:
            ok = self._nb_move(4, cfg.outx_target2, vel = 200)
            if not ok:
                self._log_once("S4_OX2_FAIL", "S4 outx_target2 fail")
                return
            self._cmd_sent_s4     = True
            self._step_timeout_s4 = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_s4:
                self.get_logger().warn("[S4] OUTX_TARGET2 timeout — resend lệnh")
                self._cmd_sent_s4 = False
            # [BLOCKING-FIX] verify _at_position(4, outx_target2)
            elif self._arrived(4) and self._at_position(4, cfg.outx_target2):
                self._enter_s4(SystemState.S4_OUTY_PICK)

    def _s4_cyl2_retract_ready(self):
        """
        Bước đầu STATE 4: ép Cyl2 RETRACT trước khi OutY/OutX di chuyển.
        Theo mapping hiện tại:
          - S21 ON  = Cyl2 retracted
          - S22 OFF = Cyl2 không còn extended
        Nếu chưa đúng thì gửi Cyl2 RETRACT và retry mỗi 3s, không cho vào
        S4_CHECK_OUTY_SAFE cho tới khi feedback hợp lệ.
        """
        if not self._s4_hold_cyl4_if_s17_on():
            return
        cyl2_ret, cyl2_ext = self._snap(S21_CYL2_RETRACTED, S22_CYL2_EXTENDED)
        if cyl2_ret and not cyl2_ext:
            self.get_logger().info("[S4] Cyl2 RETRACT ready: S21 ON + S22 OFF -> CHECK_OUTY_SAFE")
            self._enter_s4(SystemState.S4_CHECK_OUTY_SAFE)
            return

        if not self._cmd_sent_s4:
            self._cyl2_retract()
            self._cyl2_retry_t  = time.time() + 3.0
            self._step_start_s4 = time.time()
            self._cmd_sent_s4   = True
        elif time.time() > self._cyl2_retry_t:
            self.get_logger().info("[S4] Retry Cyl2 retract at STATE 4 entry")
            self._cyl2_retract()
            self._cyl2_retry_t = time.time() + 3.0

        if time.time() - self._step_start_s4 > CYLINDER_TIMEOUT_S:
            self._notify_step('warn', 'STATE 4', 'Cyl2 retract before pick',
                f'{self._sensor_label(21)} chưa ON hoặc {self._sensor_label(22)} chưa OFF '
                f'sau {CYLINDER_TIMEOUT_S:.0f}s — vẫn đang retry mỗi 3s',
                check=['van Cyl2 retract', 'áp khí',
                       f'dây {self._sensor_label(21)}/{self._sensor_label(22)}'],
                action=['Kiểm tra van/khí nén', 'Hoặc nhấn STOP để abort'])
            self._step_start_s4 = time.time()

        self._log_once("S4_WAIT_CYL2_RET_ENTRY",
                       f"Cho Cyl2 RETRACT khi vào STATE 4 | S21={cyl2_ret} S22={cyl2_ext}")

    # ══════════════════════════════════════════════════════════════
    # WARNING — CAO RỦI RO: Cyl2 EXTEND ngay sau khi OutY tới target.
    #   BẮT BUỘC verify _arrived(5) AND _at_position(5, outy_pick_pos)
    #   trước khi sang S4_CYL2_EXTEND. Sai vị trí = Cyl2 đóng nhầm
    #   vào khung gia công / khay khác → hỏng cơ khí.
    #   Tương tự rule S2A_POS_PLACE_TRAY_ROBOT_CYL1 (Pos1).
    # ══════════════════════════════════════════════════════════════
    def _s4_outy_pick(self):
        """
        STATE 4: OutY → outy_pick_pos (100mm) — hạ xuống vị trí kẹp khay.
        CAO RỦI RO: Cyl2 sẽ EXTEND để kẹp khay ngay sau khi đến vị trí; sai vị trí
        = Cyl2 đóng nhầm vào khung gia công. Verify _at_position trước khi sang
        S4_CYL2_EXTEND. Có thể sẽ sửa dựa trên vị trí thực tế

        """
        cfg = self.config
        if not self._s4_hold_cyl4_if_s17_on():
            return
        if not self._cmd_sent_s4:
            ok = self._nb_move(5, cfg.outy_pick_pos, vel =140)
            if not ok:
                self._log_once("S4_PICK_FAIL", "S4 outy_pick fail")
                return
            self._cmd_sent_s4     = True
            self._step_timeout_s4 = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_s4:
                self.get_logger().warn("[S4] OUTY_PICK timeout — resend lệnh")
                self._cmd_sent_s4 = False
            # [BLOCKING-FIX] CAO — Cyl2 sẽ extend kẹp khay; sai vị trí = va chạm
            elif self._arrived(5) and self._at_position(5, cfg.outy_pick_pos):
                self._enter_s4(SystemState.S4_CYL2_EXTEND)

    def _s4_cyl2_extend(self):
        """
        Kích Cyl2 EXTEND kẹp khay output — mirror pattern Cyl1 (_s1_cyl1_extend):
          • Retry lệnh valve mỗi 3s — cover luôn case _cyl2_extend() trả False
            (IO chưa ready) mà trước đây bị bỏ qua return value.
          • Cross-check 2 sensor: S22 ON AND S21 OFF (RULE 9 — không tin 1 sensor).
          • KHÔNG _error theo timeout — notify warn sau CYLINDER_TIMEOUT_S rồi
            TIẾP TỤC retry (operator có thể STOP nếu muốn abort).
        Next: S4_CYL4_RETRACT.
        """
        cyl2_ret, cyl2_ext = self._snap(S21_CYL2_RETRACTED, S22_CYL2_EXTENDED)
        if not self._cmd_sent_s4:
            self._cyl2_extend()
            self._cyl2_retry_t  = time.time() + 3.0
            self._step_start_s4 = time.time()
            self._cmd_sent_s4   = True
        if cyl2_ext and not cyl2_ret:
            self.get_logger().info("[S4] S22 ON + S21 OFF — Cyl2 kẹp khay OK")
            if self._servo3_at_last_tray_position():
                self.get_logger().info("[S4] Servo3 ở target2 khay cuối -> RETRACT Cyl4 để nhả platform")
                self._enter_s4(SystemState.S4_CYL4_RETRACT)
            else:
                self.get_logger().info("[S4] Servo3 chưa ở target2 -> giữ Cyl4 EXTEND, bỏ qua nhả platform")
                self._enter_s4(SystemState.S4_OUTY_TARGET1)
            return
        if time.time() > self._cyl2_retry_t:
            self.get_logger().info("[S4] Retry Cyl2 extend")
            self._cyl2_extend()
            self._cyl2_retry_t = time.time() + 3.0
        if time.time() - self._step_start_s4 > CYLINDER_TIMEOUT_S:
            self._notify_step('warn', 'STATE 4', 'Cyl2 extend',
                f'{self._sensor_label(22)} chưa ON sau {CYLINDER_TIMEOUT_S:.0f}s '
                f'— vẫn đang retry mỗi 3s',
                check=['van Cyl2 extend', 'áp khí',
                       f'dây {self._sensor_label(21)}/{self._sensor_label(22)}',
                       'IO module đã connect chưa'],
                action=['Kiểm tra van/khí nén', 'Hoặc nhấn STOP để abort'])
            self._step_start_s4 = time.time()  # throttle warn — lặp mỗi 15s
        self._log_once("S4_WAIT_S22",
                       f"Cho S22 ON + S21 OFF | S21={cyl2_ret} S22={cyl2_ext}")

    def _s4_cyl4_retract(self):
        """
        Sau khi Cyl2 đã kẹp khay (S22 ON + S21 OFF), thu Cyl4 để nhả platform:
          • Gửi Cyl4 RETRACT.
          • Chỉ đi tiếp khi S25 ON + S26 OFF.
          • Retry mỗi 3s; không hard-error, operator có thể STOP nếu cần.

        Next: S4_OUTY_TARGET1.
        """
        cyl4_ret, cyl4_ext = self._snap(S25_CYL4_RETRACTED, S26_CYL4_EXTENDED)
        if not self._cmd_sent_s4:
            self._cyl4_retract()
            self._cyl4_retry_t  = time.time() + 3.0
            self._step_start_s4 = time.time()
            self._cmd_sent_s4   = True

        if cyl4_ret and not cyl4_ext:
            self.get_logger().info("[S4] S25 ON + S26 OFF — Cyl4 retract OK, platform released")
            self._enter_s4(SystemState.S4_OUTY_TARGET1)
            return

        if time.time() > self._cyl4_retry_t:
            self.get_logger().info("[S4] Retry Cyl4 retract")
            self._cyl4_retract()
            self._cyl4_retry_t = time.time() + 3.0

        if time.time() - self._step_start_s4 > CYLINDER_TIMEOUT_S:
            self._notify_step('warn', 'STATE 4', 'Cyl4 retract',
                f'{self._sensor_label(25)} chưa ON hoặc {self._sensor_label(26)} chưa OFF '
                f'sau {CYLINDER_TIMEOUT_S:.0f}s — vẫn đang retry mỗi 3s',
                check=['van Cyl4 retract', 'áp khí',
                       f'dây {self._sensor_label(25)}/{self._sensor_label(26)}',
                       'Cyl4 có đang bị kẹt cơ khí không'],
                action=['Kiểm tra van/khí nén', 'Hoặc nhấn STOP để abort'])
            self._step_start_s4 = time.time()

        self._log_once("S4_WAIT_S25",
                       f"Cho S25 ON + S26 OFF | S25={cyl4_ret} S26={cyl4_ext}")

    def _s4_outy_target1(self):
        """
        STATE 4: nâng OutY lên outy_target1 (10mm) cùng khay (Cyl2 đang kẹp).
        Next: S4_OUTX_TARGET3 (đẩy OutX về vị trí thả).
        """
        cfg = self.config
        if not self._cmd_sent_s4:
            ok = self._nb_move(5, cfg.outy_target1, vel = 200)
            if not ok:
                self._log_once("S4_OY1_FAIL", "S4 outy_target1 fail")
                return
            self._cmd_sent_s4     = True
            self._step_timeout_s4 = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_s4:
                self.get_logger().warn("[S4] OUTY_TARGET1 timeout — resend lệnh")
                self._cmd_sent_s4 = False
            # [BLOCKING-FIX] verify _at_position(5, outy_target1)
            elif self._arrived(5) and self._at_position(5, cfg.outy_target1):
                self._enter_s4(SystemState.S4_OUTX_TARGET3)

    def _s4_outx_target3(self):
        """
        STATE 4: OutX → outx_target3 (20mm) — vị trí đặt khay output.
        Gate _outy_safe() mỗi tick (OutY đang giữ khay ở target1=10mm — vẫn phải
        nằm trong safe zone trước khi OutX dịch ngang). Timeout → resend.
        Next: S4_CHECK_S19.
        """
        cfg = self.config
        if not self._outy_safe():
            self._log_once("S4_OX3_OYSAFE", "[S4] OutY chưa safe — khóa OutX")
            return
        if not self._cmd_sent_s4:
            ok = self._nb_move(4, cfg.outx_target3, vel = 200)
            if not ok:
                self._log_once("S4_OX3_FAIL", "S4 outx_target3 fail")
                return
            self._cmd_sent_s4     = True
            self._step_timeout_s4 = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_s4:
                self.get_logger().warn("[S4] OUTX_TARGET3 timeout — resend lệnh")
                self._cmd_sent_s4 = False
            # [BLOCKING-FIX] verify _at_position(4, outx_target3)
            elif self._arrived(4) and self._at_position(4, cfg.outx_target3):
                self._enter_s4(SystemState.S4_CHECK_S19)

    def _s4_check_s19(self):
        """
        Check S19 (Check Tray Pos 2) — output stack có khay hay không.
        Chuyển sang S4_CYL5_SYNC để đồng bộ cylinder theo S19 trước khi OutY chạy.
        """
        self._enter_s4(SystemState.S4_CYL5_SYNC)

    def _s4_cyl5_sync(self):
        """
        Đồng bộ Cylinder 5 theo S19 với cross-check hai limit:
          - S19 ON  → EXTEND; chỉ hợp lệ khi S28 ON + S27 OFF, rồi scan S20.
          - S19 OFF → RETRACT; chỉ hợp lệ khi S27 ON + S28 OFF, rồi xuống row 1.
        Nếu S19 đổi trong lúc chờ, lệnh cylinder tự đảo theo trạng thái mới.
        Retry lệnh mỗi 3 giây cho đến khi feedback hợp lệ.
        """
        s19 = self.sensor(S19_CHECK_TRAY_P2)
        cyl5_ret, cyl5_ext = self._snap(S27_CYL5_RETRACTED, S28_CYL5_EXTENDED)

        if s19 and cyl5_ext and not cyl5_ret:
            self.get_logger().info("[S4] S19 ON + Cyl5 EXTENDED (S28=ON S27=OFF) -> scan S20")
            self._outy_jog_start = time.time()
            self._enter_s4(SystemState.S4_OUTY_SCAN_S20)
            return

        if not s19 and cyl5_ret and not cyl5_ext:
            self.get_logger().info("[S4] S19 OFF + Cyl5 RETRACTED (S27=ON S28=OFF) -> row 1")
            self._enter_s4(SystemState.S4_OUTY_ROW1)
            return

        now = time.time()
        if not self._cmd_sent_s4 or now - self._cyl5_retry_t >= 3.0:
            ok = self._cyl5_extend() if s19 else self._cyl5_retract()
            if ok:
                self._cmd_sent_s4 = True
                self._cyl5_retry_t = now
            else:
                self._log_once("S4_CYL5_IO", "Cyl5 chưa gửi được lệnh — chờ IO Module 2")

        target = "EXTEND" if s19 else "RETRACT"
        self._log_once(
            "S4_CYL5_SYNC",
            f"Cho Cyl5 {target}: S19={int(s19)} S27={int(cyl5_ret)} S28={int(cyl5_ext)}"
        )

    def _s4_outy_row1(self):
        """
        Output stack rỗng → di chuyển OutY thẳng đến row 1 (cuối stack)
        ở `outy_output_zones[1][2]`. Next: S4_CYL2_RETRACT (thả khay).
        """
        cfg = self.config
        if not self._cmd_sent_s4:
            # Di chuyen nhanh toi vi tri Target cua Row 1 trong Zone
            target = cfg.outy_output_zones[1][2]
            ok = self._nb_move(5, target, vel = 120)
            if not ok:
                self._log_once("S4_ROW1_FAIL", "S4 outy_row1 fail")
                return
            self._cmd_sent_s4     = True
            self._step_timeout_s4 = time.time() + self.config.move_timeout
        else:
            if time.time() > self._step_timeout_s4:
                self.get_logger().warn("[S4] OUTY_ROW1 timeout — resend lệnh")
                self._cmd_sent_s4 = False
            # [BLOCKING-FIX] verify _at_position(5, row1 target)
            elif (self._arrived(5)
                  and self._at_position(5, cfg.outy_output_zones[1][2])):
                self._enter_s4(SystemState.S4_CYL2_RETRACT)

    def _s4_outy_scan_s20(self):
        """
        Scan OutY tìm row trống trong output stack:
          1. S20 ON khi đã armed → xét zone ngay.
          2. Armed gate re-evaluate mỗi tick:
             - OutX vẫn tại outx_target3 ± position_tolerance (_at_position)
             - OutY ∈ [outy_scan_valid_min_mm, outy_scan_valid_max_mm]
          3. Disarm + warn nếu OutX drift khỏi target giữa lúc scan.

        S20 ON hợp lệ → _zone_to_row(outy_output_zones) → stop(5) →
        S4_OUTY_DROP (DROP tự gửi move — KHÔNG pre-issue ở đây, fix double-issue
        do _enter_s4 reset _cmd_sent_s4).

        S20 ON NGOÀI mọi zone (_zone_to_row → None): xử lý như noise —
        đi cùng nhánh retry/fallback bên dưới, không đoán row.
        Hết hành trình không có edge (hoặc trigger ngoài zone):
          - retry < s4_scan_noise_retry_limit (default 1) → S4_RETRY_SCAN_HOME
          - hết retry → fallback row1 + notify warn (operator kiểm tra S19/S20).
        """
        cfg = self.config

        # SENSOR-FIRST: giống S4 tại scan Pos1. Khi scan đã armed, ưu tiên
        # kiểm tra S20 và gửi STOP trước mọi lần đọc encoder OutY. Feedback
        # encoder FAS có thể cập nhật chậm hơn tín hiệu DI; đọc vị trí trước
        # làm OutY tiếp tục chạy thêm một đoạn dù S20 đã ON ngoài thực tế.
        if (self._cmd_sent_s4
                and self._s20_armed
                and self.sensor(S20_SCAN_STACK_P2)):
            detect_t = time.time()
            self._stop(5)  # dừng cơ khí trước, vị trí chỉ dùng phân row/log
            trigger_pos = self._pos(5)
            if trigger_pos is None:
                trigger_pos = self._pos_cached(5)

            row = (self._zone_to_row(trigger_pos, cfg.outy_output_zones)
                   if trigger_pos is not None else None)
            pos_text = 'N/A' if trigger_pos is None else f'{trigger_pos:.1f}'
            zone_msg = f'row{row}' if row is not None else 'NO_ZONE'
            self.get_logger().warn(
                f'[S4 SENSOR-FIRST] S20 ON -> STOP @ OutY={pos_text}mm ({zone_msg})'
            )

            self._s4_s20_on_logged = True
            if row is not None:
                target_mm = cfg.outy_output_zones[row][2]
                self._notify(
                    'silent_ok', 'POS2 S20 — DETECT/STOP',
                    f'OutY stop={pos_text} mm | Row {row} | Target={target_mm:.1f} mm'
                    f' | Phản ứng={time.time() - detect_t:.3f}s'
                )
                self._outy_jog_pos = target_mm
                self._s4_scan_noise_retry = 0
                self._enter_s4(SystemState.S4_OUTY_DROP)
            else:
                self._notify(
                    'silent_info', 'POS2 S20 — DETECT/STOP NGOÀI ZONE',
                    f'OutY stop={pos_text} mm | Chuyển retry scan an toàn'
                )
                # Không chạy mù sau khi đã STOP; về safe rồi scan lại.
                self._enter_s4(SystemState.S4_RETRY_SCAN_HOME)
            return

        oy = self._pos(5)
        if oy is None:
            return

        scan_row = 2 if self.sensor(S19_CHECK_TRAY_P2) else None
        scan_target = (max(cfg.outy_output_zones[scan_row][0], cfg.outy_output_zones[scan_row][1])
                       if scan_row is not None
                       else getattr(cfg, 'target_scanoutp2', 500.0))

        if not self._cmd_sent_s4:
            ok = self._nb_move(5, scan_target, vel=cfg.outy_search_velocity)
            if not ok:
                self._log_once("S4_SCAN_FAIL", "S4 outy_scan: nb_move fail")
                return
            self._cmd_sent_s4     = True
            self._step_timeout_s4 = time.time() + cfg.move_timeout
            self._s20_armed       = False
            self._s20_prev        = self.sensor(S20_SCAN_STACK_P2)
            self._s4_s20_on_logged = False
            self.get_logger().info(
                f"[S4 SCAN] OutY → {scan_target:.0f}mm vel={cfg.outy_search_velocity} "
                f"S19={'ON' if self.sensor(S19_CHECK_TRAY_P2) else 'OFF'}"
            )

        # ── Armed gate (mirror S1 RULE 3): OutX tại target + OutY trong vùng valid ──
        valid_min = self._conf(
            'outy_scan_valid_min_mm',
            getattr(cfg, 'outy_scan_arm_mm', getattr(cfg, 'outy_safe_zone', 50.0)))
        valid_max = self._conf('outy_scan_valid_max_mm', scan_target)
        outx_at_target = self._at_position(4, cfg.outx_target3)
        oy_in_valid    = valid_min <= oy <= valid_max
        new_armed      = outx_at_target and oy_in_valid

        if self._s20_armed and not outx_at_target:
            ox_now = self._pos(4)
            self._log_once(
                "S4_SCAN_OUTX_DRIFT",
                f"[S4 SCAN] OutX trôi khỏi {cfg.outx_target3}mm "
                f"({ox_now if ox_now is None else f'{ox_now:.2f}'}mm, "
                f"tol={cfg.position_tolerance}mm) — DISARM S20"
            )
        if new_armed and not self._s20_armed:
            self._s20_prev = self.sensor(S20_SCAN_STACK_P2)
        self._s20_armed = new_armed

        s20_now = self.sensor(S20_SCAN_STACK_P2)
        # S20 là NO: chạm khay → OFF sang ON → rising edge
        s20_rising = self._s20_armed and (not self._s20_prev) and s20_now
        if self._s20_armed:
            self._s20_prev = s20_now

        invalid_trigger_pos = None
        forced_row = None
        if self._s20_armed and s20_now:
            trigger_pos = self._pos(5) or oy
            row = self._zone_to_row(trigger_pos, cfg.outy_output_zones)
            if not getattr(self, '_s4_s20_on_logged', False):
                zone_msg = f"row{row}" if row is not None else "NO_ZONE"
                self.get_logger().warn(
                    f"[S4 SCAN] S20 ON detect @ OutY={trigger_pos:.1f}mm ({zone_msg})"
                )
                if row is not None:
                    target_mm = cfg.outy_output_zones[row][2]
                    self._notify(
                        'silent_ok',
                        'POS2 S20 — DETECT',
                        f'OutY={trigger_pos:.1f} mm | Row {row} | Target={target_mm:.1f} mm'
                    )
                else:
                    self._notify(
                        'silent_info',
                        'POS2 S20 — NGOÀI ZONE',
                        f'OutY={trigger_pos:.1f} mm | Không khớp row nào'
                    )
                self._s4_s20_on_logged = True
            if row is None:
                # Trigger rơi khe hở giữa 2 zone / nhiễu → rơi xuống nhánh fail
                # bên dưới NGAY tick này (retry scan / hết retry → fallback row1
                # + notify vì đang giữ khay phải đặt được). Trước đây row=None
                # không thể xảy ra (fallback row1 trong _zone_to_row) — giờ
                # PHẢI xử lý, nếu không sẽ KeyError tại outy_output_zones[row].
                invalid_trigger_pos = trigger_pos
            else:
                target_mm = cfg.outy_output_zones[row][2]
                self._outy_jog_pos = target_mm
                self._s4_scan_noise_retry = 0
                self.get_logger().info(
                    f"[S4 SCAN] S20 ON @ {trigger_pos:.1f}mm → "
                    f"row{row} ({target_mm:.0f}mm)"
                )
                self._stop(5)
                self._enter_s4(SystemState.S4_OUTY_DROP)
                return

        timed_out = time.time() > self._step_timeout_s4
        at_target = self._arrived(5) or oy >= scan_target - 2.0
        if at_target and scan_row is not None:
            row_at_scan_end = self._zone_to_row(oy, cfg.outy_output_zones)
            if row_at_scan_end == scan_row:
                forced_row = scan_row
        if timed_out or at_target or invalid_trigger_pos is not None:
            if invalid_trigger_pos is not None:
                self.get_logger().warn(
                    f"[S4 SCAN] S20 trigger @ {invalid_trigger_pos:.1f}mm NGOÀI mọi zone "
                    f"— xử lý như nhiễu (kiểm tra calib outy_output_zones)"
                )
            self._stop(5)
            if forced_row is not None:
                tgt = cfg.outy_output_zones[forced_row][2]
                self.get_logger().warn(
                    f"[S4 SCAN] Reached row{forced_row} zone @ {oy:.1f}mm without new S20 edge "
                    f"→ place row{forced_row} ({tgt:.0f}mm)"
                )
                self._outy_jog_pos = tgt
                self._s4_scan_noise_retry = 0
                self._enter_s4(SystemState.S4_OUTY_DROP)
                return
            if self._s4_scan_noise_retry < self._conf('s4_scan_noise_retry_limit', 1):
                self._s4_scan_noise_retry += 1
                self._notify('warn', 'S4: Không bắt được S20 hợp lệ',
                             f'Retry scan lần {self._s4_scan_noise_retry}')
                self._enter_s4(SystemState.S4_RETRY_SCAN_HOME)
                return
            tgt = cfg.outy_output_zones[1][2]
            self.get_logger().warn(
                f"[S4 SCAN] S20 không trigger sau retry → fallback row1 ({tgt:.0f}mm)")
            self._notify_step('warn', 'STATE 4', 'scan S20 fallback',
                f'{self._sensor_label(19)} báo có khay nhưng {self._sensor_label(20)} '
                f'không trigger — fallback row1',
                check=[f'{self._sensor_label(20)} sạch + thẳng hàng',
                       f'{self._sensor_label(19)} có nhiễu không'],
                action=['Kiểm tra cảm biến S19/S20 sau khi state hoàn tất'])
            self._outy_jog_pos = tgt
            self._s4_scan_noise_retry = 0
            self._enter_s4(SystemState.S4_OUTY_DROP)
            return

        self._log_once(
            "S4_SCAN",
            f"[S4 SCAN] OutY {oy:.0f}mm arm={'OK' if self._s20_armed else 'NO'} "
            f"S20={'ON' if s20_now else 'OFF'}")

    # ══════════════════════════════════════════════════════════════
    # WARNING — CAO RỦI RO: Cyl2 RETRACT thả khay sau khi OutY tới
    # target. BẮT BUỘC verify _arrived(5) AND _at_position(5,
    # _outy_jog_pos) trước khi sang S4_CYL2_RETRACT. Sai vị trí =
    # khay rơi ngoài stack / đè khay cũ → kẹt stack.
    # ══════════════════════════════════════════════════════════════
    def _s4_outy_drop(self):
        """
        Sau khi scan xong, di chuyển OutY đến target chốt được (`_outy_jog_pos`)
        với vận tốc chậm `outy_slow_vel`. continuous_update=True cho phép cập nhật
        target động trong khi servo đang chạy.
        CAO RỦI RO: vị trí sai = Cyl2 retract thả khay ngoài stack hoặc đè khay khác.
        Verify _at_position(5, _outy_jog_pos) trước khi sang S4_CYL2_RETRACT.
        """
        if not self._cmd_sent_s4:
            target = self._outy_jog_pos or 0.0
            if target <= 0:
                self._enter_s4(SystemState.S4_CYL2_RETRACT)
                return
            ok = self._nb_move(5, target, vel=self.config.outy_slow_vel, continuous_update=True)
            if not ok:
                return
            self._cmd_sent_s4 = True
            self._step_timeout_s4 = time.time() + self.config.move_timeout

        if time.time() > self._step_timeout_s4:
            self._cmd_sent_s4 = False
        # [BLOCKING-FIX] CAO — Cyl2 retract thả khay; sai vị trí = khay rơi sai chỗ
        elif self._arrived(5) and self._at_position(5, self._outy_jog_pos or 0.0):
            self.get_logger().info(f"[S4] OutY tới target ({self._outy_jog_pos:.0f}mm) → CYL2 RETRACT")
            self._enter_s4(SystemState.S4_CYL2_RETRACT)

    # ══════════════════════════════════════════════════════════════
    # WARNING — Cyl2 retract VERIFY cross-check 2 sensor:
    #   S21 ON (retracted) AND NOT S22 (extended). Tin 1 sensor đơn
    #   = nguy cơ false-positive (vd S21 nhiễu ON dù Cyl2 chưa nhả)
    #   → OutY/OutX di chuyển khi tay vẫn kẹp khay → giật rách.
    #   Tương tự pattern S9+S10 cho Cyl1.
    # ══════════════════════════════════════════════════════════════
    def _s4_cyl2_retract_state(self):
        """
        Retract Cyl2 nhả khay vào output stack — mirror _s1_wait_release (Cyl1):
          • Retry lệnh valve mỗi 3s.
          • Cross-check 2 sensor: S21 ON AND S22 OFF (giữ nguyên — đã đúng RULE 9).
          • KHÔNG _error theo timeout — warn rồi tiếp tục retry.
        Next: S4_CYL5_EXTEND_HOLD.
        """
        cyl2_ret, cyl2_ext = self._snap(S21_CYL2_RETRACTED, S22_CYL2_EXTENDED)
        if not self._cmd_sent_s4:
            self._cyl2_retract()
            self._cyl2_retry_t  = time.time() + 3.0
            self._step_start_s4 = time.time()
            self._cmd_sent_s4   = True
        if cyl2_ret and not cyl2_ext:
            self.get_logger().info("[S4] S21 ON + S22 OFF — Cyl2 nhả xong -> Cyl5 EXTEND giữ khay")
            self._enter_s4(SystemState.S4_CYL5_EXTEND_HOLD)
            return
        if time.time() > self._cyl2_retry_t:
            self.get_logger().info("[S4] Retry Cyl2 retract")
            self._cyl2_retract()
            self._cyl2_retry_t = time.time() + 3.0
        if time.time() - self._step_start_s4 > CYLINDER_TIMEOUT_S:
            self._notify_step('warn', 'STATE 4', 'Cyl2 retract',
                f'{self._sensor_label(21)} chưa ON sau {CYLINDER_TIMEOUT_S:.0f}s '
                f'— vẫn đang retry mỗi 3s',
                check=['van Cyl2 retract', 'áp khí',
                       f'dây {self._sensor_label(21)}/{self._sensor_label(22)}'],
                action=['Kiểm tra van/khí nén', 'Hoặc nhấn STOP để abort'])
            self._step_start_s4 = time.time()
        self._log_once("S4_WAIT_S21",
                       f"Cho S21 ON + S22 OFF | S21={cyl2_ret} S22={cyl2_ext}")

    def _s4_cyl5_extend_hold(self):
        """
        Sau khi Cyl2 đã retract nhả khay xuống output stack, extend Cyl5 để giữ khay.
        Cross-check: S28 ON AND S27 OFF. Next: OutY rút về outy_target1.
        """
        cyl5_ret, cyl5_ext = self._snap(S27_CYL5_RETRACTED, S28_CYL5_EXTENDED)
        if not self._cmd_sent_s4:
            self._cyl5_extend()
            self._cyl5_retry_t  = time.time() + 3.0
            self._step_start_s4 = time.time()
            self._cmd_sent_s4   = True
        if cyl5_ext and not cyl5_ret:
            self.get_logger().info("[S4] S28 ON + S27 OFF — Cyl5 đã giữ khay -> OutY về safe")
            self._enter_s4(SystemState.S4_OUTY_OUTX_HOME)
            return
        if time.time() > self._cyl5_retry_t:
            self.get_logger().info("[S4] Retry Cyl5 extend giữ khay")
            self._cyl5_extend()
            self._cyl5_retry_t = time.time() + 3.0
        if time.time() - self._step_start_s4 > CYLINDER_TIMEOUT_S:
            self._notify_step('warn', 'STATE 4', 'Cyl5 extend hold',
                f'{self._sensor_label(28)} chưa ON sau {CYLINDER_TIMEOUT_S:.0f}s '
                f'— vẫn đang retry mỗi 3s',
                check=['van Cyl5 extend', 'áp khí',
                       f'dây {self._sensor_label(27)}/{self._sensor_label(28)}'],
                action=['Kiểm tra van/khí nén', 'Hoặc nhấn STOP để abort'])
            self._step_start_s4 = time.time()
        self._log_once("S4_WAIT_S28_HOLD",
                       f"Cho S28 ON + S27 OFF | S27={cyl5_ret} S28={cyl5_ext}")

    def _s4_outy_outx_home(self):
        """
        Sau khi Cyl5 đã giữ khay, đưa OutY về outy_target1 (raise safe) trước.
        Khi OutY đã rút khỏi khay thì sang bước retract Cyl5.
        Next: S4_CYL5_RETRACT_RELEASE.
        """
        cfg = self.config
        oy = self._pos(5)
        if oy is None:
            return

        oy_safe = oy <= cfg.outy_target1 + 5.0

        if not oy_safe:
            if not self._cmd_sent_s4:
                if not self._nb_move(5, cfg.outy_target1, vel = 120):
                    self._log_once("S4_HOME_OY_FAIL", "S4 OutY home fail")
                    return
                self._cmd_sent_s4     = True
                self._step_timeout_s4 = time.time() + self.config.move_timeout
            else:
                if time.time() > self._step_timeout_s4:
                    self.get_logger().warn("[S4] HOME OutY timeout — resend lệnh")
                    self._cmd_sent_s4 = False
                elif self._arrived(5) and self._at_position(5, cfg.outy_target1):
                    self.get_logger().info("[S4] OutY đã về safe -> Cyl5 RETRACT nhả cơ cấu giữ")
                    self._enter_s4(SystemState.S4_CYL5_RETRACT_RELEASE)
            return

        self.get_logger().info("[S4] OutY đang safe -> Cyl5 RETRACT nhả cơ cấu giữ")
        self._enter_s4(SystemState.S4_CYL5_RETRACT_RELEASE)

    def _s4_cyl5_retract_release(self):
        """
        Sau khi OutY đã rút về safe, retract Cyl5 để nhả cơ cấu giữ khay.
        Cross-check: S27 ON AND S28 OFF. Next: OutX về outx_target1.
        """
        cyl5_ret, cyl5_ext = self._snap(S27_CYL5_RETRACTED, S28_CYL5_EXTENDED)
        if not self._cmd_sent_s4:
            self._cyl5_retract()
            self._cyl5_retry_t  = time.time() + 3.0
            self._step_start_s4 = time.time()
            self._cmd_sent_s4   = True
        if cyl5_ret and not cyl5_ext:
            self.get_logger().info("[S4] S27 ON + S28 OFF — Cyl5 đã retract -> OutX về target1")
            self._enter_s4(SystemState.S4_OUTX_HOME)
            return
        if time.time() > self._cyl5_retry_t:
            self.get_logger().info("[S4] Retry Cyl5 retract")
            self._cyl5_retract()
            self._cyl5_retry_t = time.time() + 3.0
        if time.time() - self._step_start_s4 > CYLINDER_TIMEOUT_S:
            self._notify_step('warn', 'STATE 4', 'Cyl5 retract release',
                f'{self._sensor_label(27)} chưa ON sau {CYLINDER_TIMEOUT_S:.0f}s '
                f'— vẫn đang retry mỗi 3s',
                check=['van Cyl5 retract', 'áp khí',
                       f'dây {self._sensor_label(27)}/{self._sensor_label(28)}'],
                action=['Kiểm tra van/khí nén', 'Hoặc nhấn STOP để abort'])
            self._step_start_s4 = time.time()
        self._log_once("S4_WAIT_S27_RELEASE",
                       f"Cho S27 ON + S28 OFF | S27={cyl5_ret} S28={cyl5_ext}")

    def _s4_outx_home(self):
        """
        Bước cuối STATE 4: OutX về outx_target1 sau khi OutY safe và Cyl5 đã retract.
        Next: S4_COMPLETE.
        """
        cfg = self.config
        ox = self._pos(4)
        if ox is None:
            return

        ox_at_target = abs(ox - cfg.outx_target1) <= 5.0
        if not ox_at_target:
            if not self._cmd_sent_s4:
                if not self._nb_move(4, cfg.outx_target1, vel = 150):
                    self._log_once("S4_HOME_OX_FAIL", "S4 OutX home fail")
                    return
                self._cmd_sent_s4     = True
                self._step_timeout_s4 = time.time() + self.config.move_timeout
            else:
                if time.time() > self._step_timeout_s4:
                    self.get_logger().warn("[S4] HOME OutX timeout — resend lệnh")
                    self._cmd_sent_s4 = False
                elif self._arrived(4) and self._at_position(4, cfg.outx_target1):
                    self._enter_s4(SystemState.S4_COMPLETE)
            return

        # OutX đã ở target1.
        self._enter_s4(SystemState.S4_COMPLETE)

    def _s4_retry_scan_home(self):
        """
        Retry sau khi scan S20 thất bại lần 1 (mirror _s1_retry_scan_home):
        đưa OutY về outy_target1 (safe) rồi quay lại S4_OUTY_SCAN_S20.
        Reset _s20_armed/_s20_prev trước khi scan lại.
        Timeout → resend (không ERROR).
        """
        cfg = self.config
        if not self._cmd_sent_s4:
            ok = self._nb_move(5, cfg.outy_target1)
            if not ok:
                self._log_once("S4_RETRY_MOVE_FAIL",
                               "[S4] retry scan: OutY về safe fail — chờ thử lại")
                return
            self._cmd_sent_s4     = True
            self._step_timeout_s4 = time.time() + cfg.move_timeout
            return

        if time.time() > self._step_timeout_s4:
            self.get_logger().warn("[S4] retry scan: OutY về safe timeout — resend")
            self._cmd_sent_s4 = False
            return

        # [BLOCKING-FIX] verify về safe thật trước khi scan lại
        if self._arrived(5) and self._at_position(5, cfg.outy_target1):
            self._cmd_sent_s4 = False
            self._step_timeout_s4 = 0.0
            self._s20_armed = False
            self._s20_prev  = self.sensor(S20_SCAN_STACK_P2)
            self.get_logger().info("[S4 SCAN] Retry lại từ OutY safe")
            self._enter_s4(SystemState.S4_OUTY_SCAN_S20)

    def _s4_complete(self):
        self._pub_cartridge_busy(False)
        self._pub_cartridge_pos2_busy(False)
        self._notify('info', 'State 4 done', 'Thay khay output thanh cong')
        self.get_logger().info("[S4] COMPLETE")
        self._s4_trigger = False
        # Trước đây bug: _enter_s4(S3_CHECK_OUTXY_SAFE) — set state_s4 thành
        # giá trị S3_*, nhưng _dispatch_s4 không có branch S3_* → state_s4 kẹt
        # vô hạn, không tick + state_s3 vẫn IDLE → S3 không chạy. Fix: trả
        # state_s4 về IDLE, kích S3 qua dispatcher đúng.
        self._enter_s4(SystemState.IDLE)
        if (not self.sensor(S18_FEED_OK)
                and not self.sensor(S17_PLATFORM)
                and self._servo3_at_last_tray_position()):
            self.get_logger().info("[S4→S3] S18 OFF + S17 OFF sau khi lấy khay cuối -> Servo3 về home check S17")
            self._s3_output_loaded = False
            self._s3_finish_after_home_check = True
            self._s3_force_home = True
            self._enter_s3(SystemState.S3_CHECK_OUTXY_SAFE)
        elif self._can_start_s3():
            self.get_logger().info("[S4→S3] S17 ON + S18 OFF → cấp khay output mới")
            self._s3_output_loaded = False
            self._s3_finish_after_home_check = False
            self._enter_s3(SystemState.S3_CHECK_OUTXY_SAFE)


# ─── Main ─────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    system = None

    def _sig(signum, frame):
        if system:
            try:
                system.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        os._exit(0)

    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    try:
        config = None
        for candidate in [
            os.path.join(os.path.dirname(__file__), '..', 'config', 'cartridge_config.yaml'),
            os.path.expanduser('~/ros2_ws/src/system_feed_cartridge/config/cartridge_config.yaml'),
        ]:
            if os.path.exists(candidate):
                config = CartridgeConfig.load(candidate)
                break
        
        if config is None:
            config = CartridgeConfig()

        system = CartridgeSystem(config)
        print("=" * 60)
        print("  Cartridge System v8")
        print("  MANUAL + START → Homing → JOG mode")
        print("  MANUAL: Nhấn STATE 1/3 → check sensor thật → enter state")
        print("  MANUAL: Nhấn STATE 2/4 → pub robot done_tray topic → enter state")
        print("  AUTO/AI: Hệ thống tự trigger STATE 1/2/3/4 khi đủ điều kiện sensor")
        print("=" * 60)

        rclpy.spin(system)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if system:
            try:
                system.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        # Only use os._exit as a last resort if things hang
        # os._exit(0)


if __name__ == '__main__':
    main()
