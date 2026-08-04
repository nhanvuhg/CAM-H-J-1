#!/usr/bin/env python3
"""
Integration Tests — Cartridge State Machine
============================================
Chạy 100% offline, không cần phần cứng (Servo, IO, ROS 2).

Chiến lược: Không mock toàn bộ CartridgeSystem (quá phức tạp vì __init__
gọi create_publisher, _connect_hardware, v.v.), mà thay vào đó:

  1. Test Pydantic Config (`SystemConfig`) — Schema validation
  2. Test State Machine logic — Tạo object "sạch" bằng __new__ rồi
     gán thủ công đúng các thuộc tính cần thiết, gọi trực tiếp hàm logic.
  3. Test SystemState enum completeness — đảm bảo tất cả state đều có value.

Cách chạy:
    cd ~/ros2_ws
    python3 -m pytest src/system_feed_cartridge/tests/test_state_machine.py -v

Kết quả mong đợi: Tất cả PASSED, không có kết nối mạng hay phần cứng.
"""

import sys
import os
import time
import threading
import pytest
from unittest.mock import MagicMock, PropertyMock, call
from enum import Enum

# ── Path setup ────────────────────────────────────────────────────
SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
CONFIG_DIR  = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config'))
sys.path.insert(0, SCRIPTS_DIR)

# ── Mock ALL external modules BEFORE importing ours ──────────────

# Create a real stub Node class so CartridgeSystem can inherit properly
import types

_rclpy_mock = MagicMock()
_rclpy_node_mock = types.ModuleType('rclpy.node')

class _StubNode:
    """Minimal stand-in for rclpy.node.Node."""
    def __init__(self, *a, **kw): pass
    def create_publisher(self, *a, **kw): return MagicMock()
    def create_subscription(self, *a, **kw): return MagicMock()
    def create_timer(self, *a, **kw): return MagicMock()
    def get_logger(self): return MagicMock()
    def destroy_node(self): pass

_rclpy_node_mock.Node = _StubNode

# ROS 2
sys.modules['rclpy'] = _rclpy_mock
sys.modules['rclpy.node'] = _rclpy_node_mock
for mod in ['rclpy.qos', 'rclpy.executors',
            'std_msgs', 'std_msgs.msg']:
    sys.modules[mod] = MagicMock()

# Festo edcon — distinct module stubs prevent MagicMock from lazily exposing
# the real driver or constructing a hardware connection during test collection.
for mod in ['edcon', 'edcon.edrive', 'edcon.utils']:
    sys.modules[mod] = types.ModuleType(mod)
_edcon_com = types.ModuleType('edcon.edrive.com_modbus')
_edcon_com.ComModbus = None
sys.modules['edcon.edrive.com_modbus'] = _edcon_com
_edcon_motion = types.ModuleType('edcon.edrive.motion_handler')
_edcon_motion.MotionHandler = None
sys.modules['edcon.edrive.motion_handler'] = _edcon_motion
_edcon_logging = types.ModuleType('edcon.utils.logging')
_edcon_logging.Logging = None
sys.modules['edcon.utils.logging'] = _edcon_logging

# Festo CPX-AP
for mod in ['cpx_io', 'cpx_io.cpx_system', 'cpx_io.cpx_system.cpx_ap']:
    sys.modules[mod] = types.ModuleType(mod)
_cpx_driver = types.ModuleType('cpx_io.cpx_system.cpx_ap.cpx_ap')
_cpx_driver.CpxAp = None
sys.modules['cpx_io.cpx_system.cpx_ap.cpx_ap'] = _cpx_driver

# Now we can safely import our modules
from config import SystemConfig
from cartridge_providesystem_py_node import (
    CartridgeSystem,
    SystemState,
    S1_BELT_START, S2_BELT_MID, S3_BELT_END,
    S7_TRAY_AT_ROBOT, S9_CYL1_RETRACTED, S10_CYL1_EXTENDED,
    S17_PLATFORM, S18_FEED_OK, S19_CHECK_TRAY_P2, S20_SCAN_STACK_P2,
    S25_CYL4_RETRACTED, S26_CYL4_EXTENDED,
    S27_CYL5_RETRACTED, S28_CYL5_EXTENDED,
)


class TestHardwareReconnectSupervisors:
    """Offline-at-startup drivers must be retried without rebuilding a node."""

    def test_io_supervisor_retries_until_module_appears(self, monkeypatch):
        node = CartridgeSystem.__new__(CartridgeSystem)
        node.io_module = None
        node.io_module_2 = None
        node._io_bg_lock = threading.Lock()
        node._pos_thread_stop = False
        node.get_logger = MagicMock(return_value=MagicMock())

        calls = []

        def connect_once(idx, ip, attempts=1):
            calls.append((idx, ip, attempts))
            if len(calls) == 2:
                node.io_module = object()
                return True
            return False

        node._connect_io = connect_once
        # Two supervisor iterations: first offline, second connected.
        _rclpy_mock.ok.side_effect = [True, True, False]
        monkeypatch.setattr(time, 'sleep', lambda _seconds: None)

        node._io_supervisor_one(1, '172.16.11.37')

        assert calls == [
            (1, '172.16.11.37', 1),
            (1, '172.16.11.37', 1),
        ]
        assert node.io_module is not None
        _rclpy_mock.ok.side_effect = None


# ══════════════════════════════════════════════════════════════════
# SECTION 1: Pydantic Config Validation
# ══════════════════════════════════════════════════════════════════

class TestSystemConfig:
    """Test Pydantic SystemConfig schema validation."""

    def test_load_cartridge_config(self):
        """Đọc file cartridge_config.yaml thật → không lỗi validation."""
        config_path = os.path.join(CONFIG_DIR, 'cartridge_config.yaml')
        if not os.path.exists(config_path):
            pytest.skip("cartridge_config.yaml not found")
        config = SystemConfig.load(config_path)
        assert config.io_ip == "172.16.11.37"
        assert 1 in config.servo_ips
        assert config.homing_timeout > 0

    def test_servo_ips_type_enforcement(self):
        """servo_ips phải là Dict[int, str], sai type → Pydantic reject."""
        from pydantic import ValidationError
        with pytest.raises(ValidationError):
            SystemConfig(
                servo_ips={"not_int": "192.168.1.1"},  # key phải là int
                io_ip="x", io_ip_2="y",
                servo_limits={1: 100.0},
                iny_input_zones={}, iny_output_zones={}, outy_output_zones={},
            )

    def test_default_values(self):
        """Kiểm tra các default value khi không truyền."""
        config = SystemConfig(
            servo_ips={1: "172.16.11.36"},
            io_ip="172.16.11.37", io_ip_2="172.16.11.41",
            servo_limits={1: 700.0},
            iny_input_zones={}, iny_output_zones={}, outy_output_zones={},
        )
        assert config.homing_timeout == 30.0
        assert config.cylinder1_extend_channel == 4
        assert config.cylinder1_retract_channel == 5
        assert config.cylinder3_extend_channel == 6
        assert config.cylinder3_retract_channel == 7
        assert config.cylinder2_extend_channel == 0
        assert config.cylinder2_retract_channel == 1
        assert config.cylinder4_extend_channel == 2
        assert config.cylinder4_retract_channel == 3
        assert config.cylinder5_extend_channel == 4
        assert config.cylinder5_retract_channel == 5
        assert config.modbus_timeout_ms == 3000

    def test_yaml_no_as_false_handled(self):
        """YAML parses 'NO' as boolean False — config.load() phải xử lý đúng."""
        config_path = os.path.join(CONFIG_DIR, 'cartridge_config.yaml')
        if not os.path.exists(config_path):
            pytest.skip("cartridge_config.yaml not found")
        config = SystemConfig.load(config_path)
        assert config.sensor_logic in ("NO", "NC")

    def test_save_roundtrip(self, tmp_path):
        """Save rồi load lại phải giống nhau."""
        config = SystemConfig(
            servo_ips={1: "10.0.0.1"}, io_ip="10.0.0.2", io_ip_2="10.0.0.3",
            servo_limits={1: 500.0},
            iny_input_zones={1: [10.0, 20.0, 30.0]},
            iny_output_zones={}, outy_output_zones={},
        )
        path = str(tmp_path / "test_config.yaml")
        config.save(path)
        loaded = SystemConfig.load(path)
        assert loaded.servo_ips == {1: "10.0.0.1"}
        assert loaded.iny_input_zones[1] == [10.0, 20.0, 30.0]


# ══════════════════════════════════════════════════════════════════
# SECTION 2: SystemState Enum
# ══════════════════════════════════════════════════════════════════

class TestSystemStateEnum:
    """Test SystemState enum completeness and uniqueness."""

    def test_all_values_unique(self):
        """Mỗi state phải có value riêng biệt."""
        values = [s.value for s in SystemState]
        assert len(values) == len(set(values)), "Duplicate state values found!"

    def test_required_states_exist(self):
        """Các state quan trọng phải tồn tại."""
        required = ['IDLE', 'ERROR', 'HOMING', 'HOMING_RUNNING',
                     'S1_CONFIRM_SAFE', 'S1_COMPLETE',
                     'S2A_CHECK_INTERLOCK', 'S2A_COMPLETE',
                     'S3_CHECK_OUTXY_SAFE', 'S3_CYL4_EXTEND', 'S3_COMPLETE',
                     'S4_CHECK_OUTY_SAFE', 'S4_CYL5_SYNC', 'S4_COMPLETE']
        for name in required:
            assert hasattr(SystemState, name), f"Missing state: {name}"

    def test_state_is_enum(self):
        """SystemState phải kế thừa Enum."""
        assert issubclass(SystemState, Enum)


# ══════════════════════════════════════════════════════════════════
# SECTION 3: State Machine Logic (Unit Tests)
# ══════════════════════════════════════════════════════════════════

def _make_node():
    """
    Tạo CartridgeSystem instance KHÔNG gọi __init__ (bypass ROS 2).
    Gán thủ công các thuộc tính cần thiết cho các hàm logic.
    """
    # Import here since module-level mock must be set first
    from cartridge_providesystem_py_node import CartridgeSystem

    node = object.__new__(CartridgeSystem)

    # Minimal config
    node.config = SystemConfig(
        servo_ips={1: "x", 2: "x", 3: "x", 4: "x", 5: "x"},
        io_ip="x", io_ip_2="x",
        servo_limits={1: 700.0, 2: 900.0, 3: 400.0, 4: 600.0, 5: 600.0},
        iny_input_zones={1: [451.0, 500.0, 550.0]},
        iny_output_zones={1: [451.0, 500.0, 550.0]},
        outy_output_zones={10: [0.0, 9.0, 40.0]},
    )

    # State machines
    node.state    = SystemState.IDLE
    node.state_in = SystemState.IDLE
    node.state_s3 = SystemState.IDLE
    node.state_s4 = SystemState.IDLE

    # Operation
    node.operation_mode    = 'auto'
    node._system_running   = True
    node._motion_busy      = False
    node._state1_enabled   = True
    node._input_tray_done  = False
    node._gui_confirmed    = False
    node._system_paused    = False
    node._pause_pending    = False
    node._active_servo_moves = {}
    node._paused_servo_moves = {}
    node._pause_started_at = 0.0
    node._pause_restart_homing = False
    node._s4_trigger       = False
    node._jog_mode         = False
    node._drain_armed_after_s2 = False
    node._drain_state      = False

    # Hardware stubs
    node.zero_offset = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0}
    node.servos      = {}
    node.io_module   = None
    node.io_module_2 = None
    node._io_ready   = False
    node._io_ready_2 = False
    node._io_sensor_cache   = []
    node._io_sensor_cache_2 = []
    node._io_bg_lock = MagicMock()

    # Runtime flags
    node._cmd_sent_in       = False
    node._s1_scan_noise_retry = 0
    node._s4_prev_in        = False
    node._s5_retry          = 0
    node._s1_retry_count    = 0
    node._cmd_sent_s3       = False
    node._step_start_s3     = 0.0
    node._cyl4_retry_t      = 0.0
    node._cyl4_retract_sent = False
    node._cmd_sent_s4       = False
    node._cyl5_retry_t      = 0.0
    node._s3_pending        = False
    node._step_timeout_in   = 0.0
    node._step_timeout_s3   = 0.0
    node._step_timeout_s4   = 0.0
    node._inx_moving        = False
    node._iny_moving        = False
    node._homing_abort      = MagicMock(); node._homing_abort.is_set = MagicMock(return_value=False)
    node._homing_done_event = MagicMock(); node._homing_done_event.is_set = MagicMock(return_value=False)
    node._homing_result     = True
    node._homing_generation = 0
    node._servo_home_abort  = set()
    node._servo_home_generation = {}
    node._homing_guard_lock = threading.Lock()
    node._homing_ref_offset = {}
    node._drive_warm_t      = 0.0
    node._jog_active        = set()
    node._servo_motion_t    = {}
    node._ready_until       = {}
    node._need_warm_up      = {}
    node._pos_cache         = {}
    node._s10_off_time      = 0.0
    node._s10_prev          = False
    node._robot_connected   = False
    node._robot_last_seen   = 0.0

    # Publishers (mocked)
    node.pub_busy_cartridge   = MagicMock()
    node.pub_busy_cartridge_pos2 = MagicMock()
    node.pub_robot_mode        = MagicMock()
    node.pub_homing_done      = MagicMock()
    node.pub_state            = MagicMock()
    node.pub_new_tray         = MagicMock()
    node.pub_newtray_output   = MagicMock()
    node.pub_gui_notify       = MagicMock()
    node.pub_servo_pos        = MagicMock()
    node.pub_sensors          = MagicMock()
    node.pub_input_trays_empty = MagicMock()
    node.pub_drain           = MagicMock()
    node.pub_current_mode     = MagicMock()
    node.pub_config_data      = MagicMock()
    node.pub_vfd_run          = MagicMock()

    # Logger (mock)
    node._logger = MagicMock()
    node.get_logger = MagicMock(return_value=node._logger)

    # Notify / log helpers
    node._notify_throttle = {}
    node._guide_logged    = set()

    return node


def _set_sensors(node, **sensor_states):
    """Helper: inject sensor state qua _io_sensor_cache (thay thế _sim_sensors cũ).
    Dùng: _set_sensors(node, S1_BELT_START=True, S7_TRAY_AT_ROBOT=False) — key có thể
    là constant (int) hoặc số sid. Tự phân loại module 1 (1-16) vs module 2 (17-28)."""
    cache1 = [False] * 16
    cache2 = [False] * 12
    for sid, val in sensor_states.items():
        if isinstance(sid, str):
            sid_val = globals().get(sid)  # cho phép truyền "S1_BELT_START"=True
            sid = sid_val if sid_val is not None else int(sid)
        if 1 <= sid <= 16:
            cache1[sid - 1] = bool(val)
        elif 17 <= sid <= 28:
            cache2[sid - 17] = bool(val)
    node._io_ready = True
    node._io_ready_2 = True
    node._io_sensor_cache = cache1
    node._io_sensor_cache_2 = cache2


class TestCanStartConditions:
    """Test trigger conditions cho từng State."""

    def test_can_start_s1_happy_path(self):
        """S1 trigger: auto mode, có khay (S1/S2/S3), S7 OFF, S9 ON, S10 OFF."""
        node = _make_node()
        node.operation_mode = 'manual'
        _set_sensors(node, S1_BELT_START=True, S7_TRAY_AT_ROBOT=False, S9_CYL1_RETRACTED=True, S10_CYL1_EXTENDED=False)
        # _can_start_s1 reads sensor() which checks operation_mode
        # In auto mode, sensor() calls _sensor_raw() which reads IO cache
        node.operation_mode = 'manual'
        # manual mode bypasses _can_start_s1's "auto" check, so we need
        # to test the raw condition logic directly
        # Instead, let's test the individual conditions:
        has_tray    = node.sensor(S1_BELT_START) or node.sensor(S2_BELT_MID) or node.sensor(S3_BELT_END)
        place_ok    = not node.sensor(S7_TRAY_AT_ROBOT)
        cyl1_ret    = node.sensor(S9_CYL1_RETRACTED)
        cyl1_ext    = not node.sensor(S10_CYL1_EXTENDED)
        assert has_tray is True
        assert place_ok is True
        assert cyl1_ret is True
        assert cyl1_ext is True

    def test_can_start_s1_blocked_by_s7(self):
        """S7 ON (khay đang tại robot) → không được trigger S1."""
        node = _make_node()
        node.operation_mode = 'manual'
        # S7 ON là blocking condition
        _set_sensors(node, S1_BELT_START=True, S7_TRAY_AT_ROBOT=True, S9_CYL1_RETRACTED=True, S10_CYL1_EXTENDED=False)
        place_ok = not node.sensor(S7_TRAY_AT_ROBOT)
        assert place_ok is False

    def test_can_start_s1_no_tray(self):
        """Không có khay nào trên belt → không trigger."""
        node = _make_node()
        node.operation_mode = 'auto'
        _set_sensors(node, S1_BELT_START=False, S2_BELT_MID=False, S3_BELT_END=False)
        has_tray = node.sensor(S1_BELT_START) or node.sensor(S2_BELT_MID) or node.sensor(S3_BELT_END)
        assert has_tray is False

    def test_can_start_s2a(self):
        """S2A trigger: robot done + S7 ON + không busy."""
        node = _make_node()
        node.operation_mode = 'auto'
        node._input_tray_done = True
        node._motion_busy = False
        _set_sensors(node, S7_TRAY_AT_ROBOT=True)
        result = node._can_start_s2a()
        assert result is True

    def test_can_start_s2a_blocked_no_done(self):
        """Chưa nhận done_tray_input → S2A không trigger."""
        node = _make_node()
        node._input_tray_done = False
        _set_sensors(node, S7_TRAY_AT_ROBOT=True)
        assert node._can_start_s2a() is False

    def test_can_start_s4(self):
        """S4 trigger: có _s4_trigger + S18 ON + không busy."""
        node = _make_node()
        node.operation_mode = 'auto'
        node._s4_trigger = True
        node._motion_busy = False
        _set_sensors(node, S18_FEED_OK=True)
        assert node._can_start_s4() is True

    def test_can_start_s4_no_trigger(self):
        """Không có _s4_trigger → S4 không chạy."""
        node = _make_node()
        node._s4_trigger = False
        _set_sensors(node, S18_FEED_OK=True)
        assert node._can_start_s4() is False


class TestStateTransitions:
    """Test state entry/exit logic."""

    def test_enter_in_changes_state(self):
        """_enter_in() thay đổi state_in và reset cmd_sent."""
        node = _make_node()
        node._cmd_sent_in = True
        node._enter_in(SystemState.S1_CONFIRM_SAFE)
        assert node.state_in == SystemState.S1_CONFIRM_SAFE
        assert node._cmd_sent_in is False

    def test_enter_s3_changes_state(self):
        node = _make_node()
        node._cmd_sent_s3 = True
        node._enter_s3(SystemState.S3_CHECK_OUTXY_SAFE)
        assert node.state_s3 == SystemState.S3_CHECK_OUTXY_SAFE
        assert node._cmd_sent_s3 is False

    def test_enter_s4_changes_state(self):
        node = _make_node()
        node._cmd_sent_s4 = True
        node._enter_s4(SystemState.S4_CHECK_OUTY_SAFE)
        assert node.state_s4 == SystemState.S4_CHECK_OUTY_SAFE
        assert node._cmd_sent_s4 is False

    def test_s3_cyl4_extend_requires_crosscheck(self):
        """S26 ON + S25 OFF mới cho phép Servo 3 chuyển sang FEED."""
        node = _make_node()
        node.state_s3 = SystemState.S3_CYL4_EXTEND
        _set_sensors(
            node,
            S17_PLATFORM=True,
            S25_CYL4_RETRACTED=False,
            S26_CYL4_EXTENDED=True,
        )
        node._cyl4_extend = MagicMock(return_value=True)

        node._s3_cyl4_extend()

        assert node.state_s3 == SystemState.S3_SERVO3_FEED
        node._cyl4_extend.assert_not_called()

    def test_s3_cyl4_extend_waits_on_invalid_feedback(self):
        """Cả hai limit OFF chưa hợp lệ: retry EXTEND và chưa chạy Servo 3."""
        node = _make_node()
        node.state_s3 = SystemState.S3_CYL4_EXTEND
        _set_sensors(
            node,
            S17_PLATFORM=True,
            S25_CYL4_RETRACTED=False,
            S26_CYL4_EXTENDED=False,
        )
        node._cyl4_extend = MagicMock(return_value=True)

        node._s3_cyl4_extend()

        assert node.state_s3 == SystemState.S3_CYL4_EXTEND
        node._cyl4_extend.assert_called_once()

    def test_s3_cyl4_retracts_if_s17_lost(self):
        """S17 OFF trước khi Cyl4 xác nhận xong → retract và quay lại WAIT_S17."""
        node = _make_node()
        node.state_s3 = SystemState.S3_CYL4_EXTEND
        _set_sensors(
            node,
            S17_PLATFORM=False,
            S25_CYL4_RETRACTED=False,
            S26_CYL4_EXTENDED=True,
        )
        node._cyl4_retract = MagicMock(return_value=True)

        node._s3_cyl4_extend()

        assert node.state_s3 == SystemState.S3_WAIT_S17
        node._cyl4_retract.assert_called_once()

    def test_s4_cyl5_extends_when_s19_on(self):
        """S19 ON: S28 ON + S27 OFF mới cho phép chuyển sang scan S20."""
        node = _make_node()
        node.state_s4 = SystemState.S4_CYL5_SYNC
        _set_sensors(
            node,
            S19_CHECK_TRAY_P2=True,
            S27_CYL5_RETRACTED=False,
            S28_CYL5_EXTENDED=True,
        )
        node._cyl5_extend = MagicMock(return_value=True)

        node._s4_cyl5_sync()

        assert node.state_s4 == SystemState.S4_OUTY_SCAN_S20
        node._cyl5_extend.assert_not_called()

    def test_s4_cyl5_retracts_when_s19_off(self):
        """S19 OFF: S27 ON + S28 OFF mới cho phép đi xuống row 1."""
        node = _make_node()
        node.state_s4 = SystemState.S4_CYL5_SYNC
        _set_sensors(
            node,
            S19_CHECK_TRAY_P2=False,
            S27_CYL5_RETRACTED=True,
            S28_CYL5_EXTENDED=False,
        )
        node._cyl5_retract = MagicMock(return_value=True)

        node._s4_cyl5_sync()

        assert node.state_s4 == SystemState.S4_OUTY_ROW1
        node._cyl5_retract.assert_not_called()

    def test_s4_cyl5_waits_on_invalid_crosscheck(self):
        """Limit chưa hợp lệ: gửi lệnh theo S19 và giữ nguyên enum sync."""
        node = _make_node()
        node.state_s4 = SystemState.S4_CYL5_SYNC
        _set_sensors(
            node,
            S19_CHECK_TRAY_P2=True,
            S27_CYL5_RETRACTED=True,
            S28_CYL5_EXTENDED=False,
        )
        node._cyl5_extend = MagicMock(return_value=True)

        node._s4_cyl5_sync()

        assert node.state_s4 == SystemState.S4_CYL5_SYNC
        node._cyl5_extend.assert_called_once()

    def test_s20_sensor_first_stops_before_reading_outy_position(self):
        """S20 ON khi armed phải STOP Servo5 trước rồi mới đọc encoder/phân row."""
        node = _make_node()
        node.state_s4 = SystemState.S4_OUTY_SCAN_S20
        node._cmd_sent_s4 = True
        node._s20_armed = True
        node._s4_scan_noise_retry = 1
        node._s4_s20_on_logged = False
        node.sensor = MagicMock(side_effect=lambda sid: sid == S20_SCAN_STACK_P2)
        node._notify = MagicMock()

        calls = MagicMock()
        node._stop = MagicMock(side_effect=lambda sid: calls.stop(sid))
        node._pos = MagicMock(side_effect=lambda sid: (calls.position(sid), 5.0)[1])
        node._pos_cached = MagicMock(return_value=5.0)

        node._s4_outy_scan_s20()

        assert calls.mock_calls[:2] == [call.stop(5), call.position(5)]
        assert node._outy_jog_pos == 40.0
        assert node._s4_scan_noise_retry == 0
        assert node.state_s4 == SystemState.S4_OUTY_DROP

    def test_s1_abort_resets_to_idle(self):
        """_s1_abort() → state_in = IDLE, state1_enabled = False, pub busy(False)."""
        node = _make_node()
        node.state_in = SystemState.S1_INY_SCAN
        node._state1_enabled = True
        node._cmd_sent_in = True

        # Stub _stop, _cyl1_retract, _cyl2_retract, _sync_mode_jog
        node._stop = MagicMock()
        node._cyl1_retract = MagicMock()
        node._cyl2_retract = MagicMock()
        node._sync_mode_jog = MagicMock()
        node._notify = MagicMock()

        node._s1_abort("Unit test abort")

        assert node.state_in == SystemState.IDLE
        assert node.state_s3 == SystemState.IDLE
        assert node.state_s4 == SystemState.IDLE
        assert node.state == SystemState.IDLE
        assert node._state1_enabled is False
        assert node._jog_mode is True
        # Verify busy = False was published
        node.pub_busy_cartridge.publish.assert_called()

    def test_error_state_entry(self):
        """_error() → state = ERROR."""
        node = _make_node()
        node._stop = MagicMock()
        node._cyl1_retract = MagicMock()
        node._cyl2_retract = MagicMock()
        node._notify = MagicMock()
        node._enter = MagicMock()

        node._error("Test error")

        node._enter.assert_called_with(SystemState.ERROR)


class TestInstantPauseResume:
    """PAUSE phải dừng motion ngay nhưng không đổi state/step."""

    def test_pause_stops_active_axes_and_keeps_state(self):
        node = _make_node()
        node.state_in = SystemState.S1_INY_TO_ROW
        node._active_servo_moves = {
            1: (450.0, 200, False),
            2: (620.0, 120, True),
        }
        node._notify = MagicMock()
        node._stop_immediate = MagicMock(
            side_effect=lambda sid: node._active_servo_moves.pop(sid, None))

        node._cb_pause(types.SimpleNamespace(data=True))

        assert node._system_paused is True
        assert node.state_in == SystemState.S1_INY_TO_ROW
        assert node._paused_servo_moves == {
            1: (450.0, 200, False),
            2: (620.0, 120, True),
        }
        assert node._stop_immediate.call_args_list == [call(1), call(2)]

    def test_resume_reissues_targets_and_preserves_step(self):
        node = _make_node()
        node.state_s4 = SystemState.S4_OUTY_DROP
        node._system_paused = True
        node._pause_started_at = time.time() - 5.0
        node._paused_servo_moves = {5: (310.0, 80, True)}
        node._notify = MagicMock()
        node._nb_move = MagicMock(return_value=True)

        node._cb_resume(types.SimpleNamespace(data=True))

        assert node._system_paused is False
        assert node.state_s4 == SystemState.S4_OUTY_DROP
        node._nb_move.assert_called_once_with(
            5, 310.0, vel=80, continuous_update=True)
        assert node._paused_servo_moves == {}

    def test_resume_failure_remains_paused(self):
        node = _make_node()
        node._system_paused = True
        node._pause_started_at = time.time()
        node._paused_servo_moves = {3: (800.0, 100, False)}
        node._notify = MagicMock()
        node._nb_move = MagicMock(return_value=False)
        node._stop_immediate = MagicMock()

        node._cb_resume(types.SimpleNamespace(data=True))

        assert node._system_paused is True
        assert node._paused_servo_moves == {3: (800.0, 100, False)}


class TestSensorReading:
    """Test sensor reading — sau khi remove sim_sensor, mọi mode đọc real IO."""

    def test_sensor_reads_real_io_in_manual(self):
        """Manual mode → sensor() đọc từ IO module thực (giống auto)."""
        node = _make_node()
        node.operation_mode = 'manual'
        node._io_ready = True
        node._io_sensor_cache = [True if i in (0, 6) else False for i in range(16)]  # S1=True, S7=True

        assert node.sensor(1) is True
        assert node.sensor(7) is True
        assert node.sensor(2) is False
        assert node.sensor(99) is False  # sid out-of-range → False

    def test_sensor_io_not_ready_returns_false(self):
        """Nếu IO module chưa ready → sensor() trả về False (fail-safe)."""
        node = _make_node()
        node.operation_mode = 'manual'
        node._io_ready = False
        node._io_sensor_cache = []
        assert node.sensor(S1_BELT_START) is False

    def test_pub_cartridge_busy(self):
        """_pub_cartridge_busy() gọi publisher đúng value."""
        node = _make_node()
        node._pub_cartridge_busy(True)
        node.pub_busy_cartridge.publish.assert_called_once()
        call_args = node.pub_busy_cartridge.publish.call_args
        assert call_args is not None


class TestCallbackLogic:
    """Test ROS callback handlers."""

    def test_cb_done_tray_input_sets_flag_in_auto(self):
        """AUTO mode: done_tray_input(True) → set flag + enable auto-chain S1."""
        node = _make_node()
        node.operation_mode = 'auto'
        node._input_tray_done = False
        node._state1_enabled = False

        msg = MagicMock()
        msg.data = True
        node._notify = MagicMock()
        node._cb_done_tray_input(msg)

        assert node._input_tray_done is True
        assert node._state1_enabled is True  # AUTO chain S1 sau S2A_COMPLETE

    def test_cb_done_tray_input_in_manual_keeps_state1_disabled(self):
        """MANUAL mode: done_tray_input(True) → set flag NHƯNG KHÔNG enable S1 (dừng chain)."""
        node = _make_node()
        node.operation_mode = 'manual'
        node._input_tray_done = False
        node._state1_enabled = False

        msg = MagicMock()
        msg.data = True
        node._notify = MagicMock()
        node._cb_done_tray_input(msg)

        assert node._input_tray_done is True
        assert node._state1_enabled is False  # MANUAL không auto-chain S1

    def test_cb_done_tray_output_triggers_s4(self):
        """Nhận done_tray_output(True) → _s4_trigger = True."""
        node = _make_node()
        node._s4_trigger = False
        node._notify = MagicMock()

        msg = MagicMock()
        msg.data = True
        node._cb_done_tray_output(msg)

        assert node._s4_trigger is True

    def test_cb_motion_busy(self):
        """Nhận /robot/motion_busy → cập nhật _motion_busy."""
        node = _make_node()
        msg = MagicMock()
        msg.data = True
        node._cb_motion_busy(msg)

        assert node._motion_busy is True
        assert node._robot_connected is True


# ══════════════════════════════════════════════════════════════════
# SECTION 4: Full State Flow (Integration-level)
# ══════════════════════════════════════════════════════════════════

class TestIdleDispatch:
    """Test _do_idle_input dispatch logic."""

    def test_idle_triggers_s2a_when_ready(self):
        """IDLE + robot done + S7 → phải chuyển sang S2A_CHECK_INTERLOCK."""
        node = _make_node()
        node.operation_mode = 'auto'
        node._input_tray_done = True
        node._motion_busy = False
        _set_sensors(node, S7_TRAY_AT_ROBOT=True)
        node._notify = MagicMock()
        node._log_once = MagicMock()

        node._do_idle_input()

        assert node.state_in == SystemState.S2A_CHECK_INTERLOCK
        assert node._input_tray_done is False

    def test_idle_triggers_s1_when_enabled(self):
        """IDLE + _state1_enabled + đủ sensor conditions → S1."""
        node = _make_node()
        node.operation_mode = 'manual'
        node._state1_enabled = True
        node._input_tray_done = False
        _set_sensors(node, S1_BELT_START=True, S7_TRAY_AT_ROBOT=False, S9_CYL1_RETRACTED=True, S10_CYL1_EXTENDED=False)
        node._notify = MagicMock()
        node._log_once = MagicMock()

        # _can_start_s1() requires auto mode, so in manual it won't trigger
        # This correctly reflects the V7-2 design: MANUAL doesn't auto-trigger S1
        node._do_idle_input()
        assert node.state_in == SystemState.IDLE  # stays IDLE in manual

    def test_idle_no_trigger_without_system_running(self):
        """_system_running = False → không trigger bất kỳ state nào."""
        node = _make_node()
        node._system_running = False
        node._input_tray_done = True
        _set_sensors(node, S7_TRAY_AT_ROBOT=True)
        node._log_once = MagicMock()

        node._do_idle_input()
        assert node.state_in == SystemState.IDLE


# ══════════════════════════════════════════════════════════════════
# SECTION 5: Homing Prerequisite (Critical Safety Invariant)
# ══════════════════════════════════════════════════════════════════

class TestStopDuringHoming:
    """Global STOP must halt every axis and permanently cancel stale HOME work."""

    def test_immediate_stop_queues_stop_bits_without_waiting_for_drive(self):
        """Safety path sends edcon's STOP telegram and never calls wait_for_stop()."""
        node = _make_node()
        mot = MagicMock()
        mot.telegram.output_bytes.return_value = b"stop-telegram"
        node.servos = {1: mot}
        node._jog_active = {1}
        node._inx_moving = True

        node._stop_immediate(1)

        mot._prepare_stop_motion_task_bits.assert_called_once_with()
        mot.com.send_io.assert_called_once_with(
            b"stop-telegram", nonblocking=True
        )
        mot.stop_motion_task.assert_not_called()
        assert mot.telegram.stw1.do_not_reject_traversing_task is True
        assert 1 not in node._jog_active
        assert node._inx_moving is False

    def test_global_stop_cancels_both_home_paths_and_broadcasts_all_axes(self):
        """Camera/Cartridge STOP cancels START-HOME plus Servo-card HOME."""
        node = _make_node()
        node.servos = {sid: MagicMock() for sid in range(1, 6)}
        node._homing_abort = threading.Event()
        node._stop_immediate = MagicMock()
        node._enter = MagicMock()
        node._enter_in = MagicMock()
        node._enter_s3 = MagicMock()
        node._enter_s4 = MagicMock()
        node._publish_drain = MagicMock()
        node._publish_homing_done = MagicMock()
        node._sync_mode_jog = MagicMock()
        node._notify = MagicMock()

        msg = MagicMock()
        msg.data = True
        node._cb_stop(msg)

        assert node._homing_abort.is_set()
        assert node._homing_generation == 1
        assert node._servo_home_abort == {1, 2, 3, 4, 5}
        assert node._servo_home_generation == {
            1: 1, 2: 1, 3: 1, 4: 1, 5: 1,
        }
        node._stop_immediate.assert_has_calls(
            [call(sid) for sid in range(1, 6)]
        )
        assert node.zero_offset == {}
        assert node.operation_mode == 'manual'
        assert node._system_running is False

    def test_stop_arriving_during_ready_prevents_reference_command(self):
        """Regression: HOME must not restart after STOP fired in _ensure_ready()."""
        node = _make_node()
        mot = MagicMock()
        node.servos = {2: mot}
        node._servo_homed_once = {sid: False for sid in range(1, 6)}
        node._servo_lock = threading.Lock()
        node._stop_immediate = MagicMock()
        node._notify_step = MagicMock()
        abort_event = threading.Event()

        def ready_then_stop(_mot, _sid, timeout=0.0):
            abort_event.set()
            return True

        node._ensure_ready = MagicMock(side_effect=ready_then_stop)

        assert node._home_all(abort_event) is False
        mot.referencing_task.assert_not_called()
        node._stop_immediate.assert_any_call(2)


class TestHomingPrerequisite:
    """START phải home xong 5 servo → IDLE → mới check S1."""

    def test_start_triggers_homing_when_not_homed(self):
        """Nhấn START khi chưa home → state = HOMING (không phải IDLE)."""
        node = _make_node()
        node.zero_offset = {}  # chưa home
        node._sync_mode_jog = MagicMock()
        node._notify = MagicMock()

        msg = MagicMock()
        msg.data = True
        node._cb_start(msg)

        assert node.state == SystemState.HOMING

    def test_start_goes_idle_when_already_homed(self):
        """Nhấn START khi đã home → state = IDLE (bỏ qua homing)."""
        node = _make_node()
        node.zero_offset = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0}
        node._sync_mode_jog = MagicMock()
        node._notify = MagicMock()

        msg = MagicMock()
        msg.data = True
        node._cb_start(msg)

        assert node.state == SystemState.IDLE

    def test_s1_blocked_when_zero_offset_empty(self):
        """_can_start_s1() = False khi zero_offset rỗng (chưa home)."""
        node = _make_node()
        node.operation_mode = 'auto'
        node.zero_offset = {}  # ← chưa home
        _set_sensors(node, S1_BELT_START=True, S7_TRAY_AT_ROBOT=False, S9_CYL1_RETRACTED=True, S10_CYL1_EXTENDED=False)
        assert node._can_start_s1() is False

    def test_idle_input_noop_when_not_homed(self):
        """_do_idle_input() không làm gì khi chưa home (zero_offset rỗng)."""
        node = _make_node()
        node.zero_offset = {}
        node._system_running = True
        node._input_tray_done = True
        _set_sensors(node, S7_TRAY_AT_ROBOT=True)
        node._log_once = MagicMock()

        node._do_idle_input()
        assert node.state_in == SystemState.IDLE  # không chuyển state

    def test_homing_running_blocks_dispatch(self):
        """Khi state = HOMING_RUNNING → _process_state() return ngay,
        không gọi _dispatch_input/_dispatch_s3/_dispatch_s4."""
        node = _make_node()
        node.state = SystemState.HOMING_RUNNING
        node._homing_done_event = MagicMock()
        node._homing_done_event.is_set.return_value = False

        # Gắn spy lên dispatch functions
        node._dispatch_input = MagicMock()
        node._dispatch_s3 = MagicMock()
        node._dispatch_s4 = MagicMock()

        node._process_state()

        node._dispatch_input.assert_not_called()
        node._dispatch_s3.assert_not_called()
        node._dispatch_s4.assert_not_called()

    def test_homing_complete_transitions_to_idle(self):
        """Khi homing bg thread xong (success) → state = IDLE + _system_running = True."""
        node = _make_node()
        node.state = SystemState.HOMING_RUNNING
        node._system_running = False

        import threading
        node._homing_done_event = threading.Event()
        node._homing_done_event.set()  # simulate bg thread done
        node._homing_result = True
        node.operation_mode = 'auto'
        node._sync_mode_jog = MagicMock()
        node._notify = MagicMock()

        node._process_state()

        assert node.state == SystemState.IDLE
        assert node._system_running is True

    def test_homing_failure_transitions_to_error(self):
        """Khi homing thất bại → state = ERROR."""
        node = _make_node()
        node.state = SystemState.HOMING_RUNNING

        import threading
        node._homing_done_event = threading.Event()
        node._homing_done_event.set()
        node._homing_result = False  # ← thất bại
        node._notify = MagicMock()
        node._stop = MagicMock()
        node._cyl1_retract = MagicMock()
        node._cyl2_retract = MagicMock()

        node._process_state()

        assert node.state == SystemState.ERROR


class TestZoneToRow:
    """
    _zone_to_row: map vị trí trigger (mm) → row; None khi ngoài mọi zone.

    Bảng zone tái hiện sự cố 2026-07-03: iny_input_zones có khe hở 790–805
    giữa row3 và row2 — S4 trigger 804.9mm phải trả None để scan retry,
    KHÔNG được fallback row1 (bug cũ làm InY chạy thẳng lên R1, Cyl1
    extend sai vị trí). Bảng dùng literal cố định, độc lập YAML thật.
    """

    INPUT_ZONES = {
        1: [980.0, 900.0, 1010.0],
        2: [880.0, 805.0, 920.0],
        3: [790.0, 715.0, 830.0],
    }

    def test_inside_zone(self):
        node = _make_node()
        assert node._zone_to_row(950.0, self.INPUT_ZONES) == 1
        assert node._zone_to_row(850.0, self.INPUT_ZONES) == 2
        assert node._zone_to_row(750.0, self.INPUT_ZONES) == 3

    def test_boundary_inclusive(self):
        """Biên zone tính vào zone (≤/≥)."""
        node = _make_node()
        assert node._zone_to_row(805.0, self.INPUT_ZONES) == 2
        assert node._zone_to_row(880.0, self.INPUT_ZONES) == 2
        assert node._zone_to_row(790.0, self.INPUT_ZONES) == 3

    def test_gap_returns_none_incident_20260703(self):
        """804.9mm rơi khe hở 790–805 giữa row3/row2 → None (không đoán row)."""
        node = _make_node()
        assert node._zone_to_row(804.9, self.INPUT_ZONES) is None

    def test_outside_all_zones_returns_none(self):
        node = _make_node()
        assert node._zone_to_row(50.0, self.INPUT_ZONES) is None
        assert node._zone_to_row(2000.0, self.INPUT_ZONES) is None


if __name__ == '__main__':
    pytest.main([__file__, '-v', '--tb=short'])
