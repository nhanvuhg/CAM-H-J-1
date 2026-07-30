#!/usr/bin/env python3
"""
VFD Logic Node — điều khiển biến tần ATV320 (băng tải input).

Quy tắc chạy (theo mode hệ thống):

    AUTO / AI mode (op_mode = 1/2):
        Kiểm tra S1/S2 LIÊN TỤC, không phụ thuộc state machine.
            S3 ON                 → STOP (ưu tiên cao nhất)
            S1 ON hoặc S2 ON      → RUN
            Cả 3 OFF              → giữ trạng thái hiện tại (gap giữa khay)

    MANUAL mode (op_mode = 3):
        Chỉ chạy khi user nhấn nút STATE 1 → cartridge enter state_in='s1_*'.
        Trong STATE 1, áp dụng cùng logic sensor như AUTO.
        Ngoài STATE 1 (idle/đang ở step khác) → STOP.

    IDLE/ERROR (op_mode khác) → STOP.

Chạy song song với servo — không block state machine, không đợi nhau.

Input:
    /providesystem/sensors_state (String, char 0/1/2 = S1/S2/S3)
    /system_state                (String, format "global|state_in|state_out")
    /robot/set_mode              (Int32, 1=auto, 2=ai, 3=manual)
Output:
    /vfd/cmd_run (Bool) → rs485_bus_node trên RevPi A → Modbus RTU → ATV320
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Int32, String
import time


class VfdLogicNode(Node):
    MODE_AUTO   = 1
    MODE_AI     = 2
    MODE_MANUAL = 3

    def __init__(self):
        super().__init__('vfd_logic_node')

        self.pub_run = self.create_publisher(Bool, '/vfd/cmd_run', 10)
        self.create_subscription(
            String, '/providesystem/sensors_state', self.sensors_cb, 10)
        self.create_subscription(
            String, '/system_state', self.state_cb, 10)
        self.create_subscription(
            Int32, '/robot/set_mode', self.mode_cb, 10)
        # homing_done: cartridge publish latching (True khi homing xong, False
        # ở init + khi clear zero_offset do STOP/mode-switch/ERROR). QoS phải
        # match TRANSIENT_LOCAL để start sau vẫn nhận state cuối.
        latching = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, '/cartridge/homing_done', self.homing_cb, latching)

        self.s1_state = False
        self.s2_state = False
        self.s3_state = False
        self.in_state1 = False        # state_in startswith 's1_' (cartridge đang ở STATE 1)
        self.cartridge_homed = False  # False cho tới khi cartridge publish homing_done=True
        self.op_mode = self.MODE_MANUAL  # default — match cartridge default
        self.current_cmd = False
        self.run_started_at = None
        self.run_timeout_s = float(self.declare_parameter('run_timeout_s', 30.0).value)

        self.publish_cmd(False)
        self.create_timer(0.5, self._heartbeat)

        self.get_logger().info(
            f"VFD Logic Node started — AUTO/AI: gate by homing+S1/S2; MANUAL: gate STATE 1; timeout={self.run_timeout_s:.1f}s")

    def homing_cb(self, msg):
        if msg.data == self.cartridge_homed:
            return
        self.cartridge_homed = msg.data
        self.get_logger().info(
            f"Cartridge homed = {self.cartridge_homed} → {'enable' if self.cartridge_homed else 'BLOCK'} sensor check")
        self.evaluate_logic()

    def mode_cb(self, msg):
        if msg.data == self.op_mode:
            return
        prev = self.op_mode
        self.op_mode = msg.data
        self.get_logger().info(
            f"Mode {self._mode_name(prev)} → {self._mode_name(self.op_mode)}")
        self.evaluate_logic()

    def _mode_name(self, m):
        return {1: 'AUTO', 2: 'AI', 3: 'MANUAL'}.get(m, f'?({m})')

    def state_cb(self, msg):
        """Parse /system_state ('global|state_in|state_out') → set in_state1."""
        parts = msg.data.split('|')
        state_in = parts[1] if len(parts) >= 2 else ''
        was = self.in_state1
        # STATE 1 = mọi sub-state bắt đầu bằng "s1_" và đang active (không phải
        # s1_complete). Chỉ relevant cho MANUAL mode gate.
        self.in_state1 = state_in.startswith('s1_') and state_in != 's1_complete'
        if was != self.in_state1:
            self.evaluate_logic()
            if self.op_mode == self.MODE_MANUAL:
                self.get_logger().info(
                    f"MANUAL gate STATE 1 {'OPENED' if self.in_state1 else 'CLOSED'} (state_in={state_in})")

    def sensors_cb(self, msg):
        if len(msg.data) < 3:
            return
        s1 = msg.data[0] == '1'
        s2 = msg.data[1] == '1'
        s3 = msg.data[2] == '1'

        if (s1, s2, s3) == (self.s1_state, self.s2_state, self.s3_state):
            return
        self.s1_state, self.s2_state, self.s3_state = s1, s2, s3
        self.evaluate_logic()

    def _sensor_decision(self, allow_carry=True):
        """Logic sensor chung: S3 ưu tiên dừng, S1|S2 ON → RUN, không thì giữ."""
        if self.s3_state:
            return False, "S3 ON"
        if self.s1_state or self.s2_state:
            return True, "S1/S2 ON"
        if allow_carry:
            return self.current_cmd, "carry-over (gap)"
        return False, "S1/S2 recheck OFF"

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # ⚠️  CRITICAL ZONE — đọc memory feedback_critical_code_zones.md trước khi sửa.
    # INVARIANT (user rule):
    #   - AUTO/AI: PHẢI cartridge_homed=True (user nhấn START → homing xong)
    #     RỒI mới check S1/S2 LIÊN TỤC. Chưa homing → STOP dù S1/S2 ON.
    #   - MANUAL: GIỮ gate in_state1 (chỉ chạy khi user nhấn nút STATE 1)
    #   - S3 luôn ưu tiên STOP; cả 3 ON → vẫn STOP
    # Đừng unify 2 mode hoặc bỏ subscribe /robot/set_mode hoặc bỏ
    # subscription /cartridge/homing_done.
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    def evaluate_logic(self, allow_carry=True):
        if self.op_mode in (self.MODE_AUTO, self.MODE_AI):
            if not self.cartridge_homed:
                new_cmd, reason = False, f"{self._mode_name(self.op_mode)} chưa homing (chờ START)"
            else:
                new_cmd, sub = self._sensor_decision(allow_carry)
                reason = f"{self._mode_name(self.op_mode)} {sub}"
        elif self.op_mode == self.MODE_MANUAL:
            if not self.in_state1:
                new_cmd, reason = False, "MANUAL gate CLOSED (not STATE 1)"
            else:
                new_cmd, sub = self._sensor_decision(allow_carry)
                reason = f"MANUAL STATE 1 {sub}"
        else:
            new_cmd, reason = False, f"mode={self._mode_name(self.op_mode)}"

        if new_cmd != self.current_cmd:
            self.current_cmd = new_cmd
            self.run_started_at = time.monotonic() if new_cmd else None
            self.publish_cmd(self.current_cmd)
            self.get_logger().info(
                f"VFD → {'RUN' if new_cmd else 'STOP'} ({reason})")

    def publish_cmd(self, run_state):
        msg = Bool()
        msg.data = run_state
        self.pub_run.publish(msg)

    def _heartbeat(self):
        self._check_run_timeout()
        # Re-publish hiện tại để rs485_bus_node trên RevPi A đồng bộ
        # sau reconnect/restart.
        self.publish_cmd(self.current_cmd)

    def _check_run_timeout(self):
        if not self.current_cmd or self.run_started_at is None or self.s3_state:
            return
        elapsed = time.monotonic() - self.run_started_at
        if elapsed < self.run_timeout_s:
            return

        # Noise guard: S1/S2 can blip ON once, then all sensors go OFF. The
        # normal gap carry-over would keep RUN forever if S3 never arrives.
        # Stop once, then re-evaluate the real S1/S2 state without carry-over.
        self.current_cmd = False
        self.run_started_at = None
        self.publish_cmd(False)
        self.get_logger().warn(
            f"VFD safety timeout {self.run_timeout_s:.1f}s without S3 → STOP, recheck S1/S2")
        self.evaluate_logic(allow_carry=False)


def main(args=None):
    rclpy.init(args=args)
    node = VfdLogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
