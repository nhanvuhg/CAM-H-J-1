#!/usr/bin/env bash
# Do xem MoveJog co jog duoc theo truc Cartesian (X/Y/Z/Rx/Ry/Rz) khong.
#
# ===================== KET QUA DA DO 14/08/2026 =====================
# Chay tren robot that (Nova5, firmware hien tai). KET LUAN: KHONG.
#
#   MoveJog(X+,CoordType=1,User=0,Tool=0)  -> -50001   sai cu phap
#   MoveJog(X+,1,0,0)                      -> -50001   sai cu phap
#   MoveJog(X+)                            -> -6       truc khong hop le
#   MoveJog(J6+)                           ->  0       robot chay that
#   MoveJog()                              ->  0       lenh dung
#
# -50001 la ma "sai tham so" — chinh driver dung no de do firmware
# (dobot_api.py:307). Bare la cu phap DUY NHAT firmware nay nhan, ma o cu phap
# do chi ton tai ten truc khop. Khong truyen duoc CoordType => khong co duong
# nao bat MoveJog jog theo Cartesian => SpeedFactor khong the tac dong.
#
# Vi vay jog Cartesian giu ServoP streaming, va toc do duoc chia o phia GUI:
# robot_controller.cpp sendCartesianStep() nhan buoc voi speed_ratio_/100.
# Khong can chay lai script nay tru khi NANG CAP FIRMWARE.
# ====================================================================
#
# LENH NAY LAM ROBOT CHUYEN DONG THAT.
#   - Ha speed he thong xuong 10% trong GUI TRUOC khi chay.
#   - Dung tay canh nut dung khan.
#   - Robot se dung ke ca khi bi Ctrl+C.
#
#   ./dobot_jog_probe.sh                 # thu ca 3 bien the tren truc X+
#   ./dobot_jog_probe.sh Z+ 250          # truc khac, 250 ms
#   ./dobot_jog_probe.sh J6+ 200 bare    # doi chung duong: truc khop
#
# 14/08/2026 — viet lai loi do dung nguy hiem: ban cu goi "ros2 service call"
# cho tung lenh, moi lan ton ~1.5-2s discovery. Lenh STOP vi the den CHAM 2
# giay so voi y muon: burst 200ms thanh ~2s chuyen dong that (do duoc J6 xoay
# 20 do thay vi 3 do). Gio toan bo jog+sleep+stop chay trong MOT node rclpy
# duy nhat, discovery chi mot lan truoc khi bam gio, nen burst dung thoi luong.

set -uo pipefail

AXIS="${1:-X+}"
DURATION_MS="${2:-300}"
ONLY="${3:-}"

set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$HOME/ros2_ws/ros2_env.sh" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null || true
set -u

echo "Truc: $AXIS   moi lan jog: ${DURATION_MS} ms"
echo "Ha speed xuong 10% trong GUI truoc khi tiep tuc."
read -r -p "Enter de chay, Ctrl+C de huy... " _

# Toan bo phan cham vao robot nam trong day: mot node, discovery mot lan,
# stop nam trong finally nen moi duong thoat (loi, Ctrl+C, het gio) deu dung.
exec python3 - "$AXIS" "$DURATION_MS" "$ONLY" <<'PY'
import sys, time
import rclpy
from rclpy.node import Node
from dobot_msgs_v3.srv import MoveJog
from dobot_msgs_v3.msg import ToolVectorActual

axis, duration_ms, only = sys.argv[1], float(sys.argv[2]), sys.argv[3]

VARIANTS = [
    ("named",      ["CoordType=1", "User=0", "Tool=0"]),
    ("positional", ["1", "0", "0"]),
    ("bare",       []),
]

rclpy.init()
node = Node("dobot_jog_probe")
cli = node.create_client(MoveJog, "/nova5/dobot_bringup/MoveJog")

pose = {}
def on_pose(m):
    pose.update(x=m.x, y=m.y, z=m.z, rx=m.rx, ry=m.ry, rz=m.rz)
node.create_subscription(ToolVectorActual, "/nova5/dobot_msgs_v3/msg/ToolVectorActual", on_pose, 10)

def spin(sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)

def call(axis_id, params, wait=5.0):
    req = MoveJog.Request()
    req.axis_id = axis_id
    req.param_value = params
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=wait)
    return fut.result().res if fut.done() and fut.result() else None

def stop():
    return call("", [])

if not cli.wait_for_service(timeout_sec=5.0):
    print("Khong thay service MoveJog — Dobot bringup chua chay?", file=sys.stderr)
    rclpy.shutdown(); sys.exit(1)

spin(1.0)  # gom pose dau tien; discovery xong TRUOC khi bam gio
if not pose:
    print("Khong nhan duoc ToolVectorActual — khong do duoc chuyen dong.", file=sys.stderr)
    rclpy.shutdown(); sys.exit(1)

try:
    for label, params in VARIANTS:
        if only and only != label:
            continue
        print(f"\n{'='*62}\n[{label}]  MoveJog({axis}) param_value={params}\n{'='*62}")
        before = dict(pose)
        res = call(axis, params)
        spin(duration_ms / 1000.0)
        stop()
        spin(0.6)
        after = dict(pose)

        d = {k: after[k] - before[k] for k in after}
        lin = max(abs(d[k]) for k in ("x", "y", "z"))
        rot = max(abs(d[k]) for k in ("rx", "ry", "rz"))
        print("res  :", res)
        print("delta:", "  ".join(f"{k}={v:+.3f}" for k, v in d.items()))
        print(f"=> {'CO CHUYEN DONG' if lin > 0.15 or rot > 0.15 else 'KHONG chuyen dong'}"
              f"  (max lin {lin:.3f} mm, max rot {rot:.3f} deg)")
        if only:
            break
        input("\nEnter de thu bien the tiep... ")
finally:
    stop()
    print("\n-> da gui STOP")
    rclpy.shutdown()

print("\nKet luan:")
print("  bien the nao lam robot chay  -> dung param do trong jogStart()")
print("  khong bien the nao chay      -> giu ServoP, nhan buoc theo speed_ratio_")
PY
