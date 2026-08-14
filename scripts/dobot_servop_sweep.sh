#!/usr/bin/env bash
# Do xem ServoP streaming bam kip toi toc do nao — de chon tran toc do jog
# Cartesian trong robot_controller.cpp sendCartesianStep().
#
# Vi sao can do chu khong doan: jog Cartesian khong dung MoveJog (firmware tu
# choi, xem dobot_jog_probe.sh) ma stream setpoint ServoP moi 33ms. Toc do =
# buoc/chu ky, do GUI quyet dinh hoan toan. Neu buoc qua lon, controller bam
# khong kip -> robot TUT LAI SAU setpoint. Luc nha nut, GUI ngung stream nhung
# robot con chay not toi setpoint cuoi => VOT QUA diem nha tay. Do chinh la
# thu script nay do: ti so dat/lenh, va do vot sau khi ngung stream.
#
# Moi muc toc do: di 20mm theo truc chon, do, roi TU DONG QUAY VE diem xuat
# phat. Quang duong toi da moi lan la 20mm nen chi can 20mm khoang trong.
#
# LENH NAY LAM ROBOT CHUYEN DONG THAT.
#   - Bao dam trong 20mm theo huong chon (va 20mm nguoc lai).
#   - Tay canh nut dung khan.
#   - Robot dung stream va quay ve khi Ctrl+C.
#
#   ./dobot_servop_sweep.sh            # truc Z, len 20mm roi ve
#   ./dobot_servop_sweep.sh X 20       # truc X, 20mm
#   ./dobot_servop_sweep.sh Z 15       # truc Z, 15mm

set -uo pipefail

AXIS="${1:-Z}"
TRAVEL_MM="${2:-20}"

set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$HOME/ros2_ws/ros2_env.sh" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null || true
set -u

echo "Truc: $AXIS   quang duong moi lan: ${TRAVEL_MM}mm (di roi tu dong ve)"
echo "Bao dam co ${TRAVEL_MM}mm khoang trong theo CA HAI huong."
read -r -p "Enter de chay, Ctrl+C de huy... " _

exec python3 - "$AXIS" "$TRAVEL_MM" <<'PY'
import sys, time
import rclpy
from rclpy.node import Node
from dobot_msgs_v3.srv import ServoP
from dobot_msgs_v3.msg import ToolVectorActual

AXES = {"X": 0, "Y": 1, "Z": 2, "RX": 3, "RY": 4, "RZ": 5}
axis_name = sys.argv[1].upper()
travel = float(sys.argv[2])
if axis_name not in AXES:
    print("Truc phai la X/Y/Z/RX/RY/RZ", file=sys.stderr); sys.exit(1)
idx = AXES[axis_name]

TICK = 0.033                                   # dung chu ky cua jog_timer_
LEVELS = [24, 40, 60, 80, 100, 120]            # mm/s dinh muon thu

rclpy.init()
node = Node("dobot_servop_sweep")
cli = node.create_client(ServoP, "/nova5/dobot_bringup/ServoP")

pose = {}
def on_pose(m):
    pose["v"] = [m.x, m.y, m.z, m.rx, m.ry, m.rz]
node.create_subscription(ToolVectorActual, "/nova5/dobot_msgs_v3/msg/ToolVectorActual", on_pose, 10)

def spin(sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.005)

def send(p):
    req = ServoP.Request()
    req.x, req.y, req.z, req.rx, req.ry, req.rz = p
    cli.call_async(req)

def stream(start, delta, step):
    """Stream tu start den start+delta, buoc `step` moi TICK. Tra ve pose do duoc."""
    target = list(start)
    moved, n = 0.0, 0
    sign = 1.0 if delta > 0 else -1.0
    while abs(moved) < abs(delta):
        inc = sign * min(step, abs(delta) - abs(moved))
        target[idx] += inc
        moved += inc
        n += 1
        send(target)
        spin(TICK)
    return target, n

if not cli.wait_for_service(timeout_sec=5.0):
    print("Khong thay service ServoP — Dobot bringup chua chay?", file=sys.stderr)
    rclpy.shutdown(); sys.exit(1)
spin(1.0)
if "v" not in pose:
    print("Khong nhan duoc ToolVectorActual.", file=sys.stderr)
    rclpy.shutdown(); sys.exit(1)

home = list(pose["v"])
print(f"\nDiem xuat phat: {axis_name}={home[idx]:.3f}")
print(f"{'muon':>7} {'buoc':>7} {'lenh':>7} {'dat':>7} {'bam':>6} {'vot':>7}")
print(f"{'mm/s':>7} {'mm':>7} {'mm':>7} {'mm':>7} {'%':>6} {'mm':>7}")

try:
    for want in LEVELS:
        step = want * TICK
        before = pose["v"][idx]
        last_cmd, _ = stream(pose["v"], travel, step)

        spin(0.15)
        at_stop = pose["v"][idx]          # ngay khi ngung stream
        spin(1.0)
        settled = pose["v"][idx]          # sau khi robot chay not

        reached = settled - before
        track = 100.0 * reached / travel if travel else 0.0
        coast = settled - at_stop         # vot sau khi nha
        print(f"{want:7.0f} {step:7.3f} {travel:7.2f} {reached:7.2f} {track:6.1f} {coast:7.2f}")

        # quay ve dung diem xuat phat, luon o toc do cham nhat cho an toan
        stream(pose["v"], home[idx] - pose["v"][idx], LEVELS[0] * TICK)
        spin(1.0)
        input("   Enter de thu muc tiep (Ctrl+C de dung)... ")
finally:
    # luon tra ve diem xuat phat truoc khi thoat
    try:
        spin(0.2)
        if "v" in pose and abs(pose["v"][idx] - home[idx]) > 0.05:
            print("\n-> dang quay ve diem xuat phat...")
            stream(pose["v"], home[idx] - pose["v"][idx], LEVELS[0] * TICK)
            spin(1.0)
        print(f"-> {axis_name}={pose['v'][idx]:.3f} (xuat phat {home[idx]:.3f})")
    except Exception as e:
        print(f"!! khong quay ve duoc: {e}", file=sys.stderr)
    rclpy.shutdown()

print("\nDoc ket qua:")
print("  bam ~100%, vot ~0  -> muc nay an toan, dung lam tran")
print("  bam tut duoi ~95%  -> controller khong theo kip, DUNG vuot muc nay")
print("  vot lon dan        -> nha nut se troi qua diem mong muon")
PY
