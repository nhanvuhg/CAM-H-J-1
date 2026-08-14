#!/usr/bin/env bash
# Do xem ServoP streaming bam kip toi toc do nao — de chon tran toc do jog
# Cartesian trong robot_controller.cpp sendCartesianStep().
#
# Vi sao can do chu khong doan: jog Cartesian khong dung MoveJog (firmware tu
# choi, xem dobot_jog_probe.sh) ma stream setpoint ServoP moi 33ms. Toc do =
# buoc/chu ky, do GUI quyet dinh hoan toan. Neu buoc qua lon, controller bam
# khong kip -> robot TUT LAI SAU setpoint. Luc nha nut, GUI ngung stream nhung
# robot con chay not toi setpoint cuoi => VOT QUA diem nha tay.
#
# DOC KET QUA — hai cot quan trong:
#   dat mm/s : toc do THUC do tu ToolVectorActual trong luc stream. Neu no
#              khong tang nua du muon tang, controller da bao hoa.
#   vot mm   : vi tri vuot XA NHAT ra ngoai setpoint cuoi trong luc tat dao
#              dong. Day chinh la do troi qua diem nha nut. Muon no gan 0.
# Cot "lenh/ve dich" chi de doi chieu: ServoP la setpoint TUYET DOI nen sau khi
# on dinh robot luon ve toi diem cuoi, khong dung lam thuoc do bam duoc.
#
# Moi muc toc do: di 20mm theo truc chon, do, roi TU DONG QUAY VE diem xuat
# phat o toc do cham nhat. Quang duong toi da moi lan la 20mm.
#
# Luu y: ServoP KHONG chiu anh huong cua SpeedFactor (do la ly do ton tai cua
# ca cai fix nay), nen khong can ha speed he thong truoc khi chay.
#
# LENH NAY LAM ROBOT CHUYEN DONG THAT.
#   - Bao dam trong 20mm theo CA HAI chieu cua truc chon.
#   - Tay canh nut dung khan.
#   - Robot quay ve diem xuat phat ke ca khi bi Ctrl+C.
#
#   ./dobot_servop_sweep.sh            # truc Z, 20mm
#   ./dobot_servop_sweep.sh X 20       # truc X, 20mm
#   ./dobot_servop_sweep.sh Z 20 60,80,100   # chi vai muc
#
# Khong chay tu terminal (stdin khong phai tty) thi bo qua cac cho dung hoi.

set -uo pipefail

AXIS="${1:-Z}"
LEVELS_ARG="${3:-}"
TRAVEL_MM="${2:-20}"

set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$HOME/ros2_ws/ros2_env.sh" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null || true
set -u

echo "Truc: $AXIS   quang duong moi muc: ${TRAVEL_MM}mm (di roi tu dong ve)"
if [ -t 0 ]; then
    echo "Bao dam co ${TRAVEL_MM}mm khoang trong theo CA HAI huong."
    read -r -p "Enter de chay, Ctrl+C de huy... " _
fi

exec python3 - "$AXIS" "$TRAVEL_MM" ${3:+"$3"} <<'PY'
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
unit = "mm" if idx < 3 else "deg"
interactive = sys.stdin.isatty()

TICK = 0.033                                   # dung chu ky cua jog_timer_
LEVELS = [float(x) for x in sys.argv[3].split(",")] if len(sys.argv) > 3 \
         else [24, 40, 60, 80, 100, 120]

rclpy.init()
node = Node("dobot_servop_sweep")
cli = node.create_client(ServoP, "/nova5/dobot_bringup/ServoP")

pose = {}
samples = []          # (thoi diem, vi tri truc dang do) — de tinh toc do that
def on_pose(m):
    v = [m.x, m.y, m.z, m.rx, m.ry, m.rz]
    pose["v"] = v
    samples.append((time.monotonic(), v[idx]))
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
    """Stream tu start den start+delta, buoc `step` moi TICK."""
    target = list(start)
    moved = 0.0
    sign = 1.0 if delta > 0 else -1.0
    while abs(moved) < abs(delta) - 1e-9:
        inc = sign * min(step, abs(delta) - abs(moved))
        target[idx] += inc
        moved += inc
        send(target)
        spin(TICK)
    return target

def peak_speed(pts, window=0.05):
    """Toc do lon nhat do duoc tren cua so >= `window` giay.

    Lay cua so thay vi hai mau lien tiep: pose ve 96Hz nen hai mau chi cach
    ~10ms, nhieu do luong se thoi phong toc do len."""
    best = 0.0
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            dt = pts[j][0] - pts[i][0]
            if dt < window:
                continue
            best = max(best, abs(pts[j][1] - pts[i][1]) / dt)
            break
    return best

if not cli.wait_for_service(timeout_sec=5.0):
    print("Khong thay service ServoP — Dobot bringup chua chay?", file=sys.stderr)
    rclpy.shutdown(); sys.exit(1)
spin(1.0)
if "v" not in pose:
    print("Khong nhan duoc ToolVectorActual.", file=sys.stderr)
    rclpy.shutdown(); sys.exit(1)

home = list(pose["v"])
print(f"\nDiem xuat phat: {axis_name}={home[idx]:.3f}{unit}\n")
print(f"{'muon':>6} {'buoc':>7} {'dat':>8} {'so voi':>7} {'ve dich':>8} {'vot':>7}")
print(f"{unit+'/s':>6} {unit:>7} {unit+'/s':>8} {'muon':>7} {unit:>8} {unit:>7}")
print("-" * 50)

results = []
try:
    for want in LEVELS:
        step = want * TICK
        before = pose["v"][idx]

        samples.clear()
        last = stream(pose["v"], travel, step)
        during = list(samples)            # mau trong luc stream
        final = last[idx]                 # setpoint CUOI cung da gui
        sign = 1.0 if travel > 0 else -1.0

        # Do vot that: vi tri vuot XA NHAT ra ngoai setpoint cuoi trong suot
        # qua trinh tat dao dong. Ban truoc lay hieu tai mot moc co dinh 150ms
        # nen roi vao pha khac nhau tuy toc do -> so am, khong don dieu.
        samples.clear()
        spin(1.4)
        overshoot = max((sign * (p - final) for _, p in samples), default=0.0)
        overshoot = max(0.0, overshoot)
        settled = pose["v"][idx]

        got = peak_speed(during)
        ratio = 100.0 * got / want if want else 0.0
        reached = settled - before
        results.append((want, got, ratio, overshoot))
        print(f"{want:6.0f} {step:7.3f} {got:8.1f} {ratio:6.0f}% {reached:8.2f} {overshoot:7.2f}")

        # quay ve dung diem xuat phat, luon o toc do cham nhat cho an toan
        stream(pose["v"], home[idx] - pose["v"][idx], LEVELS[0] * TICK)
        spin(1.2)
        if interactive:
            input("   Enter de thu muc tiep (Ctrl+C de dung)... ")
finally:
    try:
        spin(0.2)
        if "v" in pose and abs(pose["v"][idx] - home[idx]) > 0.05:
            print("\n-> dang quay ve diem xuat phat...")
            stream(pose["v"], home[idx] - pose["v"][idx], LEVELS[0] * TICK)
            spin(1.2)
        print(f"\n-> ket thuc tai {axis_name}={pose['v'][idx]:.3f} (xuat phat {home[idx]:.3f})")
    except Exception as e:
        print(f"!! khong quay ve duoc: {e}", file=sys.stderr)
    rclpy.shutdown()

if results:
    ok = [r for r in results if r[2] >= 90 and abs(r[3]) <= 0.5]
    print("\nDoc ket qua:")
    print("  'so voi muon' tut duoi ~90%  -> controller da bao hoa, tang nua vo ich")
    print("  'vot' lon dan                -> nha nut se troi qua diem mong muon")
    if ok:
        print(f"\n=> muc cao nhat con dat >=90% va vot <=0.5{unit}: {ok[-1][0]:.0f}{unit}/s")
    else:
        print("\n=> khong muc nao dat chuan, giu nguyen tran hien tai")
PY
