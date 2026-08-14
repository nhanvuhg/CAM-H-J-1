#!/usr/bin/env bash
# Thu xem MoveJog co jog duoc theo truc Cartesian (X/Y/Z/Rx/Ry/Rz) khong.
#
# Vi sao can: jog Cartesian trong GUI dang chay bang ServoP streaming voi buoc
# hard-code 0.5mm/33ms, nen SpeedFactor khong tac dong — keo speed len van 15mm/s.
# Neu MoveJog nhan duoc truc Cartesian thi bo duoc ServoP va speed se an nhu
# jog theo khop.
#
# Nghi ngo: driver gui "MoveJog(X+)" khong kem CoordType. Giao thuc Dobot V4
# mac dinh CoordType=0 (khop), ma o che do khop thi "X+" khong phai ten truc
# hop le -> controller bo qua.
#
# LENH NAY LAM ROBOT CHUYEN DONG THAT.
#   - Ha speed he thong xuong 10% trong GUI TRUOC khi chay.
#   - Dung tay canh nut dung khan.
#   - Script luon gui lenh STOP khi thoat, ke ca khi bi Ctrl+C.
#
#   ./dobot_jog_probe.sh                 # thu ca 3 bien the tren truc X+
#   ./dobot_jog_probe.sh Z+ 250          # truc khac, 250 ms
#   ./dobot_jog_probe.sh X+ 300 named    # chi mot bien the

set -uo pipefail

AXIS="${1:-X+}"
DURATION_MS="${2:-300}"
ONLY="${3:-}"
SRV="/nova5/dobot_bringup/MoveJog"
TYPE="dobot_msgs_v3/srv/MoveJog"
LOG="$HOME/ros2_ws/logs/dobot_bringup.log"

# set -u phai tat quanh day: script setup cua ROS tham chieu bien chua dat va
# se giet script ngay dong dau (start_all.sh cung lam the).
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$HOME/ros2_ws/ros2_env.sh" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null || true
set -u

stop_jog() {
    # axis_id rong = lenh dung cua Dobot. Goi vo dieu kien, ke ca khi chua jog.
    ros2 service call "$SRV" "$TYPE" "{axis_id: '', param_value: []}" >/dev/null 2>&1 || true
}
trap 'echo; echo "-> gui STOP"; stop_jog' EXIT INT TERM

if ! ros2 service list 2>/dev/null | grep -qx "$SRV"; then
    echo "Khong thay service $SRV — Dobot bringup chua chay?" >&2
    exit 1
fi

echo "Truc: $AXIS   moi lan jog: ${DURATION_MS} ms"
echo "Ha speed xuong 10% trong GUI truoc khi tiep tuc."
read -r -p "Enter de chay, Ctrl+C de huy... " _

probe() {
    local label="$1"; shift
    local params="$1"
    [ -n "$ONLY" ] && [ "$ONLY" != "$label" ] && return 0

    echo
    echo "=============================================================="
    echo "[$label]  MoveJog($AXIS) params=$params"
    echo "=============================================================="
    local before
    before=$(wc -l < "$LOG" 2>/dev/null || echo 0)

    ros2 service call "$SRV" "$TYPE" \
        "{axis_id: '$AXIS', param_value: $params}" 2>&1 | grep -E "response|res=" || true

    sleep "$(awk "BEGIN{print $DURATION_MS/1000}")"
    stop_jog

    echo "--- driver tra ve ---"
    tail -n +"$((before + 1))" "$LOG" 2>/dev/null | grep -iE "movejog|errorid|,\{" | tail -5
    echo
    read -r -p "Robot CO chuyen dong khong? Enter de thu bien the tiep... " _
}

# 1. Cu phap co ten cua giao thuc V4 — kha nang cao nhat.
probe named "['CoordType=1','User=0','Tool=0']"
# 2. Cung tham so nhung dat theo vi tri, phong khi firmware doi kieu cu.
probe positional "['1','0','0']"
# 3. Doi chung: dung y het GUI hien tai. Neu cai nay cung chay thi van de
#    nam o cho khac, khong phai thieu CoordType.
probe bare "[]"

echo
echo "Xong. Ket luan:"
echo "  bien the nao lam robot chay  -> dung param do trong jogStart()"
echo "  khong bien the nao chay      -> giu ServoP, nhan buoc theo speed_ratio_"
