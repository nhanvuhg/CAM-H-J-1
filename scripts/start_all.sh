#!/usr/bin/env bash
set -uo pipefail

# ═══════════════════════════════════════════════════════════
# 🚀 START ALL — Full System Launcher
# ═══════════════════════════════════════════════════════════
# Starts:
#   1. cartridge_providesystem_py  — Servo control (Festo CMMT-AS)
#   2. vfd_logic_node              — VFD belt auto/sensor control
#   3. dobot_bringup_v3            — Dobot Nova5 driver
#   4. robot_logic + motion_exec   — Robot pick-and-place logic
#   5. gripper_festo_node          — Festo gripper (venv)
#   6. dual_camera_system          — 2× V4L2 CUDA camera + 2× TensorRT YOLO
#   7. cartridge_gui.py            — HTML GUI (port 8080, optional)
#   8. unified_control_gui         — QML GUI (HDMI)
#   9. rs485_bus_node              — RevPi A (remote via SSH)
#  10. loadcell_node               — RevPi A 4-20mA (remote via SSH)
#
# Usage: bash start_all.sh [--web] [--no-web]
# Stop:  Ctrl+C (kills all)
# ═══════════════════════════════════════════════════════════

WS="$HOME/ros2_ws"
export DISPLAY=${DISPLAY:-:0}
export FILL_HP_USERS_FILE="${FILL_HP_USERS_FILE:-$WS/src/unified_control_gui/fill_hp_users.json}"
CAM0_AI_MODEL="${CAM0_AI_MODEL:-$HOME/models/data_input_hp1.engine}"
CAM1_AI_MODEL="${CAM1_AI_MODEL:-$HOME/models/data_output_hp1.engine}"
CAMERA_CAPTURE_FPS="${CAMERA_CAPTURE_FPS:-25}"
AI_MAX_FPS="${AI_MAX_FPS:-20}"

# Keep Qt geometry identical to the RevPi reference display. Jetson's X server
# reports 92x91 DPI while the RevPi reports 96x96; explicit 1:1 scaling avoids
# Qt deriving different horizontal/vertical metrics from those values.
export QT_AUTO_SCREEN_SCALE_FACTOR="${QT_AUTO_SCREEN_SCALE_FACTOR:-0}"
export QT_SCALE_FACTOR="${QT_SCALE_FACTOR:-1}"
export QT_FONT_DPI="${QT_FONT_DPI:-96}"

# ── Auto-detect XAUTHORITY (cần thiết khi chạy từ SSH) ──
if [ -z "${XAUTHORITY:-}" ]; then
    if [ -f "$HOME/.Xauthority" ]; then
        export XAUTHORITY="$HOME/.Xauthority"
    elif [ -f "/run/user/$(id -u)/gdm/Xauthority" ]; then
        export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
    fi
fi
echo "🖥️  Display: DISPLAY=$DISPLAY  XAUTHORITY=${XAUTHORITY:-<not set>}"

# ── Source ROS 2 + env ──
set +u
# Single source of truth — ros2_env.sh set ROS_DOMAIN_ID, RMW, FastDDS
# profile. File này cũng được source từ ~/.bashrc (xem chú thích trong
# file) → mọi terminal đã có env trước khi gọi start_all.sh. Source lại
# ở đây chỉ guard trường hợp start_all chạy từ context không bashrc
# (vd cron, systemd, .desktop file launch).
source "$WS/ros2_env.sh"
echo "ℹ️  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  FastDDS=${FASTRTPS_DEFAULT_PROFILES_FILE:-<default>}"

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "❌ /opt/ros/humble/setup.bash not found — Jetson launcher requires ROS 2 Humble"
    exit 1
fi
source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ]  && source "$WS/install/setup.bash"  || echo "⚠️  $WS/install/setup.bash not found — run: colcon build"
set -u

# Kiểm tra binary có sẵn không
if [ ! -f "$WS/install/unified_control_gui/lib/unified_control_gui/unified_control_gui" ]; then
    echo "❌ unified_control_gui binary not found. Chạy: cd ~/ros2_ws && colcon build --packages-select unified_control_gui"
    exit 1
fi
if [ ! -x "$WS/install/yolo_tensorrt_ros2/lib/yolo_tensorrt_ros2/yolo_tensorrt_node" ]; then
    echo "❌ TensorRT YOLO node not found. Chạy: cd ~/ros2_ws && colcon build --packages-select yolo_tensorrt_ros2 csi_camera"
    exit 1
fi
for ai_model in "$CAM0_AI_MODEL" "$CAM1_AI_MODEL"; do
    if [ ! -r "$ai_model" ]; then
        echo "❌ TensorRT model not found/readable: $ai_model"
        exit 1
    fi
done

LOG_DIR="$WS/logs"
mkdir -p "$LOG_DIR"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🚀 Full System — Cartridge + Robot Launcher"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# ─── Anti-parallel guard (Desktop double-click / SSH launcher) ───
# A kernel-backed lock avoids false matches from wrapper shells whose command
# line happens to contain "start_all.sh". The lock is released automatically
# when this launcher exits.
LOCKFILE="/tmp/cartridge_system.lock"
exec 9>"$LOCKFILE"
if ! flock -n 9; then
    echo "🛑 start_all.sh is already running — keeping the existing instance"
    exit 1
fi

# ─── Cleanup process cũ (gộp 3 vòng pkill thành 1) ───
PIDFILE="/tmp/cartridge_system.pid"
NODE_PATTERNS=(
    "cartridge_providesystem_py"
    "cartridge_gui.py"
    "unified_control_gui/unified_control_gui"
    "robot_logic_node"
    "motion_executor"
    "dobot_bringup"
    "gripper_festo_node"
    "dual_csi_camera"
    "csi_camera_node"
    "v4l2_dual_camera_cuda_node"
    "v4l2_dual_camera_node"
    "dual_camera_system"
    # KHÔNG thêm "rpicam-vid" vào broad pkill -9 — sẽ tạo CFE zombie state
    # (kernel không kịp release /dev/media0). Graceful kill xử lý riêng bên dưới.
    "overlay_bboxes_node"
    "vision_decision_node"
    "yolo_tensorrt_node"
    "component_container"
    "vfd_logic_node"
)

camera_stack_pids() {
    {
        pgrep -f "^${WS}/install/csi_camera/lib/csi_camera/v4l2_dual_camera_cuda_node([[:space:]]|$)" 2>/dev/null || true
        pgrep -f "^${WS}/install/csi_camera/lib/csi_camera/v4l2_dual_camera_node([[:space:]]|$)" 2>/dev/null || true
        pgrep -f "^${WS}/install/yolo_tensorrt_ros2/lib/yolo_tensorrt_ros2/yolo_tensorrt_node([[:space:]]|$)" 2>/dev/null || true
        pgrep -f "^${WS}/install/bbox_drawer_cpp/lib/bbox_drawer_cpp/overlay_bboxes_node([[:space:]]|$)" 2>/dev/null || true
        pgrep -f "^${WS}/install/robot_control_main/lib/robot_control_main/vision_decision_node([[:space:]]|$)" 2>/dev/null || true
    } | sort -nu
}

stop_jetson_camera_nodes() {
    local camera_pids
    local remaining_pids
    camera_pids=$(camera_stack_pids)
    [ -z "$camera_pids" ] && return

    echo "📷 Stopping camera stack gracefully..."
    kill -TERM $camera_pids 2>/dev/null || true
    local camera_wait=0
    while true; do
        remaining_pids=""
        for camera_pid in $camera_pids; do
            if kill -0 "$camera_pid" 2>/dev/null; then
                remaining_pids="$remaining_pids $camera_pid"
            fi
        done
        [ -z "$remaining_pids" ] && break
        if [ "$camera_wait" -ge 6 ]; then
            echo "⚠️  Forcing remaining camera stack PIDs:$remaining_pids"
            kill -KILL $remaining_pids 2>/dev/null || true
            break
        fi
        sleep 1
        camera_wait=$((camera_wait + 1))
    done
}

cleanup_fastdds_shm() {
    # SHM segments are created per FastDDS participant. They are safe to remove
    # only when no local ROS participant still has the file open.
    for participant in cartridge_providesystem_py vfd_logic_node.py \
        unified_control_gui dobot_bringup robot_logic_node motion_executor \
        v4l2_dual_camera_cuda_node v4l2_dual_camera_node yolo_tensorrt_node \
        overlay_bboxes_node vision_decision_node; do
        if pgrep -f "$participant" >/dev/null 2>&1; then
            echo "🧹 FastDDS cleanup skipped: participant still running ($participant)"
            return
        fi
    done

    ros2 daemon stop 2>/dev/null || true
    local removed=0
    local segment
    shopt -s nullglob
    for segment in /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*; do
        if fuser "$segment" >/dev/null 2>&1; then
            echo "🧹 Keeping active FastDDS segment: $segment"
            continue
        fi
        if unlink -- "$segment" 2>/dev/null; then
            removed=$((removed + 1))
        fi
    done
    shopt -u nullglob
    echo "🧹 Removed $removed stale FastDDS SHM segments"
}

NEED_CLEANUP=0
for p in "${NODE_PATTERNS[@]}"; do
    if pgrep -f "$p" >/dev/null 2>&1; then NEED_CLEANUP=1; break; fi
done
[ -f "$PIDFILE" ] && NEED_CLEANUP=1

if [ "$NEED_CLEANUP" -eq 1 ]; then
    echo "🔍 Cleanup process cũ..."
    # Bước 1: TERM theo PIDFILE (graceful, có thời gian flush log/đóng socket)
    if [ -f "$PIDFILE" ]; then
        OLD_PIDS=$(cat "$PIDFILE" 2>/dev/null || true)
        for pid in $OLD_PIDS; do
            kill -0 "$pid" 2>/dev/null && kill -TERM "$pid" 2>/dev/null || true
        done
        sleep 1
        stop_jetson_camera_nodes
        for pid in $OLD_PIDS; do
            kill -9 "$pid" 2>/dev/null || true
        done
        rm -f "$PIDFILE"
    fi
    # Bước 2: pkill -9 thẳng các pattern (catch-all cho process không có trong PIDFILE)
    for p in "${NODE_PATTERNS[@]}"; do
        pkill -9 -f "$p" 2>/dev/null || true
    done
    pkill -9 -f "192.168.27" 2>/dev/null || true
    fuser -k 29999/tcp 2>/dev/null || true

    sleep 1

    # Đợi process chính chết (giảm 12s → 4s — pkill -9 thường < 1s)
    _wait=0
    while pgrep -f "cartridge_providesystem_py" >/dev/null 2>&1; do
        if [ $_wait -ge 4 ]; then
            echo "⚠️  Process vẫn còn sau 4s — tiếp tục"
            break
        fi
        sleep 1
        _wait=$((_wait + 1))
    done

    # Đợi TCP đến servo/IO :502 đóng (giảm 30s → 5s — TIME_WAIT trên local
    # KHÔNG block outbound mới vì port destination khác, chỉ cosmetic).
    _wait=0
    while ss -tn state established 2>/dev/null | grep -qE "192\.168\.27\.(24[89]|25[0-3]):502"; do
        if [ $_wait -ge 5 ]; then
            break
        fi
        sleep 1
        _wait=$((_wait + 1))
    done

    echo "✅ Clean slate"
else
    echo "✅ No old processes — skip cleanup"
fi
cleanup_fastdds_shm
echo ""

# ── PIDs ──
PID_PROVIDE=""
PID_VFD_LOGIC=""
PID_DOBOT=""
PID_ROBOT=""
PID_GRIPPER=""
PID_CAMERA=""
PID_WEB_GUI=""
PID_QML_GUI=""
CAMERA_SUPERVISOR_STATUS="/tmp/camera_stack_supervisor.status"

_CLEANUP_DONE=0
cleanup() {
    [ "$_CLEANUP_DONE" -eq 1 ] && return
    _CLEANUP_DONE=1
    echo ""
    echo "🛑 Shutting down..."
    rm -f "$PIDFILE"
    for pid in "${PID_QML_GUI:-}" "${PID_WEB_GUI:-}" "${PID_CAMERA:-}" "${PID_GRIPPER:-}" "${PID_ROBOT:-}" "${PID_DOBOT:-}" "${PID_PROVIDE:-}" "${PID_VFD_LOGIC:-}"; do
        [ -n "$pid" ] && kill "$pid" 2>/dev/null || true
    done
    sleep 2
    stop_jetson_camera_nodes
    for pid in "${PID_QML_GUI:-}" "${PID_WEB_GUI:-}" "${PID_CAMERA:-}" "${PID_GRIPPER:-}" "${PID_ROBOT:-}" "${PID_DOBOT:-}" "${PID_PROVIDE:-}" "${PID_VFD_LOGIC:-}"; do
        [ -n "$pid" ] && kill -9 "$pid" 2>/dev/null || true
    done
    # Fallback kill all known process names
    pkill -9 -f "cartridge_providesystem_py" 2>/dev/null || true
    pkill -9 -f "cartridge_gui.py" 2>/dev/null || true
    pkill -9 -f "unified_control_gui/unified_control_gui" 2>/dev/null || true
    pkill -9 -f "vfd_logic_node.py" 2>/dev/null || true
    pkill -9 -f "robot_logic_node" 2>/dev/null || true
    pkill -9 -f "motion_executor" 2>/dev/null || true
    pkill -9 -f "dobot_bringup" 2>/dev/null || true
    pkill -9 -f "gripper_festo_node" 2>/dev/null || true
    pkill -9 -f "dual_csi_camera" 2>/dev/null || true
    pkill -9 -f "dual_camera_system" 2>/dev/null || true
    pkill -9 -f "overlay_bboxes_node" 2>/dev/null || true
    pkill -9 -f "vision_decision_node" 2>/dev/null || true
    pkill -9 -f "yolo_tensorrt_node" 2>/dev/null || true
    pkill -9 -f "component_container" 2>/dev/null || true
    echo "✅ All processes stopped."
}
trap cleanup EXIT INT TERM HUP QUIT

# ══════════════════════════════════════════
# WAVE 1 — All hardware-facing nodes start in parallel.
# Each node connects to its own hardware concurrently (cartridge: 5 servo
# + 2 IO modules in threads; dobot: command+feedback sockets parallel via
# nova5.launch.py). Total startup ≈ slowest single node, not the sum.
# Inter-node ROS service deps are handled by service_is_ready / async
# clients inside the nodes — no sleep needed.
# ══════════════════════════════════════════

# ── [1] Cartridge Provide System Node ──
LOG_PROVIDE="$LOG_DIR/cartridge_node.log"
CARTRIDGE_BIN="$WS/install/system_feed_cartridge/lib/system_feed_cartridge/cartridge_providesystem_py"
echo "  [1] 🔧 Cartridge Provide System Node..."
"$CARTRIDGE_BIN" > "$LOG_PROVIDE" 2>&1 &
PID_PROVIDE=$!
echo "        PID=$PID_PROVIDE  Log: $LOG_PROVIDE"
echo "$PID_PROVIDE" > "$PIDFILE"

# ── [2] VFD Logic Node ──
LOG_VFD_LOGIC="$LOG_DIR/vfd_logic_node.log"
VFD_LOGIC_PY="$WS/install/unified_control_gui/lib/unified_control_gui/vfd_logic_node.py"
echo "  [2] 📡 VFD Logic Node..."
python3 "$VFD_LOGIC_PY" > "$LOG_VFD_LOGIC" 2>&1 &
PID_VFD_LOGIC=$!
echo "        PID=$PID_VFD_LOGIC  Log: $LOG_VFD_LOGIC"
echo "$PID_VFD_LOGIC" >> "$PIDFILE"

# ── [3] Dobot Bringup (command port 29999 + feedback 30004 in parallel) ──
LOG_DOBOT="$LOG_DIR/dobot_bringup.log"
echo "  [3] 🤖 Dobot Bringup (Nova5)..."
ros2 launch dobot_bringup_v3 nova5.launch.py > "$LOG_DOBOT" 2>&1 &
PID_DOBOT=$!
echo "        PID=$PID_DOBOT  Log: $LOG_DOBOT"
echo "$PID_DOBOT" >> "$PIDFILE"

# ── [4] Robot Logic + Motion Executor (params loaded inside launch file) ──
LOG_ROBOT="$LOG_DIR/robot_logic_node.log"
echo "  [4] 🧠 Robot Logic + Motion Executor..."
ros2 launch robot_control_main robot_logic.launch.py > "$LOG_ROBOT" 2>&1 &
PID_ROBOT=$!
LOG_MOTION="$LOG_ROBOT"   # same log file; kept for tail-f line below
echo "        PID=$PID_ROBOT  Log: $LOG_ROBOT"
echo "$PID_ROBOT" >> "$PIDFILE"

# ── [5] Gripper Node (Festo CPX, venv) ──
# ⚠️ DISABLED: gripper/picker đã tích hợp trong cartridge_providesystem (cùng CPX 172.16.11.37)
#    Chạy riêng gây xung đột channel (2 node ghi ngược coil trên cùng valve).
LOG_GRIPPER="$LOG_DIR/gripper_festo_node.log"
# "$WS/run_gripper_node.sh" > "$LOG_GRIPPER" 2>&1 &
# PID_GRIPPER=$!
# echo "        PID=$PID_GRIPPER  Log: $LOG_GRIPPER"
# echo "$PID_GRIPPER" >> "$PIDFILE"
echo "  [5] ⏭️  Gripper Node OFF (tích hợp trong Cartridge Node)"

# ── [6] Dual Jetson CSI Camera + TensorRT YOLO System ──
LOG_CAMERA="$LOG_DIR/dual_camera_system.log"
echo "  [6] 📷 Dual CSI + TensorRT YOLO (input/output models)..."

# One broken V4L2 stream can leave the paired V3Link receiver in an invalid
# state. The camera node reports the error and exits; this supervisor then
# stops both streams and restarts the pair with backoff. A stable five-minute
# run clears the failure counter.
camera_stack_supervisor() {
    local camera_launch_pid=""
    local consecutive_failures=0
    local run_started=0
    local run_seconds=0
    local retry_delay=3

    _stop_camera_child() {
        if [ -n "$camera_launch_pid" ] && kill -0 "$camera_launch_pid" 2>/dev/null; then
            kill -TERM "$camera_launch_pid" 2>/dev/null || true
            local shutdown_wait=0
            local child_state=""
            while kill -0 "$camera_launch_pid" 2>/dev/null; do
                child_state=$(ps -o stat= -p "$camera_launch_pid" 2>/dev/null || true)
                if [ -z "$child_state" ] || [[ "$child_state" == Z* ]]; then
                    break
                fi
                if [ "$shutdown_wait" -ge 6 ]; then
                    kill -KILL "$camera_launch_pid" 2>/dev/null || true
                    break
                fi
                sleep 1
                shutdown_wait=$((shutdown_wait + 1))
            done
            wait "$camera_launch_pid" 2>/dev/null || true
        fi
        stop_jetson_camera_nodes
        exit 0
    }
    trap _stop_camera_child TERM INT HUP

    while true; do
        stop_jetson_camera_nodes

        echo "STARTING failures=$consecutive_failures" > "$CAMERA_SUPERVISOR_STATUS"
        echo "[$(date --iso-8601=seconds)] Starting paired camera stack"
        run_started=$(date +%s)
        ros2 launch csi_camera dual_camera_system.launch.py \
            enable_inference:=true \
            cam0_model:="$CAM0_AI_MODEL" \
            cam1_model:="$CAM1_AI_MODEL" \
            capture_fps:="$CAMERA_CAPTURE_FPS" \
            max_inference_fps:="$AI_MAX_FPS" &
        camera_launch_pid=$!
        echo "RUNNING launch_pid=$camera_launch_pid failures=$consecutive_failures" \
            > "$CAMERA_SUPERVISOR_STATUS"

        wait "$camera_launch_pid"
        local launch_rc=$?
        camera_launch_pid=""
        run_seconds=$(( $(date +%s) - run_started ))

        if [ "$run_seconds" -ge 300 ]; then
            consecutive_failures=0
        else
            consecutive_failures=$((consecutive_failures + 1))
        fi

        case "$consecutive_failures" in
            0|1) retry_delay=3 ;;
            2) retry_delay=10 ;;
            3) retry_delay=30 ;;
            *) retry_delay=60 ;;
        esac

        echo "RECOVERING rc=$launch_rc runtime_s=$run_seconds failures=$consecutive_failures retry_s=$retry_delay" \
            > "$CAMERA_SUPERVISOR_STATUS"
        echo "[$(date --iso-8601=seconds)] Camera stack exited rc=$launch_rc after ${run_seconds}s; paired recovery in ${retry_delay}s"
        stop_jetson_camera_nodes
        sleep "$retry_delay"
    done
}

: > "$LOG_CAMERA"
camera_stack_supervisor >> "$LOG_CAMERA" 2>&1 &
PID_CAMERA=$!
echo "        Supervisor PID=$PID_CAMERA  Log: $LOG_CAMERA"
echo "$PID_CAMERA" >> "$PIDFILE"

# Brief settle window so the GUI subscribers see publishers ready on first
# discovery cycle (avoids "UNKNOWN" placeholders flickering at startup).
sleep 2

# ══════════════════════════════════════════
# WAVE 2 — GUIs (start in parallel after WAVE 1 settle).
# ══════════════════════════════════════════

# ── [7] Web GUI (cartridge_gui.py — port 8080) ──
LOG_WEB="$LOG_DIR/cartridge_web_gui.log"
WEB_GUI="$WS/src/system_feed_cartridge/scripts/cartridge_gui_web.py"
WEB_GUI_ENABLED=true
for arg in "$@"; do [ "$arg" = "--no-web" ] && WEB_GUI_ENABLED=false; done

if $WEB_GUI_ENABLED && [ -f "$WEB_GUI" ]; then
    echo "  [7] 🌐 Web GUI (port 8080)..."
    python3 "$WEB_GUI" > "$LOG_WEB" 2>&1 &
    PID_WEB_GUI=$!
    echo "        PID=$PID_WEB_GUI  Log: $LOG_WEB  Access: http://$(hostname -I | awk '{print $1}'):8080"
    echo "$PID_WEB_GUI" >> "$PIDFILE"
else
    echo "  [7] ⏭️  Web GUI skipped (--no-web)"
fi

# ── [8] QML GUI (native, HDMI) ──
LOG_QML="$LOG_DIR/unified_gui.log"
QML_BIN="$WS/install/unified_control_gui/lib/unified_control_gui/unified_control_gui"
GUI_RESTART_FLAG="/tmp/unified_gui_restart_requested"

start_qml_gui() {
  if [ -n "${DISPLAY:-}" ]; then
    echo "  [8] 🖥️  QML GUI (DISPLAY=$DISPLAY)..."
    "$QML_BIN" > "$LOG_QML" 2>&1 &
    PID_QML_GUI=$!
    echo "        PID=$PID_QML_GUI  Log: $LOG_QML"
    echo "$PID_QML_GUI" >> "$PIDFILE"
  else
    echo "  [8] ⚠️  DISPLAY not set — skipping QML GUI"
    PID_QML_GUI=""
  fi
}

rm -f "$GUI_RESTART_FLAG"
start_qml_gui

# ══════════════════════════════════════════
# [9] RS485 BUS NODE — RevPi A (Loadcell + VFD)
# ══════════════════════════════════════════
REVPI_HOST="${REVPI_HOST:-${REVPI_A_HOST:-172.16.11.31}}"
REVPI_USER="${REVPI_USER:-pi}"
REVPI_WS="${REVPI_WS:-/home/${REVPI_USER}/ros2_jazzy}"

LOG_REVPI="$LOG_DIR/revpi_a_nodes.log"

if ping -c 1 -W 1 "$REVPI_HOST" >/dev/null 2>&1; then
    echo "  [9] 📡 RevPi A ($REVPI_HOST) — đang start rs485_bus_node..."
    ssh -o BatchMode=yes -o ConnectTimeout=5 \
        -o StrictHostKeyChecking=accept-new \
        -o UserKnownHostsFile="$HOME/.ssh/known_hosts" \
        "${REVPI_USER}@${REVPI_HOST}" \
        "tmux kill-session -t rs485_bus 2>/dev/null || true; sleep 1; \
         tmux new-session -d -s rs485_bus 'exec bash /home/pi/start_rs485.sh > /tmp/rs485_bus_node.log 2>&1'" >> "$LOG_REVPI" 2>&1 \
    && echo "        ✅ rs485_bus_node started on RevPi A via start_rs485.sh" \
    || echo "        ⚠️  SSH failed — rs485_bus_node không start được. Xem: $LOG_REVPI"
else
    echo "  [9] ⏭️  RevPi A ($REVPI_HOST) không thấy trên LAN — bỏ qua rs485_bus_node"
    echo "        Khi RevPi A online: bash ~/deploy_revpi.sh  rồi restart start_all.sh"
fi

# ══════════════════════════════════════════
# [10] LOADCELL NODE — RevPi A (4-20mA)
# ══════════════════════════════════════════
LOADCELL_HOST="${LOADCELL_HOST:-${REVPI_A_HOST:-172.16.11.31}}"
LOADCELL_USER="${LOADCELL_USER:-pi}"

LOG_LOADCELL="$LOG_DIR/loadcell_node.log"

if ping -c 1 -W 1 "$LOADCELL_HOST" >/dev/null 2>&1; then
    echo "  [10] ⚖️  RevPi A ($LOADCELL_HOST) — đang start loadcell_node (4-20mA)..."
    ssh -o BatchMode=yes -o ConnectTimeout=5 \
        -o StrictHostKeyChecking=accept-new \
        -o UserKnownHostsFile="$HOME/.ssh/known_hosts" \
        "${LOADCELL_USER}@${LOADCELL_HOST}" \
        "tmux kill-session -t loadcell 2>/dev/null || true; sleep 1; \
         tmux new-session -d -s loadcell 'exec bash /home/pi/start_loadcell.sh > /tmp/loadcell_node.log 2>&1'" >> "$LOG_LOADCELL" 2>&1 \
    && echo "        ✅ loadcell_node started on RevPi A via start_loadcell.sh" \
    || echo "        ⚠️  SSH failed — loadcell_node không start được. Xem: $LOG_LOADCELL"
else
    echo "  [10] ⏭️  RevPi A ($LOADCELL_HOST) không thấy trên LAN — bỏ qua loadcell_node"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ All processes started! (10 components)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📋 Logs:"
echo "  tail -f $LOG_PROVIDE        # Cartridge feeder"
echo "  tail -f $LOG_DOBOT          # Dobot driver"
echo "  tail -f $LOG_ROBOT          # Robot logic"
echo "  tail -f $LOG_GRIPPER        # Gripper"
echo "  tail -f $LOG_CAMERA         # 2x CSI camera + TensorRT YOLO"
[ -n "${PID_QML_GUI:-}" ] && echo "  tail -f $LOG_QML             # QML GUI"
echo "  ssh pi@${REVPI_A_HOST} cat /tmp/loadcell_node.log  # Loadcell"
echo ""
echo "🌐 Web GUI: bash start_all.sh --web"
echo ""
echo "Press Ctrl+C to stop all"
echo ""

# Monitor — GUI exit thường → dừng toàn bộ hệ thống.
# GUI exit code 42 hoặc restart flag → chỉ restart lại QML GUI, giữ node khác.
# Bỏ auto-restart crash: user yêu cầu khi tắt file không retry/reconnect lại
# GUI; tránh "zombie restart" che lỗi cứng (CUDA OOM, TensorRT, segfault...).
while true; do
    if [ -n "${PID_QML_GUI:-}" ]; then
        wait "$PID_QML_GUI" 2>/dev/null
        GUI_EXIT=$?
        if [ "$GUI_EXIT" -eq 42 ] || [ -f "$GUI_RESTART_FLAG" ]; then
            echo "[GUI] 🔄 Restart requested (code=$GUI_EXIT)"
            rm -f "$GUI_RESTART_FLAG"
            start_qml_gui
            continue
        fi
        echo "[GUI] 🔴 Exited (code=$GUI_EXIT) — dừng hệ thống"
        break
    else
        sleep 3
    fi
done
