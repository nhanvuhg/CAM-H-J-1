#!/usr/bin/env bash
set -euo pipefail

WS="${ROS2_WS:-$HOME/ros2_ws}"
LOG_DIR="$WS/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/restart_system_nodes.log"
LOCK_FILE="/tmp/unified_restart_system_nodes.lock"
CAMERA_SUPERVISOR_PIDFILE="/tmp/camera_stack_supervisor.pid"

exec >>"$LOG_FILE" 2>&1
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "Restart already in progress; ignoring duplicate request"
  exit 0
fi

echo ""
echo "============================================================"
echo "Restart system nodes: $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================================"

export DISPLAY="${DISPLAY:-:0}"

if [ -f "$WS/ros2_env.sh" ]; then
  # shellcheck disable=SC1090
  source "$WS/ros2_env.sh" || true
fi

if [ -f "/opt/ros/humble/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash || true
  set -u
fi

if [ -f "$WS/install/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1090
  source "$WS/install/setup.bash" || true
  set -u
fi

stop_pattern() {
  local pattern="$1"
  pkill -TERM -f "$pattern" 2>/dev/null || true
}

kill_pattern() {
  local pattern="$1"
  pkill -KILL -f "$pattern" 2>/dev/null || true
}

camera_is_managed=false
if [ -r "$CAMERA_SUPERVISOR_PIDFILE" ]; then
  CAMERA_SUPERVISOR_PID=$(cat "$CAMERA_SUPERVISOR_PIDFILE" 2>/dev/null || true)
  if [[ "$CAMERA_SUPERVISOR_PID" =~ ^[0-9]+$ ]] \
      && kill -0 "$CAMERA_SUPERVISOR_PID" 2>/dev/null; then
    camera_is_managed=true
  fi
fi

echo "[1/4] Stop local runtime nodes (GUI stays alive)"
stop_pattern "cartridge_providesystem_py"
stop_pattern "vfd_logic_node.py"
stop_pattern "ros2 launch dobot_bringup_v3 nova5.launch.py"
stop_pattern "dobot_bringup"
stop_pattern "ros2 launch robot_control_main robot_logic.launch.py"
stop_pattern "ros2 launch csi_camera dual_camera_system.launch.py"
stop_pattern "robot_logic_node"
stop_pattern "motion_executor"
sleep 2
kill_pattern "cartridge_providesystem_py"
kill_pattern "vfd_logic_node.py"
kill_pattern "ros2 launch dobot_bringup_v3 nova5.launch.py"
kill_pattern "dobot_bringup"
kill_pattern "ros2 launch robot_control_main robot_logic.launch.py"
if ! $camera_is_managed; then
  kill_pattern "ros2 launch csi_camera dual_camera_system.launch.py"
fi

# Camera nodes own V4L2/CUDA buffers. When start_all.sh is active, its camera
# supervisor observes the launch process exit, performs the validated graceful
# STREAMOFF cleanup, and starts one replacement stack. Never launch a second
# camera stack here: two owners racing for /dev/video0,/dev/video1 can wedge the
# Tegra VI channel until reboot.
if $camera_is_managed; then
  echo "camera stack restart delegated to supervisor PID=$CAMERA_SUPERVISOR_PID"
fi

echo "[2/4] Start local runtime nodes"
CARTRIDGE_BIN="$WS/install/system_feed_cartridge/lib/system_feed_cartridge/cartridge_providesystem_py"
VFD_LOGIC_PY="$WS/install/unified_control_gui/lib/unified_control_gui/vfd_logic_node.py"

if [ -x "$CARTRIDGE_BIN" ]; then
  "$CARTRIDGE_BIN" > "$LOG_DIR/cartridge_node.log" 2>&1 &
  new_pid=$!
  echo "cartridge_providesystem_py PID=$new_pid"
else
  echo "WARN: missing cartridge binary: $CARTRIDGE_BIN"
fi

if [ -f "$VFD_LOGIC_PY" ]; then
  python3 "$VFD_LOGIC_PY" > "$LOG_DIR/vfd_logic_node.log" 2>&1 &
  new_pid=$!
  echo "vfd_logic_node PID=$new_pid"
else
  echo "WARN: missing vfd logic: $VFD_LOGIC_PY"
fi

ros2 launch dobot_bringup_v3 nova5.launch.py > "$LOG_DIR/dobot_bringup.log" 2>&1 &
new_pid=$!
echo "dobot_bringup PID=$new_pid"

ros2 launch robot_control_main robot_logic.launch.py > "$LOG_DIR/robot_logic_node.log" 2>&1 &
new_pid=$!
echo "robot_logic.launch PID=$new_pid"

if ! $camera_is_managed; then
  ros2 launch csi_camera dual_camera_system.launch.py > "$LOG_DIR/dual_camera_system.log" 2>&1 &
  new_pid=$!
  echo "dual_camera_system PID=$new_pid (standalone fallback)"
fi

echo "[3/4] Restart RevPi nodes"
REVPI_HOST="${REVPI_HOST:-${REVPI_A_HOST:-172.16.11.31}}"
REVPI_USER="${REVPI_USER:-pi}"

if ping -c 1 -W 1 "$REVPI_HOST" >/dev/null 2>&1; then
  ssh -o BatchMode=yes -o ConnectTimeout=5 "${REVPI_USER}@${REVPI_HOST}" \
    "tmux kill-session -t rs485_bus 2>/dev/null || true; sleep 1; tmux new-session -d -s rs485_bus 'exec bash /home/pi/start_rs485.sh > /tmp/rs485_bus_node.log 2>&1'" \
    && echo "rs485_bus restarted on RevPi ${REVPI_HOST}" \
    || echo "WARN: rs485 restart failed on RevPi ${REVPI_HOST}"

  ssh -o BatchMode=yes -o ConnectTimeout=5 "${REVPI_USER}@${REVPI_HOST}" \
    "tmux kill-session -t loadcell 2>/dev/null || true; sleep 1; tmux new-session -d -s loadcell 'exec bash /home/pi/start_loadcell.sh > /tmp/loadcell_node.log 2>&1'" \
    && echo "loadcell restarted on RevPi ${REVPI_HOST}" \
    || echo "WARN: loadcell restart failed on RevPi ${REVPI_HOST}"
else
  echo "WARN: RevPi ${REVPI_HOST} not reachable"
fi

echo "[4/4] Done"
