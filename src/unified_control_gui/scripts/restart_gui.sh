#!/usr/bin/env bash
set -euo pipefail

WS="${ROS2_WS:-$HOME/ros2_ws}"
LOG_DIR="$WS/logs"
mkdir -p "$LOG_DIR"

OLD_PID="${1:-}"
GUI_BIN="$WS/install/unified_control_gui/lib/unified_control_gui/unified_control_gui"
GUI_LOG="$LOG_DIR/unified_gui.log"
RESTART_LOG="$LOG_DIR/restart_gui.log"

exec >>"$RESTART_LOG" 2>&1

echo ""
echo "============================================================"
echo "Restart GUI: $(date '+%Y-%m-%d %H:%M:%S') old_pid=${OLD_PID:-none}"
echo "============================================================"

export DISPLAY="${DISPLAY:-:0}"
export FILL_HP_USERS_FILE="${FILL_HP_USERS_FILE:-$WS/src/unified_control_gui/fill_hp_users.json}"

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

if [ ! -x "$GUI_BIN" ]; then
  echo "ERROR: GUI binary not found: $GUI_BIN"
  exit 1
fi

echo "Starting GUI: $GUI_BIN"
setsid "$GUI_BIN" > "$GUI_LOG" 2>&1 < /dev/null &
NEW_PID=$!
disown "$NEW_PID" 2>/dev/null || true
echo "New GUI PID=$NEW_PID"

sleep 2

if ! ps -p "$NEW_PID" >/dev/null 2>&1; then
  echo "ERROR: New GUI exited immediately; keep old GUI alive"
  exit 1
fi

if [ -n "$OLD_PID" ] && ps -p "$OLD_PID" >/dev/null 2>&1; then
  echo "New GUI is alive, terminating old GUI PID=$OLD_PID"
  kill -TERM "$OLD_PID" 2>/dev/null || true
  sleep 1
fi

if [ -n "$OLD_PID" ] && ps -p "$OLD_PID" >/dev/null 2>&1; then
  echo "Old GUI still alive, killing PID=$OLD_PID"
  kill -KILL "$OLD_PID" 2>/dev/null || true
fi
