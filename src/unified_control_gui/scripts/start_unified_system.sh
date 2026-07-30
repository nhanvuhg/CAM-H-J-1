#!/usr/bin/env bash
set -euo pipefail

# Start Unified System: Cartridge Node + Unified Control GUI
# Usage: bash start_unified_system.sh
# Stop:  Ctrl+C

WS="$HOME/ros2_ws"

# Show GUI on Pi's HDMI screen (even when SSH)
export DISPLAY=${DISPLAY:-:0}

# Source ROS 2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  set +u
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/fastdds_no_shm.xml"
  export ROS_DOMAIN_ID=22
  source /opt/ros/jazzy/setup.bash || true
  set -u
fi
if [ -f "$WS/install/setup.bash" ]; then
  set +u
  source "$WS/install/setup.bash" || true
  set -u
fi
if [ -f "$WS/install/unified_control_gui/share/unified_control_gui/local_setup.bash" ]; then
  set +u
  source "$WS/install/unified_control_gui/share/unified_control_gui/local_setup.bash" || true
  set -u
fi

LOG_DIR="$WS/logs"
mkdir -p "$LOG_DIR"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🚀 Unified System Launcher"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Check for duplicates
MYPID=$$
EXISTING=$(ps aux | grep -E "(cartridge_providesystem_py|[u]nified_control_gui)" | grep -v -E "(grep|tail|start_unified|bash)" | wc -l || true)
if [ "${EXISTING:-0}" -gt 0 ]; then
    echo "⚠️  Found $EXISTING existing process(es)!"
    ps aux | grep -E "(cartridge_providesystem_py|[u]nified_control_gui)" | grep -v -E "(grep|tail|start_unified|bash)" || true
    echo ""
    echo "Killing old processes..."
    pkill -9 -f "cartridge_providesystem_py_node" 2>/dev/null || true
    pkill -9 -f "cartridge_gui.py" 2>/dev/null || true
    # Kill the GUI binary but not this script
    pgrep -f "lib/unified_control_gui/unified_control" | while read p; do kill -9 "$p" 2>/dev/null || true; done
    fuser -k 8080/tcp 2>/dev/null || true
    sleep 1
    echo "✅ Old processes killed."
fi
echo ""

NODE_LOG="$LOG_DIR/cartridge_node.log"
GUI_LOG="$LOG_DIR/unified_gui.log"

PID_NODE=""
PID_GUI=""
GUI_RESTART_FLAG="/tmp/unified_gui_restart_requested"

cleanup() {
  echo ""
  echo "Cleaning up..."
  for pid in "$PID_GUI" "$PID_NODE"; do
    if [ -n "$pid" ] && ps -p "$pid" > /dev/null 2>&1; then
      echo "  Killing PID $pid"
      kill "$pid" || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  pgrep -f "lib/unified_control_gui/unified_control" | while read p; do kill -9 "$p" 2>/dev/null || true; done
  pkill -9 -f "cartridge_providesystem_py_node" 2>/dev/null || true
  echo "  ✅ All processes stopped."
}
trap cleanup EXIT INT TERM

# [1/2] Cartridge System Node
echo "  [1/2] Starting Cartridge System Node... Log: $NODE_LOG"
ros2 run system_feed_cartridge cartridge_providesystem_py > "$NODE_LOG" 2>&1 &
PID_NODE=$!
echo "        PID=$PID_NODE"
sleep 3

start_qml_gui() {
  if [ -n "${DISPLAY:-}" ]; then
    echo "  [2/2] Starting Unified Control GUI (DISPLAY=$DISPLAY)... Log: $GUI_LOG"
    UNIFIED_GUI_MANAGED_RESTART=1 ros2 run unified_control_gui unified_control_gui > "$GUI_LOG" 2>&1 &
    PID_GUI=$!
    echo "        PID=$PID_GUI"
  else
    echo "  ⚠️  DISPLAY not set - skipping GUI"
    PID_GUI=""
  fi
}

# [2/2] Unified Control GUI
rm -f "$GUI_RESTART_FLAG"
start_qml_gui

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ All processes started!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Tail logs with:"
echo "  tail -f $NODE_LOG"
echo "  tail -f $GUI_LOG"
echo ""

# Wait indefinitely; exit code 42 means GUI requested a self-restart.
while true; do
  if [ -n "${PID_GUI:-}" ]; then
    wait "$PID_GUI" 2>/dev/null
    GUI_EXIT=$?
    if [ "$GUI_EXIT" -eq 42 ] || [ -f "$GUI_RESTART_FLAG" ]; then
      echo "  🔄 GUI restart requested (code=$GUI_EXIT)"
      rm -f "$GUI_RESTART_FLAG"
      start_qml_gui
      continue
    fi
    echo "  GUI exited (code=$GUI_EXIT). Launcher stays alive."
    PID_GUI=""
  fi
  sleep 3
done
