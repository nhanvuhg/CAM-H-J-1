#!/usr/bin/env bash
set -euo pipefail

# Desktop launcher for the direct V4L2 YOLO dataset capture tool.
# The tool owns both cameras itself; do not run Camera x2 at the same time.
APP="/home/nhan/capture_yolo_gui.py"
LOCKFILE="/tmp/jetson_yolo_v4l2_capture.lock"

export DISPLAY="${DISPLAY:-:0}"
export PYTHONUNBUFFERED=1
export QT_AUTO_SCREEN_SCALE_FACTOR="0"
export QT_SCALE_FACTOR="1"

exec 8>"$LOCKFILE"
if ! flock -n 8; then
    echo "Tool chụp YOLO đang chạy." >&2
    exit 1
fi

if [ ! -f "$APP" ]; then
    echo "Không tìm thấy: $APP" >&2
    exit 1
fi

exec python3 "$APP" "$@"
