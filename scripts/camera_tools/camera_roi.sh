#!/bin/bash
################################################################################
# camera_roi.sh — Mở camera_roi.py (chụp PNG 1280x720 cho ROI calibration)
#
# GUI: /home/pi/camera_roi.py   (PyQt5)
# Lưu ảnh: /home/pi/Pictures/roi/cam_roi_<timestamp>.png
################################################################################

set -uo pipefail

GUI_PY=/home/pi/camera_roi.py

echo "=========================================="
echo "📷 Camera ROI Capture (1280x720 production)"
echo "=========================================="

export DISPLAY="${DISPLAY:-:0}"
[ -z "${XAUTHORITY:-}" ] && [ -f "$HOME/.Xauthority" ] && export XAUTHORITY="$HOME/.Xauthority"

# Dọn process camera cũ tránh xung đột libcamera
echo "🔍 Dọn process camera cũ..."
pkill -9 -f "camera_capture_gui.py" 2>/dev/null || true
pkill -9 -f "camera_roi.py"         2>/dev/null || true
pkill -9 -f "rpicam-vid"            2>/dev/null || true
sleep 1

mkdir -p "$HOME/Pictures/roi"

echo ""
echo "🔎 Camera libcamera thấy được:"
rpicam-hello --list-cameras 2>&1 | grep -E "^[0-9] :" || {
    echo "   ❌ Không thấy camera!"
    exit 1
}

echo ""
echo "🚀 Mở GUI: $GUI_PY"
echo "   Lưu ảnh vào: $HOME/Pictures/roi/"
echo ""

exec python3 "$GUI_PY"
