#!/usr/bin/env python3
"""
camera_roi.py

GUI PyQt5 chụp ảnh CSI camera Ở TỈ LỆ PRODUCTION 1280×720 — dùng riêng cho
calibrate ROI (vision_decision_node). KHÔNG phải để train (training capture
vẫn dùng camera_capture_gui.py @ 1280×960).

- Stream camera qua rpicam-vid với --width 1280 --height 720 (match production)
- Lưu ảnh PNG vào ~/Pictures/roi/cam_roi_<timestamp>.png
- Ảnh xuất ra dùng được trực tiếp với roi_picker.py (toạ độ 1:1 production)

Chạy độc lập:
    python3 /home/pi/camera_roi.py
"""

import os
import sys
import time
import signal
import subprocess
from datetime import datetime

import cv2
import numpy as np
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap, QKeySequence
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QLineEdit,
    QVBoxLayout, QHBoxLayout, QShortcut, QMessageBox,
)

# ─── Config ───────────────────────────────────────────────────────────────
# Production camera_node publish 1280×720 (16:9). Capture cùng tỉ lệ để toạ độ
# pick được dùng trực tiếp với Detection2D bbox (vision_decision_node).
WIDTH        = 1280
HEIGHT       = 720
FPS          = 10
CAMERA_INDEX = 0
SAVE_DIR     = os.path.expanduser('~/Pictures/roi')
PREVIEW_MAX_W = 960     # Preview QLabel max width (giữ GUI gọn)
# ──────────────────────────────────────────────────────────────────────────


class FrameGrabber(QThread):
    """Spawn rpicam-vid, read YUV420 frames from stdout, emit BGR ndarray."""
    frame_ready = pyqtSignal(np.ndarray)
    error       = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running     = True
        self._frame_bytes = (WIDTH * HEIGHT * 3) // 2
        self.proc         = None

    def run(self):
        cmd = [
            'rpicam-vid',
            '--camera',    str(CAMERA_INDEX),
            '-t',          '0',
            '--nopreview',
            '--codec',     'yuv420',
            '--width',     str(WIDTH),
            '--height',    str(HEIGHT),
            '--framerate', str(FPS),
            # CRITICAL cho GMSL2: dùng full sensor mode 4056x3040.
            # Mode 2028:1520 KHÔNG qua được GMSL2 deserializer → V4L2 dequeue timeout.
            '--mode',      '4056:3040:12:P',
            '--denoise',   'cdn_off',
            '--flush',
            '-o',          '-',
        ]
        log_path = '/tmp/rpicam_gui.log'
        try:
            log_fd = open(log_path, 'wb')
            self.proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=log_fd, bufsize=0,
            )
        except FileNotFoundError:
            self.error.emit('Không tìm thấy rpicam-vid trong PATH')
            return

        while self._running:
            buf = b''
            while len(buf) < self._frame_bytes and self._running:
                if self.proc.poll() is not None:
                    self.error.emit(
                        f'rpicam-vid đã thoát (rc={self.proc.returncode}). '
                        f'Xem {log_path}'
                    )
                    return
                chunk = self.proc.stdout.read(self._frame_bytes - len(buf))
                if not chunk:
                    break
                buf += chunk

            if len(buf) != self._frame_bytes:
                continue

            yuv = np.frombuffer(buf, dtype=np.uint8).reshape(
                (HEIGHT * 3 // 2, WIDTH)
            )
            bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
            self.frame_ready.emit(bgr)

    def stop(self):
        self._running = False
        if self.proc and self.proc.poll() is None:
            try:
                self.proc.send_signal(signal.SIGINT)
                self.proc.wait(timeout=2)
            except Exception:
                try: self.proc.kill()
                except Exception: pass
        self.wait(2000)


class CameraGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f'CSI Camera ROI — {WIDTH}x{HEIGHT} (production scale)')
        self.resize(PREVIEW_MAX_W + 40, int(PREVIEW_MAX_W * HEIGHT / WIDTH) + 200)

        os.makedirs(SAVE_DIR, exist_ok=True)

        self.current_bgr = None
        self.shot_count  = 0

        # ── UI ────────────────────────────────────────
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        self.view = QLabel('Đang khởi động camera...')
        self.view.setAlignment(Qt.AlignCenter)
        self.view.setMinimumSize(PREVIEW_MAX_W, int(PREVIEW_MAX_W * HEIGHT / WIDTH))
        self.view.setStyleSheet('background: #111; color: #999;')
        self.view.setScaledContents(False)  # ta tự scale giữ tỉ lệ
        root.addWidget(self.view, 1)

        # Ô tên file
        name_row = QHBoxLayout()
        lbl = QLabel('Tên file:')
        lbl.setStyleSheet('font-size: 11pt;')
        name_row.addWidget(lbl)
        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText('Để trống → cam_<timestamp>.png   |   Có nhập → <tên>_<timestamp>.png')
        self.name_edit.setStyleSheet('font-size: 12pt; padding: 6px;')
        name_row.addWidget(self.name_edit, 1)
        root.addLayout(name_row)

        # Nút bấm
        btn_row = QHBoxLayout()
        self.btn_shot = QPushButton('📸 Chụp ảnh  (C / Space)')
        self.btn_shot.setMinimumHeight(48)
        self.btn_shot.setStyleSheet(
            'font-size: 16pt; font-weight: bold; background: #2c7; color: white;'
        )
        self.btn_shot.clicked.connect(self.capture)
        btn_row.addWidget(self.btn_shot, 3)

        self.btn_quit = QPushButton('Thoát')
        self.btn_quit.setMinimumHeight(48)
        self.btn_quit.clicked.connect(self.close)
        btn_row.addWidget(self.btn_quit, 1)
        root.addLayout(btn_row)

        self.status = QLabel(f'📁 Lưu vào: {SAVE_DIR}  |  Ảnh PNG: {WIDTH}x{HEIGHT}')
        self.status.setStyleSheet('padding: 4px; color: #555;')
        root.addWidget(self.status)

        QShortcut(QKeySequence('C'), self, activated=self.capture)
        QShortcut(QKeySequence('Space'), self, activated=self.capture)

        # ── Grabber ───────────────────────────────────
        self.grabber = FrameGrabber()
        self.grabber.frame_ready.connect(self.on_frame)
        self.grabber.error.connect(self.on_error)
        self.grabber.start()

    def on_frame(self, bgr: np.ndarray):
        self.current_bgr = bgr  # giữ frame gốc full resolution để lưu
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        img = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        pm = QPixmap.fromImage(img).scaledToWidth(
            PREVIEW_MAX_W, Qt.SmoothTransformation
        )
        self.view.setPixmap(pm)

    def on_error(self, msg: str):
        self.view.setText(f'❌ {msg}')
        QMessageBox.critical(self, 'Camera error', msg)

    def capture(self):
        if self.current_bgr is None:
            self.status.setText('⚠️  Chưa có frame nào — chờ camera lên...')
            return
        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        # Sanitize tên người dùng nhập: chỉ giữ [A-Za-z0-9_-], thay khoảng trắng = '_'
        raw = self.name_edit.text().strip().replace(' ', '_')
        safe = ''.join(c for c in raw if c.isalnum() or c in ('_', '-'))
        prefix = safe if safe else 'cam_roi'
        path = os.path.join(SAVE_DIR, f'{prefix}_{ts}.png')
        ok = cv2.imwrite(path, self.current_bgr)
        self.shot_count += 1
        if ok:
            self.status.setText(f'✅ Đã lưu #{self.shot_count}: {path}')
        else:
            self.status.setText(f'❌ Lỗi lưu file: {path}')

    def closeEvent(self, event):
        self.grabber.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    win = CameraGUI()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
