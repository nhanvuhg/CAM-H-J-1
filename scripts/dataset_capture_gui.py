#!/usr/bin/env python3
"""Capture exact Jetson ROS input frames for the two production YOLO models.

The application subscribes to the exact AI input topics:
  cam0 /cam0HP/image_raw -> input model  (tray, cartridge)
  cam1 /cam1HP/image_raw -> output model (tray, cartridge, cartridgefall)

Images are saved without overlays, stretching, cropping, or color conversion.
The stable V4L2 RG10 workers publish BGR8 640x360 from the same tuned frames
used at inference. Ultralytics must letterbox these 16:9 images to imgsz=640.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import os
from pathlib import Path
import re
import shutil
import signal
import sys
import time
from collections import deque
from datetime import datetime
from typing import Dict, Iterable, Optional, Tuple

import cv2
import numpy as np
from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QKeySequence, QPalette, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QShortcut,
    QVBoxLayout,
    QWidget,
)

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


EXPECTED_WIDTH = 640
EXPECTED_HEIGHT = 360
EXPECTED_ENCODING = "bgr8"
TOPICS = {
    0: "/cam0HP/image_raw",
    1: "/cam1HP/image_raw",
}
HEALTH_TOPICS = {
    0: "/camera/cam0/health",
    1: "/camera/cam1/health",
}
ROLES = {
    0: "input",
    1: "output",
}
CLASS_NAMES = {
    0: ("tray", "cartridge"),
    1: ("tray", "cartridge", "cartridgefall"),
}
ENGINE_PATHS = {
    0: Path.home() / "models" / "data_input_hp_final_2_fp16.engine",
    1: Path.home() / "models" / "data_output_hp_final_fp16.engine",
}
DEFAULT_BASE_DIR = Path.home() / "Datasets" / "Jetson_YOLO_Data"
MAX_SAVE_FRAME_AGE_SECONDS = 1.0
CRASH_FRAME_AGE_SECONDS = 2.5
MIN_FREE_BYTES = 5 * 1024**3


def safe_name(text: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_-]+", "_", text.strip()).strip("_")
    return cleaned[:64]


class RosCameraWorker(QThread):
    frame_ready = pyqtSignal(int, object, object)
    health_ready = pyqtSignal(int, str)
    worker_error = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = True
        self._last_preview_emit = {0: 0.0, 1: 0.0}
        self._arrival_times = {0: deque(maxlen=60), 1: deque(maxlen=60)}

    def run(self) -> None:
        node = None
        executor = None
        try:
            rclpy.init(args=None)
            node = Node("jetson_yolo_dataset_capture_gui")
            image_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            health_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            for camera_id in (0, 1):
                node.create_subscription(
                    Image,
                    TOPICS[camera_id],
                    lambda message, cid=camera_id: self._on_image(cid, message),
                    image_qos,
                )
                node.create_subscription(
                    String,
                    HEALTH_TOPICS[camera_id],
                    lambda message, cid=camera_id: self.health_ready.emit(
                        cid, message.data
                    ),
                    health_qos,
                )

            executor = SingleThreadedExecutor()
            executor.add_node(node)
            while self._running and rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
        except Exception as exc:
            self.worker_error.emit(f"ROS camera subscriber error: {exc}")
        finally:
            if executor is not None:
                executor.shutdown(timeout_sec=1.0)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def _on_image(self, camera_id: int, message: Image) -> None:
        received = time.monotonic()
        arrivals = self._arrival_times[camera_id]
        arrivals.append(received)

        # Preview/save state only needs 15 updates/s; DDS is still drained at
        # the full camera rate so this subscriber can never back-pressure it.
        if received - self._last_preview_emit[camera_id] < (1.0 / 15.0):
            return
        self._last_preview_emit[camera_id] = received

        try:
            if message.encoding != EXPECTED_ENCODING:
                raise ValueError(
                    f"{TOPICS[camera_id]} encoding={message.encoding}, "
                    f"expected {EXPECTED_ENCODING}"
                )
            row_bytes = int(message.step)
            required_bytes = row_bytes * int(message.height)
            raw = np.frombuffer(message.data, dtype=np.uint8, count=required_bytes)
            rows = raw.reshape(int(message.height), row_bytes)
            image = rows[:, : int(message.width) * 3].reshape(
                int(message.height), int(message.width), 3
            ).copy()
        except Exception as exc:
            self.worker_error.emit(f"CAM{camera_id} invalid Image message: {exc}")
            return

        measured_fps = 0.0
        if len(arrivals) >= 2:
            elapsed = arrivals[-1] - arrivals[0]
            if elapsed > 0.0:
                measured_fps = (len(arrivals) - 1) / elapsed
        metadata = {
            "received_monotonic": received,
            "ros_stamp_ns": int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec),
            "frame_id": message.header.frame_id,
            "encoding": message.encoding,
            "width": int(message.width),
            "height": int(message.height),
            "measured_fps": measured_fps,
        }
        self.frame_ready.emit(camera_id, image, metadata)

    def stop(self) -> None:
        self._running = False
        self.wait(3000)


class DatasetCaptureWindow(QMainWindow):
    def __init__(self, jpeg_quality: Optional[int] = None):
        super().__init__()
        self.setWindowTitle("Jetson YOLO Dataset Capture — CAM0 Input / CAM1 Output")
        self.resize(1500, 900)

        # Dinh dang co dinh theo phien. Tron PNG voi JPG trong cung mot thu muc
        # anh la nguon sai lech am tham khi chia train/val, nen chon mot lan
        # luc khoi dong chu khong phai checkbox bat tat giua chung.
        if jpeg_quality is None:
            self.image_format = "PNG"
            self.image_suffix = ".png"
            self.imwrite_params = [cv2.IMWRITE_PNG_COMPRESSION, 3]
            self.format_note = "PNG lossless"
        else:
            self.image_format = "JPG"
            self.image_suffix = ".jpg"
            self.imwrite_params = [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
            self.format_note = f"JPG q{jpeg_quality}"

        self.base_dir = DEFAULT_BASE_DIR
        self.session_dir: Optional[Path] = None
        self.frames: Dict[int, Optional[np.ndarray]] = {0: None, 1: None}
        self.frame_metadata: Dict[int, dict] = {0: {}, 1: {}}
        self.health_text = {0: "WAITING", 1: "WAITING"}
        self.camera_state = {0: "WAITING", 1: "WAITING"}
        self.capture_counts = {0: 0, 1: 0}
        self.last_auto_gray: Dict[int, Optional[np.ndarray]] = {0: None, 1: None}
        self.pending_events = []

        self._build_ui()

        self.ros_worker = RosCameraWorker(self)
        self.ros_worker.frame_ready.connect(self.on_frame)
        self.ros_worker.health_ready.connect(self.on_health)
        self.ros_worker.worker_error.connect(self.on_worker_error)
        self.ros_worker.start()

        self.health_timer = QTimer(self)
        self.health_timer.timeout.connect(self.refresh_health)
        self.health_timer.start(500)

        self.auto_timer = QTimer(self)
        self.auto_timer.timeout.connect(lambda: self.capture_selected((0, 1), True))

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        title = QLabel("THU DỮ LIỆU YOLO — ẢNH RAW AI INPUT CỦA JETSON")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Sans", 18, QFont.Bold))
        root.addWidget(title)

        spec = QLabel(
            "V3Link V4L2 RG10 • ROS AI input BGR8 640×360 (16:9) • "
            f"{self.format_note} • không overlay/crop/stretch • engine 640×640 • "
            "YOLO tự letterbox pad 140 px trên/dưới"
        )
        spec.setAlignment(Qt.AlignCenter)
        spec.setStyleSheet(
            "padding: 8px; color: #b8e5ff; background: #163047; "
            "border-radius: 5px;"
        )
        root.addWidget(spec)

        camera_row = QHBoxLayout()
        self.preview_labels = {}
        self.camera_status_labels = {}
        for camera_id in (0, 1):
            role = ROLES[camera_id]
            classes = ", ".join(CLASS_NAMES[camera_id])
            group = QGroupBox(
                f"CAM{camera_id} — {role.upper()} MODEL — classes: {classes}"
            )
            layout = QVBoxLayout(group)
            preview = QLabel(f"Đang chờ {TOPICS[camera_id]} ...")
            preview.setAlignment(Qt.AlignCenter)
            preview.setMinimumSize(600, 338)
            preview.setStyleSheet("background: #111; color: #aaa;")
            layout.addWidget(preview, 1)
            status = QLabel("WAITING — chưa có frame")
            status.setStyleSheet("padding: 7px; background: #503d16; color: white;")
            layout.addWidget(status)
            self.preview_labels[camera_id] = preview
            self.camera_status_labels[camera_id] = status
            camera_row.addWidget(group, 1)
        root.addLayout(camera_row, 1)

        storage_group = QGroupBox("Thư mục và phiên chụp")
        storage_layout = QGridLayout(storage_group)
        self.folder_edit = QLineEdit(str(self.base_dir))
        self.folder_edit.setReadOnly(True)
        choose_button = QPushButton("📁 Chọn thư mục lưu")
        choose_button.clicked.connect(self.select_folder)
        storage_layout.addWidget(QLabel("Thư mục gốc:"), 0, 0)
        storage_layout.addWidget(self.folder_edit, 0, 1)
        storage_layout.addWidget(choose_button, 0, 2)

        self.session_name_edit = QLineEdit()
        self.session_name_edit.setPlaceholderText(
            "Ví dụ: ca_sang_den_trang (để trống vẫn tự tạo timestamp)"
        )
        new_session_button = QPushButton("Tạo phiên mới")
        new_session_button.clicked.connect(self.create_new_session)
        storage_layout.addWidget(QLabel("Tên phiên:"), 1, 0)
        storage_layout.addWidget(self.session_name_edit, 1, 1)
        storage_layout.addWidget(new_session_button, 1, 2)
        self.session_label = QLabel("Phiên hiện tại: chưa tạo")
        self.session_label.setWordWrap(True)
        storage_layout.addWidget(self.session_label, 2, 0, 1, 3)
        root.addWidget(storage_group)

        button_row = QHBoxLayout()
        self.capture_input_button = QPushButton("📸 CHỤP INPUT — CAM0  [0]")
        self.capture_output_button = QPushButton("📸 CHỤP OUTPUT — CAM1  [1]")
        self.capture_both_button = QPushButton("📸 CHỤP CẢ HAI  [SPACE]")
        for button in (
            self.capture_input_button,
            self.capture_output_button,
            self.capture_both_button,
        ):
            button.setMinimumHeight(58)
            button.setFont(QFont("Sans", 12, QFont.Bold))
            button_row.addWidget(button)
        self.capture_input_button.clicked.connect(
            lambda: self.capture_selected((0,), False)
        )
        self.capture_output_button.clicked.connect(
            lambda: self.capture_selected((1,), False)
        )
        self.capture_both_button.clicked.connect(
            lambda: self.capture_selected((0, 1), False)
        )
        root.addLayout(button_row)

        auto_row = QHBoxLayout()
        self.auto_checkbox = QCheckBox("Tự động chụp cả hai camera")
        self.auto_checkbox.toggled.connect(self.toggle_auto_capture)
        self.interval_spin = QDoubleSpinBox()
        self.interval_spin.setRange(0.5, 60.0)
        self.interval_spin.setSingleStep(0.5)
        self.interval_spin.setValue(1.0)
        self.interval_spin.setSuffix(" giây")
        auto_row.addWidget(self.auto_checkbox)
        auto_row.addWidget(QLabel("Chu kỳ:"))
        auto_row.addWidget(self.interval_spin)
        auto_row.addWidget(
            QLabel(
                "Auto bỏ frame gần trùng; manual luôn lưu. "
                "Khuyến nghị 1–2 giây/ảnh."
            ),
            1,
        )
        root.addLayout(auto_row)

        self.capture_summary = QLabel("Đã lưu: INPUT 0 ảnh | OUTPUT 0 ảnh")
        self.capture_summary.setStyleSheet("font-weight: bold; padding: 5px;")
        root.addWidget(self.capture_summary)

        self.crash_status = QLabel("V4L2/CSI: đang theo dõi frame và reconnect...")
        self.crash_status.setStyleSheet(
            "padding: 7px; background: #274a35; color: white;"
        )
        root.addWidget(self.crash_status)

        self.event_log = QPlainTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setMaximumBlockCount(200)
        self.event_log.setMaximumHeight(115)
        root.addWidget(self.event_log)

        self.footer_status = QLabel(
            "Phím: 0 chụp CAM0 • 1 chụp CAM1 • Space chụp cả hai • Ctrl+L chọn thư mục"
        )
        self.footer_status.setStyleSheet("padding: 5px; color: #555;")
        root.addWidget(self.footer_status)

        QShortcut(QKeySequence("0"), self, activated=lambda: self.capture_selected((0,), False))
        QShortcut(QKeySequence("1"), self, activated=lambda: self.capture_selected((1,), False))
        QShortcut(
            QKeySequence("Space"),
            self,
            activated=lambda: self.capture_selected((0, 1), False),
        )
        QShortcut(QKeySequence("Ctrl+L"), self, activated=self.select_folder)

    def select_folder(self) -> None:
        self.base_dir.mkdir(parents=True, exist_ok=True)
        selected = QFileDialog.getExistingDirectory(
            self, "Chọn thư mục gốc lưu dataset", str(self.base_dir)
        )
        if not selected:
            return
        self.base_dir = Path(selected)
        self.folder_edit.setText(str(self.base_dir))
        self.session_dir = None
        self.session_label.setText("Phiên hiện tại: chưa tạo trong thư mục mới")

    def create_new_session(self) -> None:
        try:
            self.base_dir.mkdir(parents=True, exist_ok=True)
            suffix = safe_name(self.session_name_edit.text())
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            folder_name = f"jetson_yolo_{timestamp}"
            if suffix:
                folder_name += f"_{suffix}"
            session = self.base_dir / folder_name
            counter = 1
            while session.exists():
                session = self.base_dir / f"{folder_name}_{counter:02d}"
                counter += 1

            for camera_id in (0, 1):
                role = ROLES[camera_id]
                role_dir = session / role
                (role_dir / "images").mkdir(parents=True, exist_ok=False)
                (role_dir / "labels").mkdir(parents=True, exist_ok=True)
                # Keep the numeric class contract next to every image set.
                # TensorRT only outputs numeric IDs; downstream robot logic
                # relies on this exact order.
                classes = "\n".join(CLASS_NAMES[camera_id]) + "\n"
                (role_dir / "classes.txt").write_text(
                    classes, encoding="utf-8")
            self.session_dir = session
            self.capture_counts = {0: 0, 1: 0}
            self.last_auto_gray = {0: None, 1: None}
            self._write_session_info()
            self._initialize_metadata_csv()
            for event in self.pending_events:
                self._append_crash_event_to_file(event)
            self.pending_events.clear()
            self.session_label.setText(f"Phiên hiện tại: {session}")
            self.append_event(f"Đã tạo phiên: {session}")
        except Exception as exc:
            QMessageBox.critical(self, "Không tạo được phiên", str(exc))

    def _ensure_session(self) -> bool:
        if self.session_dir is None:
            self.create_new_session()
        return self.session_dir is not None

    def _write_session_info(self) -> None:
        assert self.session_dir is not None
        content = f"""# Jetson production YOLO capture session
created_at: "{datetime.now().astimezone().isoformat(timespec='seconds')}"
camera_hardware: "2x IMX477 through V3Link"
camera_backend: "raw V4L2 RG10; shared worker for preview, ROS and dataset"
sensor_mode: 2
capture_resolution: [1920, 1080]
requested_sensor_fps: 15
processed_preview_resolution: [800, 450]
saved_resolution: [{EXPECTED_WIDTH}, {EXPECTED_HEIGHT}]
saved_encoding: "{EXPECTED_ENCODING}"
saved_format: "{self.format_note}"
geometry: "native 16:9; no crop, stretch, rotation, or overlay after ROS image_raw"
training:
  task: detect
  model: yolov8s
  imgsz: 640
  rect: false
  engine_input: [1, 3, 640, 640]
  preprocess: "letterbox 640x360 to 640x640; pad value 114; top=140 bottom=140; BGR-to-RGB; divide by 255"
  warning: "train from the saved 640x360 images; do not save or label a pre-letterboxed 640x640 image"
cameras:
  cam0:
    device: "/dev/video0"
    role: input
    topic: "{TOPICS[0]}"
    engine: "{ENGINE_PATHS[0]}"
    classes: [tray, cartridge]
  cam1:
    device: "/dev/video1"
    role: output
    topic: "{TOPICS[1]}"
    engine: "{ENGINE_PATHS[1]}"
    classes: [tray, cartridge, cartridgefall]
tone_pipeline:
  exposure: 58000
  gain: 66
  gamma: 0.50
  saturation: 3.60
  clarity: 0.38
  chroma_denoise: 0.30
notes:
  - "Only label the physical tray boundary; never label the whole frame as tray."
  - "Keep empty/background images with empty YOLO label files as hard negatives."
  - "Split train/val/test by capture session, not by adjacent frames."
"""
        (self.session_dir / "session_info.yaml").write_text(content, encoding="utf-8")

    def _initialize_metadata_csv(self) -> None:
        assert self.session_dir is not None
        path = self.session_dir / "metadata.csv"
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "captured_at",
                    "capture_group",
                    "camera_id",
                    "role",
                    "topic",
                    "relative_file",
                    "width",
                    "height",
                    "encoding",
                    "file_format",
                    "ros_stamp_ns",
                    "frame_age_ms",
                    "mean_luma",
                    "blur_variance",
                    "sha256",
                    "camera_health",
                    "auto_capture",
                ]
            )

    def on_frame(self, camera_id: int, image: np.ndarray, metadata: dict) -> None:
        self.frames[camera_id] = image
        self.frame_metadata[camera_id] = metadata
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb.shape
        qimage = QImage(
            rgb.data, width, height, channels * width, QImage.Format_RGB888
        ).copy()
        label = self.preview_labels[camera_id]
        label.setPixmap(
            QPixmap.fromImage(qimage).scaled(
                label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
        )

    def on_health(self, camera_id: int, health: str) -> None:
        self.health_text[camera_id] = health

    def on_worker_error(self, message: str) -> None:
        self.append_event(f"ROS ERROR: {message}")
        self.footer_status.setText(message)

    def refresh_health(self) -> None:
        now = time.monotonic()
        any_failed = False
        for camera_id in (0, 1):
            metadata = self.frame_metadata[camera_id]
            if not metadata:
                state = "WAITING"
                detail = "chưa nhận frame"
                color = "#604817"
            else:
                age = now - float(metadata["received_monotonic"])
                size_ok = (
                    metadata["width"] == EXPECTED_WIDTH
                    and metadata["height"] == EXPECTED_HEIGHT
                    and metadata["encoding"] == EXPECTED_ENCODING
                )
                if age > CRASH_FRAME_AGE_SECONDS:
                    state = "CRASH"
                    detail = f"mất frame {age:.1f}s"
                    color = "#9e2525"
                    any_failed = True
                elif not size_ok:
                    state = "INVALID"
                    detail = (
                        f"{metadata['width']}x{metadata['height']} "
                        f"{metadata['encoding']}"
                    )
                    color = "#9e2525"
                    any_failed = True
                else:
                    state = "STREAMING"
                    detail = (
                        f"{metadata['width']}x{metadata['height']} "
                        f"{metadata['encoding']} "
                        f"{metadata['measured_fps']:.1f} FPS age={age*1000:.0f}ms"
                    )
                    color = "#257348"

            previous = self.camera_state[camera_id]
            self.camera_state[camera_id] = state
            self.camera_status_labels[camera_id].setText(
                f"{state} — {detail}\n{self.health_text[camera_id]}"
            )
            self.camera_status_labels[camera_id].setStyleSheet(
                f"padding: 7px; background: {color}; color: white;"
            )
            if previous not in ("CRASH", "INVALID") and state in ("CRASH", "INVALID"):
                self.record_crash_event(
                    f"CAM{camera_id} {state}: {detail}; health={self.health_text[camera_id]}"
                )
            elif previous in ("CRASH", "INVALID") and state == "STREAMING":
                self.record_crash_event(f"CAM{camera_id} RECOVERED: {detail}")

        if any_failed:
            self.crash_status.setStyleSheet(
                "padding: 7px; background: #8f2424; color: white;"
            )
        else:
            self.crash_status.setStyleSheet(
                "padding: 7px; background: #274a35; color: white;"
            )
        states = " | ".join(
            f"CAM{camera_id}={self.camera_state[camera_id]}"
            for camera_id in (0, 1)
        )
        self.crash_status.setText(
            f"V4L2/CSI: {states} • nguồn frame dùng chung với YOLO"
        )

    def record_crash_event(self, message: str) -> None:
        stamped = f"{datetime.now().astimezone().isoformat(timespec='milliseconds')} {message}"
        self.append_event(stamped)
        if self.session_dir is None:
            self.pending_events.append(stamped)
        else:
            self._append_crash_event_to_file(stamped)

    def _append_crash_event_to_file(self, event: str) -> None:
        assert self.session_dir is not None
        with (self.session_dir / "camera_crash_events.log").open(
            "a", encoding="utf-8"
        ) as handle:
            handle.write(event + "\n")

    def append_event(self, message: str) -> None:
        self.event_log.appendPlainText(message)

    def toggle_auto_capture(self, enabled: bool) -> None:
        if enabled:
            if not self._ensure_session():
                self.auto_checkbox.setChecked(False)
                return
            interval_ms = int(self.interval_spin.value() * 1000)
            self.auto_timer.start(interval_ms)
            self.append_event(
                f"Auto capture started: interval={self.interval_spin.value():.1f}s"
            )
        else:
            self.auto_timer.stop()
            self.append_event("Auto capture stopped")

    def capture_selected(self, camera_ids: Iterable[int], auto_capture: bool) -> None:
        if not self._ensure_session():
            return
        assert self.session_dir is not None
        disk = shutil.disk_usage(self.session_dir)
        if disk.free < MIN_FREE_BYTES:
            self.auto_checkbox.setChecked(False)
            QMessageBox.critical(
                self,
                "Sắp hết dung lượng",
                "Còn dưới 5 GiB. Đã dừng chụp để tránh hỏng dataset/hệ thống.",
            )
            return

        capture_group = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        saved = 0
        errors = []
        for camera_id in camera_ids:
            result, message = self._save_camera_frame(
                camera_id, capture_group, auto_capture
            )
            if result:
                saved += 1
            elif message:
                errors.append(message)

        self.capture_summary.setText(
            f"Đã lưu: INPUT {self.capture_counts[0]} ảnh | "
            f"OUTPUT {self.capture_counts[1]} ảnh"
        )
        if saved:
            self.footer_status.setText(
                f"✅ Đã lưu {saved} ảnh vào {self.session_dir}"
            )
        elif errors and not auto_capture:
            self.footer_status.setText("⚠️ " + " | ".join(errors))

    def _save_camera_frame(
        self, camera_id: int, capture_group: str, auto_capture: bool
    ) -> Tuple[bool, str]:
        assert self.session_dir is not None
        frame = self.frames[camera_id]
        metadata = self.frame_metadata[camera_id]
        if frame is None or not metadata:
            return False, f"CAM{camera_id} chưa có frame"
        age = time.monotonic() - float(metadata["received_monotonic"])
        if age > MAX_SAVE_FRAME_AGE_SECONDS:
            return False, f"CAM{camera_id} frame cũ {age:.1f}s — không lưu"
        if frame.shape != (EXPECTED_HEIGHT, EXPECTED_WIDTH, 3):
            return False, f"CAM{camera_id} sai shape {frame.shape} — không lưu"

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mean_luma = float(gray.mean())
        blur_variance = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        if mean_luma < 5.0 or mean_luma > 250.0:
            return False, f"CAM{camera_id} ảnh trắng/đen bất thường mean={mean_luma:.1f}"

        if auto_capture and self.last_auto_gray[camera_id] is not None:
            change = float(
                cv2.absdiff(gray, self.last_auto_gray[camera_id]).mean()
            )
            if change < 1.5:
                return False, ""

        role = ROLES[camera_id]
        sequence = self.capture_counts[camera_id] + 1
        filename = (
            f"cam{camera_id}_{role}_{capture_group}_{sequence:06d}{self.image_suffix}"
        )
        relative = Path(role) / "images" / filename
        destination = self.session_dir / relative
        temporary = destination.with_name(destination.stem + ".tmp" + self.image_suffix)
        if not cv2.imwrite(str(temporary), frame, self.imwrite_params):
            return False, f"CAM{camera_id} lỗi ghi {self.image_format}"
        os.replace(temporary, destination)

        digest = hashlib.sha256(destination.read_bytes()).hexdigest()
        self.capture_counts[camera_id] = sequence
        if auto_capture:
            self.last_auto_gray[camera_id] = gray.copy()

        with (self.session_dir / "metadata.csv").open(
            "a", newline="", encoding="utf-8"
        ) as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    datetime.now().astimezone().isoformat(timespec="milliseconds"),
                    capture_group,
                    camera_id,
                    role,
                    TOPICS[camera_id],
                    str(relative),
                    EXPECTED_WIDTH,
                    EXPECTED_HEIGHT,
                    EXPECTED_ENCODING,
                    self.format_note,
                    metadata["ros_stamp_ns"],
                    round(age * 1000.0, 1),
                    round(mean_luma, 3),
                    round(blur_variance, 3),
                    digest,
                    self.health_text[camera_id],
                    int(auto_capture),
                ]
            )
        return True, str(destination)

    def closeEvent(self, event) -> None:
        self.auto_timer.stop()
        self.health_timer.stop()
        self.ros_worker.stop()
        event.accept()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Thu anh AI input 640x360 tu topic ROS de train YOLO.")
    parser.add_argument(
        "--jpg", nargs="?", type=int, const=95, default=None, metavar="Q",
        help="luu JPG chat luong Q (mac dinh 95) thay vi PNG. q95 nho hon PNG "
             "khoang 4 lan (PSNR ~39 dB), sai lech nen khong dang ke voi "
             "detection. Bo co nay = PNG khong mat mat.")
    args, qt_args = parser.parse_known_args()
    if args.jpg is not None and not (1 <= args.jpg <= 100):
        parser.error("--jpg phai trong khoang 1..100")

    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication([sys.argv[0]] + qt_args)
    window = DatasetCaptureWindow(jpeg_quality=args.jpg)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    signal_timer = QTimer()
    signal_timer.start(250)
    signal_timer.timeout.connect(lambda: None)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
