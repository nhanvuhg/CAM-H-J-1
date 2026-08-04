#!/usr/bin/env python3
"""ROS2 publisher for the stable dual V3Link V4L2 pipeline.

Logical mapping is intentional for this machine:
  CAM0/input  -> /dev/video1 (rack)
  CAM1/output -> /dev/video0 (green tray)

Frames are published as BGR8 640x360 for low latency. The same capture/tone
implementation is shared with the Desktop data-capture tool, which saves
1280x720 label images.
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.insert(0, "/home/nhan")
from capture_yolo_v4l2 import (  # noqa: E402
    DEVICE_MAP,
    DualV4L2Capture,
    EXPOSURE,
    FPS,
    GAIN,
    GAMMA,
    OUTPUT_BLACK_LEVEL,
    SAVED_HEIGHT,
    SAVED_WIDTH,
    SATURATION,
    WB,
)

ROS_WIDTH = 640
ROS_HEIGHT = 360

class V4L2DualCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("v4l2_dual_camera")
        self.cam0_topic = self.declare_parameter(
            "cam0_topic", "/cam0HP/image_raw"
        ).value
        self.cam1_topic = self.declare_parameter(
            "cam1_topic", "/cam1HP/image_raw"
        ).value
        self.cam0_health_topic = self.declare_parameter(
            "cam0_health_topic", "/camera/cam0/health"
        ).value
        self.cam1_health_topic = self.declare_parameter(
            "cam1_health_topic", "/camera/cam1/health"
        ).value
        # The ROS image is downscaled to 640x360, so a modestly stronger
        # luma-only edge lift is useful for YOLO labels.  Keep these as ROS
        # parameters so the production launch can tune them without changing
        # the dataset capture profile.
        self.ros_clarity = {
            0: float(self.declare_parameter("cam0_clarity", 0.45).value),
            1: float(self.declare_parameter("cam1_clarity", 0.60).value),
        }

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        health_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.image_publishers = [
            self.create_publisher(Image, self.cam0_topic, image_qos),
            self.create_publisher(Image, self.cam1_topic, image_qos),
        ]
        self.health_publishers = [
            self.create_publisher(String, self.cam0_health_topic, health_qos),
            self.create_publisher(String, self.cam1_health_topic, health_qos),
        ]
        self.last_published = [0.0, 0.0]
        self.published = [0, 0]
        self.capture: DualV4L2Capture | None = None
        try:
            self.capture = DualV4L2Capture(
                raw_width=1920,
                raw_height=1080,
                sensor_mode=2,
                # YOLO/ROS stays at the production source size for low
                # latency. The Desktop capture tool still saves 1280x720.
                output_width=ROS_WIDTH,
                output_height=ROS_HEIGHT,
                # Never decimate a Bayer mosaic with raw[::2, ::2]: that
                # selects only one RGGB photosite phase (red on this sensor),
                # then WB turns the whole ROS image magenta. Debayer all CFA
                # phases first; resize happens inside debayer().
                raw_decimation=1,
                clarity=self.ros_clarity,
            )
            self.capture.start()
            self.publish_timer = self.create_timer(0.01, self._publish_latest)
            self.health_timer = self.create_timer(1.0, self._publish_health)
            self.get_logger().info(
                f"V4L2 dual camera started: CAM0=/dev/video{DEVICE_MAP[0]} rack, "
                f"CAM1=/dev/video{DEVICE_MAP[1]} green output tray; raw=1920x1080, "
                f"published={ROS_WIDTH}x{ROS_HEIGHT}, exposure={EXPOSURE} "
                f"gain={GAIN} gamma={GAMMA:.2f} saturation={SATURATION:.2f} "
                f"WB={WB} clarity={self.ros_clarity}"
            )
        except Exception:
            self.close()
            raise

    def _publish_latest(self) -> None:
        if self.capture is None:
            return
        now_mono = time.monotonic()
        with self.capture.frame_lock:
            frames = [
                None if frame is None else frame.copy()
                for frame in self.capture.frames
            ]
            frame_times = list(self.capture.last_frame_time)
        for camera_id, frame in enumerate(frames):
            if frame is None or frame_times[camera_id] <= self.last_published[camera_id]:
                continue
            if frame.shape != (ROS_HEIGHT, ROS_WIDTH, 3):
                self.get_logger().warning(
                    f"CAM{camera_id} unexpected frame shape {frame.shape}"
                )
                continue
            message = Image()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = f"cam{camera_id}_optical_frame"
            message.height = ROS_HEIGHT
            message.width = ROS_WIDTH
            message.encoding = "bgr8"
            message.is_bigendian = 0
            message.step = ROS_WIDTH * 3
            message.data = frame.tobytes()
            self.image_publishers[camera_id].publish(message)
            self.last_published[camera_id] = frame_times[camera_id]
            self.published[camera_id] += 1

    def _publish_health(self) -> None:
        if self.capture is None:
            return
        now = time.monotonic()
        with self.capture.frame_lock:
            frame_times = list(self.capture.last_frame_time)
            fps = list(self.capture.fps)
            statuses = list(self.capture.status)
            reconnects = list(self.capture.reconnects)
        for camera_id in (0, 1):
            age = now - frame_times[camera_id] if frame_times[camera_id] else 999.0
            state = "STREAMING" if age < 2.5 else statuses[camera_id]
            message = String()
            message.data = (
                f"{state} device=/dev/video{DEVICE_MAP[camera_id]} "
                f"size={ROS_WIDTH}x{ROS_HEIGHT} fps={fps[camera_id]:.1f} "
                f"age={age:.2f}s reconnects={reconnects[camera_id]} "
                f"published={self.published[camera_id]}"
            )
            self.health_publishers[camera_id].publish(message)

    def close(self) -> None:
        if self.capture is not None:
            self.capture.close()
            self.capture = None


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = V4L2DualCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(f"V4L2 camera node failed: {exc}")
        else:
            print(f"V4L2 camera node failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
