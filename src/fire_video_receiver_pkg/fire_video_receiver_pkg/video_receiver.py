#!/usr/bin/env python3
"""Receive and optionally display the drone's compressed debug video."""

import copy
import signal
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from .receiver_core import LinkState, decode_jpeg, link_state


DEFAULT_TOPIC = "/fly/fire/debug_image/compressed"
DEFAULT_WINDOW = "Drone fire debug video"


def make_video_qos():
    """Return the sensor-data QoS with a one-frame receive queue."""
    video_qos = copy.copy(qos_profile_sensor_data)
    video_qos.depth = 1
    return video_qos


class FireVideoReceiver(Node):
    """Best-effort JPEG receiver kept separate from the active fire mission."""

    def __init__(self):
        super().__init__("fire_video_receiver")
        self.declare_parameter("topic", DEFAULT_TOPIC)
        self.declare_parameter("display", True)
        self.declare_parameter("link_timeout_s", 2.0)
        self.declare_parameter("stats_period_s", 5.0)
        self.declare_parameter("window_name", DEFAULT_WINDOW)

        self.topic = str(self.get_parameter("topic").value)
        self.display = bool(self.get_parameter("display").value)
        self.link_timeout_s = max(
            0.1, float(self.get_parameter("link_timeout_s").value))
        self.stats_period_s = max(
            0.5, float(self.get_parameter("stats_period_s").value))
        self.window_name = str(self.get_parameter("window_name").value)

        self.frame = None
        self.last_frame_at = None
        self.received_frames = 0
        self.received_bytes = 0
        self.bad_frames = 0
        self.stats_started_at = time.monotonic()
        self.last_reported_state = None
        self.window_created = False

        # Keep the standard sensor-data policy (in particular BEST_EFFORT),
        # while limiting this low-latency video subscriber to the latest frame.
        self.subscription = self.create_subscription(
            CompressedImage,
            self.topic,
            self._on_image,
            make_video_qos(),
        )
        self.status_timer = self.create_timer(0.25, self._update_status)
        self.stats_timer = self.create_timer(
            self.stats_period_s, self._report_stats)
        self.display_timer = (
            self.create_timer(1.0 / 30.0, self._render)
            if self.display else None
        )
        self.get_logger().info(
            f"standby receiver subscribed to {self.topic} "
            "(BEST_EFFORT, KEEP_LAST(1)); remote header.stamp is ignored")

    def _on_image(self, msg):
        if "jpeg" not in msg.format.lower() and "jpg" not in msg.format.lower():
            self._warn_bad_frame(f"unsupported format={msg.format!r}")
            return
        frame = decode_jpeg(msg.data)
        if frame is None:
            self._warn_bad_frame("invalid/incomplete JPEG")
            return
        self.frame = frame
        self.last_frame_at = time.monotonic()
        self.received_frames += 1
        self.received_bytes += len(msg.data)

    def _warn_bad_frame(self, reason):
        self.bad_frames += 1
        # A damaged best-effort frame is expected occasionally. Avoid flooding
        # the terminal while still leaving a useful diagnostic breadcrumb.
        if self.bad_frames == 1 or self.bad_frames % 20 == 0:
            self.get_logger().warning(
                f"dropped compressed frame: {reason} "
                f"(bad_frames={self.bad_frames})")

    def _current_state(self):
        return link_state(
            self.last_frame_at, time.monotonic(), self.link_timeout_s)

    def _update_status(self):
        state = self._current_state()
        if state == self.last_reported_state:
            return
        self.last_reported_state = state
        if state == LinkState.WAITING:
            self.get_logger().info("WAITING: no drone video received yet")
        elif state == LinkState.LIVE:
            self.get_logger().info("LIVE: drone video link is active")
        else:
            self.get_logger().warning(
                f"LOST: no valid frame for more than "
                f"{self.link_timeout_s:.1f} s")

    def _report_stats(self):
        now = time.monotonic()
        elapsed = max(now - self.stats_started_at, 1e-6)
        hz = self.received_frames / elapsed
        kbps = self.received_bytes * 8.0 / elapsed / 1000.0
        self.get_logger().info(
            f"video stats: state={self._current_state().value} "
            f"rate={hz:.1f} Hz bitrate={kbps:.1f} kbps "
            f"bad_frames={self.bad_frames}")
        self.received_frames = 0
        self.received_bytes = 0
        self.stats_started_at = now

    @staticmethod
    def _status_canvas(text):
        canvas = np.full((480, 640, 3), (20, 20, 20), dtype=np.uint8)
        cv2.putText(
            canvas, text, (45, 250), cv2.FONT_HERSHEY_SIMPLEX,
            1.2, (160, 190, 210), 2, cv2.LINE_AA)
        return canvas

    def _render(self):
        state = self._current_state()
        if self.frame is None:
            view = self._status_canvas("WAITING FOR DRONE VIDEO")
        else:
            view = self.frame.copy()
            if state == LinkState.LOST:
                gray = cv2.cvtColor(view, cv2.COLOR_BGR2GRAY)
                view = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                overlay = view.copy()
                cv2.rectangle(
                    overlay, (0, 0), (view.shape[1], view.shape[0]),
                    (0, 0, 0), -1)
                view = cv2.addWeighted(view, 0.55, overlay, 0.45, 0.0)
                cv2.putText(
                    view, "LOST", (24, 62), cv2.FONT_HERSHEY_SIMPLEX,
                    1.6, (0, 0, 255), 3, cv2.LINE_AA)

        if not self.window_created:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            self.window_created = True
        cv2.imshow(self.window_name, view)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q"), ord("Q")):
            self.get_logger().info("viewer close requested")
            rclpy.shutdown()
            return
        try:
            visible = cv2.getWindowProperty(
                self.window_name, cv2.WND_PROP_VISIBLE)
        except cv2.error:
            visible = 0
        if visible < 1:
            self.get_logger().info("viewer window closed")
            rclpy.shutdown()

    def close_window(self):
        if self.window_created:
            try:
                cv2.destroyWindow(self.window_name)
            except cv2.error:
                pass
            self.window_created = False


def main(args=None):
    rclpy.init(args=args)
    node = FireVideoReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # ros2 launch may forward a second SIGINT while this process is
        # already cleaning up. Do not let it interrupt destroy_node().
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    finally:
        node.close_window()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
