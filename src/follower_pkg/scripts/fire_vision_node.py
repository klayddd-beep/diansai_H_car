#!/usr/bin/env python3
"""ROS 2 camera node that publishes the center of a red fire target."""

import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from follower_pkg.msg import FireDetection
from fire_vision_core import RedTargetDetector, orient_frame


DEFAULT_CAMERA = (
    "/dev/v4l/by-id/"
    "usb-5M_WDR_Camera_5M_WDR_Camera_SN20210707-video-index0"
)


class FireVisionNode(Node):
    """Capture the configured camera and publish red-target detections."""

    def __init__(self) -> None:
        """Create publishers, detector, and the frame processing timer."""
        super().__init__("fire_vision")
        self.declare_parameter("device_path", DEFAULT_CAMERA)
        self.declare_parameter("capture_width", 640)
        self.declare_parameter("capture_height", 480)
        self.declare_parameter("capture_fps", 30.0)
        self.declare_parameter("processing_rate_hz", 15.0)
        self.declare_parameter("rotate_cw_deg", 90)
        self.declare_parameter("flip_horizontal", False)
        self.declare_parameter("flip_vertical", False)
        self.declare_parameter("aim_x_ratio", 0.5)
        self.declare_parameter("aim_y_ratio", 0.5)
        self.declare_parameter("roi", [0.08, 0.05, 0.92, 0.95])
        self.declare_parameter("hue_low_1", 0)
        self.declare_parameter("hue_high_1", 12)
        self.declare_parameter("hue_low_2", 168)
        self.declare_parameter("hue_high_2", 179)
        self.declare_parameter("saturation_min", 70)
        self.declare_parameter("value_min", 45)
        self.declare_parameter("red_min", 65)
        self.declare_parameter("red_over_green_min", 25)
        self.declare_parameter("red_over_blue_min", 20)
        self.declare_parameter("min_area_ratio", 0.0002)
        self.declare_parameter("max_area_ratio", 0.20)
        self.declare_parameter("morphology_open_px", 3)
        self.declare_parameter("morphology_close_px", 7)
        self.declare_parameter("track_memory_frames", 5)
        self.declare_parameter("camera_retry_s", 1.0)
        self.declare_parameter("publish_debug_image", True)

        self.device_path = str(self.get_parameter("device_path").value)
        self.capture_width = int(self.get_parameter("capture_width").value)
        self.capture_height = int(self.get_parameter("capture_height").value)
        self.capture_fps = float(self.get_parameter("capture_fps").value)
        self.rotate_cw_deg = int(self.get_parameter("rotate_cw_deg").value)
        self.flip_horizontal = bool(
            self.get_parameter("flip_horizontal").value
        )
        self.flip_vertical = bool(self.get_parameter("flip_vertical").value)
        self.camera_retry_s = float(self.get_parameter("camera_retry_s").value)
        processing_rate_hz = float(
            self.get_parameter("processing_rate_hz").value
        )
        if self.capture_width <= 0 or self.capture_height <= 0:
            raise ValueError("capture dimensions must be positive")
        if self.capture_fps <= 0.0 or processing_rate_hz <= 0.0:
            raise ValueError("camera rates must be positive")
        if self.camera_retry_s <= 0.0:
            raise ValueError("camera_retry_s must be positive")

        self.detector = RedTargetDetector(
            hue_low_1=self.get_parameter("hue_low_1").value,
            hue_high_1=self.get_parameter("hue_high_1").value,
            hue_low_2=self.get_parameter("hue_low_2").value,
            hue_high_2=self.get_parameter("hue_high_2").value,
            saturation_min=self.get_parameter("saturation_min").value,
            value_min=self.get_parameter("value_min").value,
            red_min=self.get_parameter("red_min").value,
            red_over_green_min=self.get_parameter("red_over_green_min").value,
            red_over_blue_min=self.get_parameter("red_over_blue_min").value,
            min_area_ratio=self.get_parameter("min_area_ratio").value,
            max_area_ratio=self.get_parameter("max_area_ratio").value,
            morphology_open_px=self.get_parameter(
                "morphology_open_px"
            ).value,
            morphology_close_px=self.get_parameter(
                "morphology_close_px"
            ).value,
            roi=self.get_parameter("roi").value,
            aim_x_ratio=self.get_parameter("aim_x_ratio").value,
            aim_y_ratio=self.get_parameter("aim_y_ratio").value,
            track_memory_frames=self.get_parameter(
                "track_memory_frames"
            ).value,
        )

        self.detection_pub = self.create_publisher(
            FireDetection, "/fire_vision/detection", 10
        )
        self.publish_debug = bool(
            self.get_parameter("publish_debug_image").value
        )
        self.debug_pub = (
            self.create_publisher(Image, "/fire_vision/debug_image", 2)
            if self.publish_debug
            else None
        )
        self.bridge = CvBridge() if self.publish_debug else None
        self.capture = None
        self.last_open_attempt_ns = 0
        self.camera_error_reported = False
        self.timer = self.create_timer(
            1.0 / processing_rate_hz, self.on_timer
        )
        self.get_logger().info(
            f"fire vision ready: {self.device_path}, "
            f"{self.capture_width}x{self.capture_height}"
            f"@{self.capture_fps:.0f}, "
            f"rotate_cw={self.rotate_cw_deg}"
        )

    def _release_camera(self) -> None:
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    def _open_camera(self) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        if (
            self.last_open_attempt_ns
            and (
                (now_ns - self.last_open_attempt_ns) / 1e9
                < self.camera_retry_s
            )
        ):
            return False
        self.last_open_attempt_ns = now_ns
        if not os.path.exists(self.device_path):
            if not self.camera_error_reported:
                self.get_logger().error(
                    f"camera device does not exist: {self.device_path}"
                )
                self.camera_error_reported = True
            return False

        capture = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.capture_width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.capture_height)
        capture.set(cv2.CAP_PROP_FPS, self.capture_fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            capture.release()
            if not self.camera_error_reported:
                self.get_logger().error(
                    f"failed to open camera: {self.device_path}"
                )
                self.camera_error_reported = True
            return False
        self.capture = capture
        self.camera_error_reported = False
        self.get_logger().info(f"camera opened: {self.device_path}")
        return True

    def on_timer(self) -> None:
        """Capture, orient, detect, and publish one camera frame."""
        if self.capture is None and not self._open_camera():
            return
        ok, frame = self.capture.read()
        if not ok or frame is None:
            if not self.camera_error_reported:
                self.get_logger().error("camera frame read failed; reopening")
                self.camera_error_reported = True
            self._release_camera()
            return

        oriented = orient_frame(
            frame,
            self.rotate_cw_deg,
            self.flip_horizontal,
            self.flip_vertical,
        )
        result, _, contour = self.detector.detect(oriented)
        stamp = self.get_clock().now().to_msg()
        message = FireDetection()
        message.header.stamp = stamp
        message.header.frame_id = "fire_camera"
        message.detected = result.detected
        message.center_x_px = float(result.center_x_px)
        message.center_y_px = float(result.center_y_px)
        message.error_x_norm = float(result.error_x_norm)
        message.error_y_norm = float(result.error_y_norm)
        message.area_ratio = float(result.area_ratio)
        message.image_width = oriented.shape[1]
        message.image_height = oriented.shape[0]
        self.detection_pub.publish(message)

        if self.debug_pub is not None and self.bridge is not None:
            annotated = self.detector.annotate(oriented, result, contour)
            image_message = self.bridge.cv2_to_imgmsg(
                annotated, encoding="bgr8"
            )
            image_message.header = message.header
            self.debug_pub.publish(image_message)

    def destroy_node(self):
        """Release the V4L2 handle before destroying the ROS node."""
        self._release_camera()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the ROS 2 fire vision node."""
    rclpy.init(args=args)
    node = FireVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
