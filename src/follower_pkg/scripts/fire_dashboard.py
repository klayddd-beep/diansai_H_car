#!/usr/bin/env python3
"""Full-screen aircraft telemetry and fire-mission dashboard for the car."""

import math
import os
import time

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float32MultiArray, String


PHASES = {
    0: "待命", 1: "起飞", 2: "巡逻", 3: "接近火源", 4: "降高", 5: "悬停",
    6: "抛包", 7: "恢复巡逻", 8: "返航", 9: "降落", 10: "完成", 255: "未知",
}
WINDOW_NAME = "Fire mission dashboard"


class FireDashboard(Node):
    def __init__(self):
        super().__init__("fire_dashboard")
        self.declare_parameter("fullscreen", True)
        self.declare_parameter("headless", False)
        self.declare_parameter("window_width", 1280)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("field_width_dm", 48.0)
        self.declare_parameter("field_height_dm", 40.0)
        self.declare_parameter("home_x_dm", 13.5)
        self.declare_parameter("home_y_dm", 2.5)
        self.declare_parameter("obstacles_dm", [])
        self.declare_parameter("history_limit", 20000)
        self.declare_parameter("link_timeout_s", 3.0)

        self.fullscreen = bool(self.get_parameter("fullscreen").value)
        self.headless = bool(self.get_parameter("headless").value)
        self.width = int(self.get_parameter("window_width").value)
        self.height = int(self.get_parameter("window_height").value)
        self.field_w = float(self.get_parameter("field_width_dm").value)
        self.field_h = float(self.get_parameter("field_height_dm").value)
        self.home = (
            float(self.get_parameter("home_x_dm").value),
            float(self.get_parameter("home_y_dm").value),
        )
        flat_obstacles = list(self.get_parameter("obstacles_dm").value)
        self.obstacles = [
            tuple(flat_obstacles[i:i + 4])
            for i in range(0, len(flat_obstacles) - 3, 4)
        ]
        self.history_limit = int(self.get_parameter("history_limit").value)
        self.link_timeout = float(self.get_parameter("link_timeout_s").value)
        self.font_path = self._find_cjk_font()
        self.fonts = {}

        self.telemetry = None
        self.last_telemetry_time = None
        self.track = []
        self.fire_point = None
        self.car_status = "ready"
        self.create_subscription(
            Float32MultiArray, "/drone_telemetry", self.on_telemetry, 10)
        self.create_subscription(Float32MultiArray, "/fire_event", self.on_fire, 10)
        self.create_subscription(String, "/fire_mission_status", self.on_status, 10)
        self.start_publisher = self.create_publisher(Empty, "/drone_start", 10)

        if not self.headless:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            if self.fullscreen:
                cv2.setWindowProperty(
                    WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                # Ask OpenCV for the actual full-screen drawable size when supported.
                _, _, detected_w, detected_h = cv2.getWindowImageRect(WINDOW_NAME)
                if detected_w > 100 and detected_h > 100:
                    self.width, self.height = detected_w, detected_h
            else:
                cv2.resizeWindow(WINDOW_NAME, self.width, self.height)

    @staticmethod
    def _find_cjk_font():
        candidates = (
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf",
            "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
            "/usr/share/fonts/truetype/arphic/ukai.ttc",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        )
        return next((path for path in candidates if os.path.exists(path)), None)

    def font(self, size):
        size = max(14, int(size))
        if size not in self.fonts:
            if self.font_path:
                self.fonts[size] = ImageFont.truetype(self.font_path, size)
            else:
                self.fonts[size] = ImageFont.load_default()
        return self.fonts[size]

    def on_telemetry(self, msg):
        # [x_dm, y_dm, distance_dm, height_dm, phase, seq, stamp_ms]
        if len(msg.data) < 7:
            self.get_logger().warning("忽略长度不足的 /drone_telemetry")
            return
        self.telemetry = tuple(float(value) for value in msg.data[:7])
        self.last_telemetry_time = time.monotonic()
        point = self.telemetry[:2]
        if all(math.isfinite(value) for value in point):
            if not self.track or point != self.track[-1]:
                self.track.append(point)
                if len(self.track) > self.history_limit:
                    del self.track[:len(self.track) - self.history_limit]

    def on_fire(self, msg):
        if len(msg.data) >= 2:
            self.fire_point = (float(msg.data[0]), float(msg.data[1]))

    def on_status(self, msg):
        self.car_status = msg.data

    def field_to_screen(self, point, arena):
        left, top, width, height = arena
        x, y = point
        return (
            round(left + x / self.field_w * width),
            round(top + height - y / self.field_h * height),
        )

    @staticmethod
    def star_points(center, outer, inner):
        cx, cy = center
        points = []
        for index in range(10):
            radius = outer if index % 2 == 0 else inner
            angle = -math.pi / 2 + index * math.pi / 5
            points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
        return points

    def text(self, draw, value, position, size, color=(235, 240, 245)):
        draw.text(position, str(value), font=self.font(size), fill=color)

    def draw_map(self, draw, arena):
        left, top, width, height = arena
        right, bottom = left + width, top + height
        draw.rectangle((left, top, right, bottom), fill=(33, 43, 48))
        for x in range(0, int(self.field_w) + 1, 4):
            draw.line(
                (self.field_to_screen((x, 0), arena),
                 self.field_to_screen((x, self.field_h), arena)),
                fill=(58, 70, 75), width=1)
        for y in range(0, int(self.field_h) + 1, 4):
            draw.line(
                (self.field_to_screen((0, y), arena),
                 self.field_to_screen((self.field_w, y), arena)),
                fill=(58, 70, 75), width=1)

        for xmin, ymin, xmax, ymax in self.obstacles:
            upper_left = self.field_to_screen((xmin, ymax), arena)
            lower_right = self.field_to_screen((xmax, ymin), arena)
            draw.rectangle((upper_left, lower_right), fill=(224, 226, 220),
                           outline=(130, 135, 130), width=2)

        takeoff_upper_left = self.field_to_screen((0, 7), arena)
        takeoff_lower_right = self.field_to_screen((11, 0), arena)
        draw.rectangle((takeoff_upper_left, takeoff_lower_right), fill=(5, 7, 8),
                       outline=(150, 155, 160), width=2)
        self.text(draw, "起降区", (takeoff_upper_left[0] + 5, takeoff_upper_left[1] + 3),
                  height * 0.025)

        home_px = self.field_to_screen(self.home, arena)
        radius = max(7, width // 90)
        draw.ellipse((home_px[0] - radius, home_px[1] - radius,
                      home_px[0] + radius, home_px[1] + radius), fill=(225, 55, 55))
        self.text(draw, "车", (home_px[0] + 9, home_px[1] - 15), height * 0.03,
                  (255, 100, 90))

        if len(self.track) >= 2:
            draw.line([self.field_to_screen(point, arena) for point in self.track],
                      fill=(25, 210, 235), width=max(2, width // 300), joint="curve")
        if self.telemetry:
            drone_px = self.field_to_screen(self.telemetry[:2], arena)
            radius = max(8, width // 75)
            draw.ellipse((drone_px[0] - radius, drone_px[1] - radius,
                          drone_px[0] + radius, drone_px[1] + radius),
                         fill=(255, 220, 35), outline=(15, 15, 15), width=2)

        if self.fire_point:
            fire_px = self.field_to_screen(self.fire_point, arena)
            draw.polygon(self.star_points(fire_px, 15, 7), fill=(255, 55, 35))
            self.text(draw, f"火源 ({self.fire_point[0]:.1f}, {self.fire_point[1]:.1f})",
                      (fire_px[0] + 17, fire_px[1] - 18), height * 0.027, (255, 90, 65))

        draw.rectangle((left, top, right, bottom), outline=(170, 180, 185), width=2)
        self.text(draw, "场地系：原点左下 / 单位 dm", (left, bottom + 5), height * 0.028)

    def draw_panel(self, draw, panel):
        left, top, width, height = panel
        base_size = min(width * 0.075, height * 0.055)
        small_size = base_size * 0.72
        x = left + width * 0.06
        y = top + height * 0.04
        self.text(draw, "无人机遥测", (x, y), base_size * 1.15, (90, 220, 255))
        y += base_size * 1.65

        if self.telemetry:
            px, py, distance, altitude, phase_value, _, _ = self.telemetry
            phase = PHASES.get(int(phase_value), f"未知({int(phase_value)})")
            lines = (
                f"位置  ({px:5.1f}, {py:5.1f}) dm",
                f"高度  {altitude:6.1f} dm",
                f"阶段  {phase}",
                f"里程  {distance:7.1f} dm",
            )
        else:
            lines = ("位置  --", "高度  --", "阶段  --", "里程  --")
        for line in lines:
            self.text(draw, line, (x, y), base_size)
            y += base_size * 1.5

        elapsed = (math.inf if self.last_telemetry_time is None
                   else time.monotonic() - self.last_telemetry_time)
        if elapsed <= self.link_timeout:
            link_text, link_color = "链路  正常", (65, 225, 120)
        elif math.isinf(elapsed):
            link_text, link_color = "链路  离线（未收到）", (255, 75, 65)
        else:
            link_text, link_color = f"链路  离线 {elapsed:.1f}s", (255, 75, 65)
        self.text(draw, link_text, (x, y), base_size, link_color)
        y += base_size * 1.65
        self.text(draw, f"消防车  {self.car_status}", (x, y), small_size)
        y += base_size * 1.35
        fire = (f"火源  ({self.fire_point[0]:.1f}, {self.fire_point[1]:.1f}) dm"
                if self.fire_point else "火源  --")
        self.text(draw, fire, (x, y), small_size, (255, 120, 90))
        self.text(draw, "按 空格/回车 启动无人机", (x, top + height - base_size * 1.4),
                  small_size, (185, 190, 195))

    def render(self):
        image = Image.new("RGB", (self.width, self.height), (14, 20, 24))
        draw = ImageDraw.Draw(image)
        margin = max(12, int(min(self.width, self.height) * 0.025))
        panel_width = max(330, int(self.width * 0.32))
        available_w = self.width - panel_width - margin * 3
        available_h = self.height - margin * 3
        arena_w = min(available_w, available_h * self.field_w / self.field_h)
        arena_h = arena_w * self.field_h / self.field_w
        if arena_h > available_h:
            arena_h = available_h
            arena_w = arena_h * self.field_w / self.field_h
        arena = (margin, margin, round(arena_w), round(arena_h))
        panel = (self.width - panel_width - margin, margin,
                 panel_width, self.height - 2 * margin)
        self.draw_map(draw, arena)
        self.draw_panel(draw, panel)
        if not self.headless:
            cv2.imshow(WINDOW_NAME, cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR))
        return image

    def process_events(self):
        if self.headless:
            return True
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            return False
        if key in (13, 32):
            self.start_publisher.publish(Empty())
        return cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) >= 1


def main(args=None):
    rclpy.init(args=args)
    node = FireDashboard()
    try:
        running = True
        while rclpy.ok() and running:
            rclpy.spin_once(node, timeout_sec=0.0)
            node.render()
            running = node.process_events()
            time.sleep(1.0 / 30.0)
    finally:
        if not node.headless:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
