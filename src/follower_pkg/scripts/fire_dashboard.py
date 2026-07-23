#!/usr/bin/env python3
"""Full-screen aircraft telemetry and fire-mission dashboard for the car."""

import math
import os
import re
import subprocess
import time

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image, ImageDraw, ImageFont
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import Empty, Float32MultiArray, String


PHASES = {
    0: "待命", 1: "起飞", 2: "巡逻", 3: "接近火源", 4: "降高", 5: "悬停",
    6: "抛包", 7: "恢复巡逻", 8: "返航", 9: "降落", 10: "完成", 255: "未知",
}
CAR_STATUSES = {
    "ready": "待命",
    "enroute": "前往火源",
    "extinguishing": "灭火中",
    "returning": "返航中",
    "done": "任务完成",
}
WINDOW_NAME = "Fire mission dashboard"

BG = (8, 14, 20)
SURFACE = (17, 26, 34)
SURFACE_ALT = (23, 34, 43)
BORDER = (48, 65, 76)
TEXT = (237, 243, 247)
TEXT_MUTED = (142, 158, 170)
CYAN = (41, 202, 230)
GREEN = (55, 210, 126)
AMBER = (255, 190, 60)
RED = (255, 82, 72)


class FireDashboard(Node):
    def __init__(self):
        super().__init__("fire_dashboard")
        self.declare_parameter("fullscreen", True)
        self.declare_parameter("headless", False)
        self.declare_parameter("window_width", 1280)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("field_width_dm", 48.0)
        self.declare_parameter("field_height_dm", 40.0)
        self.declare_parameter(
            "takeoff_zone_dm", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter(
            "parking_zone_dm", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("home_x_dm", 13.5)
        self.declare_parameter("home_y_dm", 2.5)
        self.declare_parameter("obstacles_dm", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("history_limit", 20000)
        self.declare_parameter("link_timeout_s", 3.0)
        self.declare_parameter("camera_timeout_s", 1.0)

        self.fullscreen = bool(self.get_parameter("fullscreen").value)
        self.headless = bool(self.get_parameter("headless").value)
        self.width = int(self.get_parameter("window_width").value)
        self.height = int(self.get_parameter("window_height").value)
        self.field_w = float(self.get_parameter("field_width_dm").value)
        self.field_h = float(self.get_parameter("field_height_dm").value)
        if self.field_w <= 0.0 or self.field_h <= 0.0:
            raise ValueError(
                "field_width_dm and field_height_dm must be positive")
        self.home = (
            float(self.get_parameter("home_x_dm").value),
            float(self.get_parameter("home_y_dm").value),
        )
        self.takeoff_zone = self._read_zone_parameter("takeoff_zone_dm")
        self.parking_zone = self._read_zone_parameter("parking_zone_dm")
        flat_obstacles = list(self.get_parameter("obstacles_dm").value)
        if len(flat_obstacles) % 4 != 0:
            raise ValueError(
                "obstacles_dm must contain xmin,ymin,xmax,ymax groups")
        if not all(math.isfinite(value) for value in flat_obstacles):
            raise ValueError("obstacles_dm values must be finite")
        self.obstacles = [
            tuple(flat_obstacles[i:i + 4])
            for i in range(0, len(flat_obstacles), 4)
        ]
        if any(xmin >= xmax or ymin >= ymax
               for xmin, ymin, xmax, ymax in self.obstacles):
            raise ValueError(
                "each obstacle must satisfy xmin<xmax and ymin<ymax")
        self.history_limit = max(
            1, int(self.get_parameter("history_limit").value))
        self.link_timeout = max(
            0.1, float(self.get_parameter("link_timeout_s").value))
        self.camera_timeout = max(
            0.1, float(self.get_parameter("camera_timeout_s").value))
        self.font_path = self._find_cjk_font()
        if self.font_path is None:
            self.get_logger().warning(
                "未找到中文字体，仪表盘中文会显示为方块；请安装 fonts-noto-cjk，"
                "并用 'fc-list :lang=zh | head' 自检")
        self.fonts = {}

        self.telemetry = None
        self.last_telemetry_time = None
        self.track = []
        self.fire_point = None
        self.car_status = "ready"
        self.bridge = CvBridge()
        self.camera_frame = None
        self.last_camera_time = None
        self.start_button_rect = None
        self.reset_button_rect = None
        self.map_tab_rect = None
        self.camera_tab_rect = None
        self.close_button_rect = None
        self.active_tab = "map"
        self.exit_requested = False
        self.start_feedback_until = 0.0
        self.reset_feedback_until = 0.0
        self.start_cooldown_s = 2.0
        self.next_screen_size_check = 0.0
        self.create_subscription(
            Float32MultiArray, "/drone_telemetry", self.on_telemetry, 10)
        self.create_subscription(
            Float32MultiArray, "/fire_event", self.on_fire, 10)
        self.create_subscription(
            String, "/fire_mission_status", self.on_status, 10)
        self.create_subscription(
            RosImage, "/fire_vision/debug_image", self.on_camera_image, 2)
        self.start_publisher = self.create_publisher(Empty, "/drone_start", 10)
        self.reset_publisher = self.create_publisher(
            Empty, "/fire_mission_reset", 10)

        if not self.headless:
            if self.fullscreen:
                if not self._refresh_fullscreen_size(force=True):
                    self.get_logger().warning(
                        "无法通过 XRandR 获取屏幕尺寸，使用配置的画布尺寸 "
                        f"{self.width}x{self.height}")
            # HighGUI keeps the source image's aspect ratio by default.  When
            # that ratio differs from the display, the unused area becomes a
            # visible border even though the native window is full-screen.
            # FREERATIO makes the image fill the complete window client area.
            cv2.namedWindow(
                WINDOW_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)
            cv2.setWindowProperty(
                WINDOW_NAME, cv2.WND_PROP_ASPECT_RATIO,
                cv2.WINDOW_FREERATIO)
            if self.fullscreen:
                cv2.moveWindow(WINDOW_NAME, 0, 0)
                cv2.setWindowProperty(
                    WINDOW_NAME, cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN)
            else:
                cv2.resizeWindow(WINDOW_NAME, self.width, self.height)
            cv2.setMouseCallback(WINDOW_NAME, self.on_mouse)

    def _read_zone_parameter(self, name):
        values = tuple(float(value) for value in self.get_parameter(name).value)
        if (len(values) != 4 or not all(math.isfinite(value) for value in values)
                or values[0] >= values[2] or values[1] >= values[3]):
            raise ValueError(
                f"{name} must be a finite xmin,ymin,xmax,ymax rectangle")
        return values

    @staticmethod
    def _screen_size_from_xrandr(output):
        match = re.search(r"\bcurrent\s+(\d+)\s+x\s+(\d+)\b", output)
        if match is None:
            return None
        width, height = (int(value) for value in match.groups())
        if width <= 100 or height <= 100:
            return None
        return width, height

    def _detect_screen_size(self):
        try:
            result = subprocess.run(
                ["xrandr", "--current"], check=False, capture_output=True,
                text=True, timeout=2.0)
        except (FileNotFoundError, subprocess.SubprocessError, OSError):
            return None
        if result.returncode != 0:
            return None
        return self._screen_size_from_xrandr(result.stdout)

    def _refresh_fullscreen_size(self, force=False):
        if self.headless or not self.fullscreen:
            return False
        now = time.monotonic()
        if not force and now < self.next_screen_size_check:
            return False
        self.next_screen_size_check = now + 1.0
        screen_size = self._detect_screen_size()
        if screen_size is None:
            return False
        if screen_size != (self.width, self.height):
            previous = (self.width, self.height)
            self.width, self.height = screen_size
            self.get_logger().info(
                f"全屏画布尺寸: {previous[0]}x{previous[1]} -> "
                f"{self.width}x{self.height}")
        elif force:
            self.get_logger().info(
                f"全屏画布尺寸: {self.width}x{self.height}")
        return True

    @staticmethod
    def _find_cjk_font():
        candidates = (
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf",
            "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
            "/usr/share/fonts/truetype/arphic/ukai.ttc",
            "/usr/share/fonts/truetype/arphic/uming.ttc",
        )
        return next(
            (path for path in candidates if os.path.exists(path)), None)

    def font(self, size):
        size = max(14, int(size))
        if size not in self.fonts:
            if self.font_path:
                self.fonts[size] = ImageFont.truetype(self.font_path, size)
            else:
                self.fonts[size] = ImageFont.load_default()
        return self.fonts[size]

    def on_telemetry(self, msg):
        # [x_dm, y_dm, distance_dm, height_dm, phase, seq]
        if len(msg.data) < 6:
            self.get_logger().warning("忽略长度不足的 /drone_telemetry")
            return
        telemetry = tuple(float(value) for value in msg.data[:6])
        if not all(math.isfinite(value) for value in telemetry):
            self.get_logger().warning("忽略包含 NaN/Inf 的 /drone_telemetry")
            return
        self.telemetry = telemetry
        self.last_telemetry_time = time.monotonic()
        point = self.telemetry[:2]
        if not self.track or point != self.track[-1]:
            self.track.append(point)
            if len(self.track) > self.history_limit:
                del self.track[:len(self.track) - self.history_limit]

    def on_fire(self, msg):
        if len(msg.data) >= 2:
            point = (float(msg.data[0]), float(msg.data[1]))
            if not all(math.isfinite(value) for value in point):
                self.get_logger().warning("忽略包含 NaN/Inf 的 /fire_event")
                return
            self.fire_point = point

    def on_status(self, msg):
        self.car_status = msg.data

    def on_camera_image(self, msg):
        try:
            self.camera_frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8").copy()
        except (CvBridgeError, TypeError, ValueError, cv2.error) as error:
            self.get_logger().warning(f"摄像头图像转换失败: {error}")
            return
        self.last_camera_time = time.monotonic()

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
            points.append((
                cx + radius * math.cos(angle),
                cy + radius * math.sin(angle),
            ))
        return points

    def text(self, draw, value, position, size, color=TEXT):
        draw.text(position, str(value), font=self.font(size), fill=color)

    def text_width(self, draw, value, size):
        box = draw.textbbox((0, 0), str(value), font=self.font(size))
        return box[2] - box[0]

    @staticmethod
    def rounded_card(
            draw, bounds, radius=18, fill=SURFACE, outline=BORDER, width=1):
        draw.rounded_rectangle(
            bounds, radius=radius, fill=fill, outline=outline, width=width)

    def draw_badge(self, draw, text, right, top, color, size):
        padding_x = max(10, int(size * 0.7))
        height = max(28, int(size * 1.75))
        text_w = self.text_width(draw, text, size)
        left = right - text_w - padding_x * 2
        draw.rounded_rectangle(
            (left, top, right, top + height), radius=height // 2,
            fill=tuple(max(0, int(channel * 0.18)) for channel in color),
            outline=color, width=1)
        dot_r = max(3, int(size * 0.22))
        dot_x = left + padding_x
        dot_y = top + height // 2
        draw.ellipse(
            (dot_x - dot_r, dot_y - dot_r, dot_x + dot_r, dot_y + dot_r),
            fill=color)
        self.text(
            draw, text,
            (dot_x + dot_r + 7, top + (height - size) * 0.35), size, color)
        return left

    def draw_map(self, draw, arena):
        left, top, width, height = arena
        right, bottom = left + width, top + height
        draw.rectangle((left, top, right, bottom), fill=(14, 25, 31))
        for x in range(0, int(self.field_w) + 1, 4):
            major = x % 12 == 0
            draw.line(
                (self.field_to_screen((x, 0), arena),
                 self.field_to_screen((x, self.field_h), arena)),
                fill=(49, 66, 74) if major else (34, 49, 57), width=1)
        for y in range(0, int(self.field_h) + 1, 4):
            major = y % 12 == 0
            draw.line(
                (self.field_to_screen((0, y), arena),
                 self.field_to_screen((self.field_w, y), arena)),
                fill=(49, 66, 74) if major else (34, 49, 57), width=1)

        for index, obstacle in enumerate(self.obstacles, start=1):
            xmin, ymin, xmax, ymax = obstacle
            upper_left = self.field_to_screen((xmin, ymax), arena)
            lower_right = self.field_to_screen((xmax, ymin), arena)
            draw.rounded_rectangle(
                (upper_left, lower_right), radius=max(3, width // 180),
                fill=(72, 83, 87), outline=(114, 128, 132), width=2)
            label = f"街区 {index}"
            label_size = max(12, int(height * 0.025))
            label_w = self.text_width(draw, label, label_size)
            self.text(draw, label,
                      ((upper_left[0] + lower_right[0] - label_w) / 2,
                       (upper_left[1] + lower_right[1] - label_size) / 2),
                      label_size, (197, 207, 208))

        takeoff_upper_left = self.field_to_screen(
            (self.takeoff_zone[0], self.takeoff_zone[3]), arena)
        takeoff_lower_right = self.field_to_screen(
            (self.takeoff_zone[2], self.takeoff_zone[1]), arena)
        draw.rectangle(
            (takeoff_upper_left, takeoff_lower_right), fill=(16, 31, 37),
            outline=(73, 111, 122), width=2)
        self.text(
            draw, "起降区",
            (takeoff_upper_left[0] + 5, takeoff_upper_left[1] + 3),
            height * 0.025, TEXT_MUTED)

        parking_upper_left = self.field_to_screen(
            (self.parking_zone[0], self.parking_zone[3]), arena)
        parking_lower_right = self.field_to_screen(
            (self.parking_zone[2], self.parking_zone[1]), arena)
        draw.rectangle(
            (parking_upper_left, parking_lower_right), fill=(102, 34, 32),
            outline=(188, 68, 61), width=2)

        home_px = self.field_to_screen(self.home, arena)
        radius = max(7, width // 90)
        draw.ellipse((home_px[0] - radius - 3, home_px[1] - radius - 3,
                      home_px[0] + radius + 3, home_px[1] + radius + 3),
                     outline=(117, 47, 44), width=2)
        draw.ellipse((home_px[0] - radius, home_px[1] - radius,
                      home_px[0] + radius, home_px[1] + radius), fill=RED)
        self.text(draw, "车", (home_px[0] + 9, home_px[1] - 15), height * 0.03,
                  (255, 139, 128))

        if len(self.track) >= 2:
            draw.line(
                [self.field_to_screen(point, arena) for point in self.track],
                fill=CYAN, width=max(2, width // 260), joint="curve")
        if self.telemetry:
            drone_px = self.field_to_screen(self.telemetry[:2], arena)
            radius = max(8, width // 75)
            draw.ellipse((drone_px[0] - radius - 5, drone_px[1] - radius - 5,
                          drone_px[0] + radius + 5, drone_px[1] + radius + 5),
                         outline=(128, 103, 33), width=2)
            draw.ellipse((drone_px[0] - radius, drone_px[1] - radius,
                          drone_px[0] + radius, drone_px[1] + radius),
                         fill=AMBER, outline=(15, 15, 15), width=2)

        if self.fire_point:
            fire_px = self.field_to_screen(self.fire_point, arena)
            draw.polygon(self.star_points(fire_px, 15, 7), fill=RED)
            fire_label = (
                f"火源 ({self.fire_point[0]:.1f}, "
                f"{self.fire_point[1]:.1f})")
            fire_label_size = height * 0.027
            fire_label_w = self.text_width(
                draw, fire_label, fire_label_size)
            fire_label_x = min(
                fire_px[0] + 17, right - fire_label_w - 6)
            fire_label_x = max(left + 6, fire_label_x)
            self.text(
                draw, fire_label, (fire_label_x, fire_px[1] - 18),
                fire_label_size, (255, 137, 120))

        draw.rectangle(
            (left, top, right, bottom), outline=(84, 105, 114), width=2)

    def draw_panel(self, draw, panel):
        left, top, width, height = panel
        self.rounded_card(draw, (left, top, left + width, top + height))
        padding = max(16, int(width * 0.055))
        x = left + padding
        inner_w = width - padding * 2
        title_size = max(20, min(30, int(width * 0.068)))
        label_size = max(13, int(title_size * 0.55))
        value_size = max(18, int(title_size * 0.78))
        y = top + padding
        self.text(draw, "飞行遥测", (x, y), title_size, TEXT)

        elapsed = (math.inf if self.last_telemetry_time is None
                   else time.monotonic() - self.last_telemetry_time)
        if elapsed <= self.link_timeout:
            link_text, link_color = "链路正常", GREEN
        elif math.isinf(elapsed):
            link_text, link_color = "等待数据", RED
        else:
            link_text, link_color = f"离线 {elapsed:.1f}s", RED
        self.draw_badge(
            draw, link_text, left + width - padding, y,
            link_color, label_size)
        y += title_size * 1.65

        card_gap = max(9, int(width * 0.025))
        card_h = max(64, int(height * 0.13))
        half_w = (inner_w - card_gap) / 2
        if self.telemetry:
            px, py, distance, altitude, phase_value, _ = self.telemetry
            phase = PHASES.get(int(phase_value), f"未知({int(phase_value)})")
            metrics = (
                ("坐标 X / Y", f"{px:.1f} / {py:.1f}", "dm", CYAN),
                ("飞行高度", f"{altitude:.1f}", "dm", AMBER),
                ("累计里程", f"{distance:.1f}", "dm", TEXT),
                ("任务阶段", phase, "", GREEN),
            )
        else:
            metrics = (
                ("坐标 X / Y", "-- / --", "dm", TEXT_MUTED),
                ("飞行高度", "--", "dm", TEXT_MUTED),
                ("累计里程", "--", "dm", TEXT_MUTED),
                ("任务阶段", "--", "", TEXT_MUTED),
            )
        for index, (label, value, unit, color) in enumerate(metrics):
            row, column = divmod(index, 2)
            card_left = x + column * (half_w + card_gap)
            card_top = y + row * (card_h + card_gap)
            self.rounded_card(
                draw,
                (card_left, card_top, card_left + half_w, card_top + card_h),
                radius=12, fill=SURFACE_ALT, outline=(43, 58, 68))
            self.text(
                draw, label, (card_left + 13, card_top + 10),
                label_size, TEXT_MUTED)
            value_y = card_top + card_h - value_size - 15
            self.text(
                draw, value, (card_left + 13, value_y), value_size, color)
            if unit:
                value_w = self.text_width(draw, value, value_size)
                self.text(
                    draw, unit,
                    (card_left + 18 + value_w,
                     value_y + value_size * 0.25),
                    label_size, TEXT_MUTED)
        y += card_h * 2 + card_gap * 2 + 4

        status_label = CAR_STATUSES.get(self.car_status, self.car_status)
        status_color = RED if self.car_status.startswith("failed:") else GREEN
        self.text(draw, "消防车任务", (x, y), label_size, TEXT_MUTED)
        self.text(
            draw, status_label, (x, y + label_size * 1.45),
            value_size, status_color)
        y += label_size * 1.45 + value_size * 1.6

        draw.line((x, y, x + inner_w, y), fill=BORDER, width=1)
        y += max(12, int(height * 0.025))
        fire = (
            f"火源  ({self.fire_point[0]:.1f}, "
            f"{self.fire_point[1]:.1f}) dm"
            if self.fire_point else "火源  --")
        self.text(draw, fire, (x, y), label_size * 1.08,
                  (255, 137, 120) if self.fire_point else TEXT_MUTED)

        button_h = max(48, int(height * 0.09))
        button_top = top + height - padding - button_h
        button_gap = max(9, int(width * 0.025))
        button_w = (inner_w - button_gap) / 2
        self.start_button_rect = (
            x, button_top, x + button_w, button_top + button_h)
        self.reset_button_rect = (
            x + button_w + button_gap, button_top,
            x + inner_w, button_top + button_h)
        showing_feedback = time.monotonic() < self.start_feedback_until
        draw.rounded_rectangle(
            self.start_button_rect,
            radius=button_h // 2,
            fill=(28, 112, 72) if showing_feedback else (21, 91, 107),
            outline=GREEN if showing_feedback else CYAN, width=2)
        prompt = "已发送" if showing_feedback else "启动无人机"
        prompt_size = max(14, int(title_size * 0.56))
        prompt_w = self.text_width(draw, prompt, prompt_size)
        self.text(draw, prompt,
                  (x + (button_w - prompt_w) / 2,
                   button_top + (button_h - prompt_size) * 0.36),
                  prompt_size, TEXT)

        reset_feedback = time.monotonic() < self.reset_feedback_until
        draw.rounded_rectangle(
            self.reset_button_rect,
            radius=button_h // 2,
            fill=(112, 72, 28) if reset_feedback else (77, 55, 37),
            outline=AMBER, width=2)
        reset_prompt = "已复位" if reset_feedback else "复位消防车"
        reset_w = self.text_width(draw, reset_prompt, prompt_size)
        self.text(
            draw, reset_prompt,
            (x + button_w + button_gap + (button_w - reset_w) / 2,
             button_top + (button_h - prompt_size) * 0.36),
            prompt_size, TEXT)

    def trigger_start(self):
        now = time.monotonic()
        if now < self.start_feedback_until:
            return
        self.start_publisher.publish(Empty())
        self.start_feedback_until = now + self.start_cooldown_s

    def trigger_reset(self):
        now = time.monotonic()
        if now < self.reset_feedback_until:
            return
        self.reset_publisher.publish(Empty())
        self.reset_feedback_until = now + self.start_cooldown_s

    def on_mouse(self, event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.close_button_rect is not None:
            left, top, right, bottom = self.close_button_rect
            if left <= x <= right and top <= y <= bottom:
                self.exit_requested = True
                return
        if self.map_tab_rect is not None:
            left, top, right, bottom = self.map_tab_rect
            if left <= x <= right and top <= y <= bottom:
                self.active_tab = "map"
                return
        if self.camera_tab_rect is not None:
            left, top, right, bottom = self.camera_tab_rect
            if left <= x <= right and top <= y <= bottom:
                self.active_tab = "camera"
                return
        if self.start_button_rect is not None:
            left, top, right, bottom = self.start_button_rect
            if left <= x <= right and top <= y <= bottom:
                self.trigger_start()
                return
        if self.reset_button_rect is not None:
            left, top, right, bottom = self.reset_button_rect
            if left <= x <= right and top <= y <= bottom:
                self.trigger_reset()

    def draw_header(self, draw, margin, header_h):
        icon_r = max(13, int(header_h * 0.27))
        icon_x = margin + icon_r
        icon_y = margin + header_h / 2
        draw.ellipse((icon_x - icon_r, icon_y - icon_r,
                      icon_x + icon_r, icon_y + icon_r), fill=RED)
        draw.polygon(
            self.star_points(
                (icon_x, icon_y), icon_r * 0.58, icon_r * 0.27),
            fill=(255, 235, 225))
        title_size = max(22, int(header_h * 0.42))
        self.text(draw, "空地协同智能消防", (icon_x + icon_r + 12, margin + 1),
                  title_size, TEXT)
        self.text(
            draw, "CAR  ·  实时任务态势",
            (icon_x + icon_r + 13, margin + title_size * 1.18),
            max(12, int(title_size * 0.5)), TEXT_MUTED)

        tab_h = max(36, int(header_h * 0.72))
        tab_w = max(104, int(tab_h * 2.8))
        tabs_left = round((self.width - tab_w * 2) / 2)
        tabs_top = round(margin + (header_h - tab_h) / 2)
        tabs_right = tabs_left + tab_w * 2
        draw.rounded_rectangle(
            (tabs_left, tabs_top, tabs_right, tabs_top + tab_h),
            radius=tab_h // 2, fill=SURFACE, outline=BORDER, width=1)
        self.map_tab_rect = (
            tabs_left, tabs_top, tabs_left + tab_w, tabs_top + tab_h)
        self.camera_tab_rect = (
            tabs_left + tab_w, tabs_top, tabs_right, tabs_top + tab_h)
        tab_size = max(13, int(tab_h * 0.38))
        for label, bounds, selected in (
                ("任务态势", self.map_tab_rect, self.active_tab == "map"),
                ("摄像头", self.camera_tab_rect,
                 self.active_tab == "camera")):
            if selected:
                draw.rounded_rectangle(
                    bounds, radius=tab_h // 2,
                    fill=(21, 91, 107), outline=CYAN, width=2)
            label_w = self.text_width(draw, label, tab_size)
            self.text(
                draw, label,
                (bounds[0] + (tab_w - label_w) / 2,
                 bounds[1] + (tab_h - tab_size) * 0.36),
                tab_size, TEXT if selected else TEXT_MUTED)

        button_size = max(36, int(header_h * 0.72))
        button_top = round(margin + (header_h - button_size) / 2)
        button_right = self.width - margin
        button_left = button_right - button_size
        self.close_button_rect = (
            button_left, button_top, button_right, button_top + button_size)
        draw.rounded_rectangle(
            self.close_button_rect, radius=max(8, button_size // 4),
            fill=SURFACE_ALT, outline=BORDER, width=1)
        cross_margin = max(10, int(button_size * 0.29))
        draw.line(
            (button_left + cross_margin, button_top + cross_margin,
             button_right - cross_margin,
             button_top + button_size - cross_margin),
            fill=TEXT, width=max(2, button_size // 14))
        draw.line(
            (button_right - cross_margin, button_top + cross_margin,
             button_left + cross_margin,
             button_top + button_size - cross_margin),
            fill=TEXT, width=max(2, button_size // 14))

        hint = "ESC"
        hint_size = max(12, int(title_size * 0.52))
        hint_w = self.text_width(draw, hint, hint_size)
        self.text(
            draw, hint,
            (button_left - max(12, margin) - hint_w,
             margin + (header_h - hint_size) / 2),
            hint_size, TEXT_MUTED)

    def draw_camera_card(self, image, draw, card):
        left, top, width, height = card
        right, bottom = left + width, top + height
        self.rounded_card(draw, (left, top, right, bottom))
        padding = max(12, int(min(width, height) * 0.035))
        title_size = max(17, min(26, int(min(width, height) * 0.055)))
        self.text(
            draw, "摄像头画面", (left + padding, top + padding),
            title_size, TEXT)

        elapsed = (
            math.inf if self.last_camera_time is None
            else time.monotonic() - self.last_camera_time)
        if elapsed <= self.camera_timeout:
            status_text, status_color = "实时", GREEN
        elif self.camera_frame is None:
            status_text, status_color = "等待", AMBER
        else:
            status_text, status_color = "已断开", RED
        status_size = max(12, int(title_size * 0.62))
        self.draw_badge(
            draw, status_text, right - padding, top + padding,
            status_color, status_size)

        viewport_top = top + padding + max(
            title_size * 1.9, status_size * 2.0)
        viewport = (
            round(left + padding), round(viewport_top),
            round(right - padding), round(bottom - padding))
        viewport_w = max(1, viewport[2] - viewport[0])
        viewport_h = max(1, viewport[3] - viewport[1])
        draw.rounded_rectangle(
            viewport, radius=max(8, padding // 2),
            fill=(5, 9, 12), outline=(43, 58, 68), width=1)

        if self.camera_frame is None:
            prompt = "等待摄像头画面"
            prompt_size = max(13, int(title_size * 0.72))
            prompt_w = self.text_width(draw, prompt, prompt_size)
            self.text(
                draw, prompt,
                (viewport[0] + (viewport_w - prompt_w) / 2,
                 viewport[1] + (viewport_h - prompt_size) / 2),
                prompt_size, TEXT_MUTED)
            return

        rgb_frame = cv2.cvtColor(self.camera_frame, cv2.COLOR_BGR2RGB)
        preview = Image.fromarray(rgb_frame)
        scale = min(
            viewport_w / preview.width, viewport_h / preview.height)
        preview_size = (
            max(1, round(preview.width * scale)),
            max(1, round(preview.height * scale)))
        resampling = getattr(Image, "Resampling", Image)
        preview = preview.resize(preview_size, resampling.LANCZOS)
        preview_left = viewport[0] + (viewport_w - preview.width) // 2
        preview_top = viewport[1] + (viewport_h - preview.height) // 2
        image.paste(preview, (preview_left, preview_top))
        draw.rectangle(viewport, outline=(43, 58, 68), width=1)

    def draw_map_card(self, draw, card):
        left, top, width, height = card
        self.rounded_card(draw, (left, top, left + width, top + height))
        padding = max(14, int(min(width, height) * 0.028))
        title_size = max(18, int(min(width, height) * 0.04))
        self.text(
            draw, "场地态势", (left + padding, top + padding),
            title_size, TEXT)
        subtitle = "原点左下  ·  单位 dm"
        subtitle_size = max(12, int(title_size * 0.58))
        subtitle_w = self.text_width(draw, subtitle, subtitle_size)
        self.text(draw, subtitle, (left + width - padding - subtitle_w,
                  top + padding + title_size * 0.3), subtitle_size, TEXT_MUTED)

        legend_h = max(25, int(title_size * 1.15))
        arena_top = top + padding + title_size * 1.65
        arena_bottom = top + height - padding - legend_h
        available_w = width - padding * 2
        available_h = max(100, arena_bottom - arena_top)
        arena_w = min(available_w, available_h * self.field_w / self.field_h)
        arena_h = arena_w * self.field_h / self.field_w
        arena_left = left + (width - arena_w) / 2
        arena_top += (available_h - arena_h) / 2
        arena = (
            round(arena_left), round(arena_top),
            round(arena_w), round(arena_h))
        self.draw_map(draw, arena)

        legend_y = top + height - padding - subtitle_size
        legend = ((CYAN, "航迹"), (AMBER, "无人机"), (RED, "火源 / 车"))
        cursor_x = left + padding
        for color, label in legend:
            dot_r = max(3, subtitle_size // 4)
            draw.ellipse((cursor_x, legend_y + 3, cursor_x + dot_r * 2,
                          legend_y + 3 + dot_r * 2), fill=color)
            cursor_x += dot_r * 2 + 6
            self.text(
                draw, label, (cursor_x, legend_y),
                subtitle_size, TEXT_MUTED)
            cursor_x += self.text_width(draw, label, subtitle_size) + 20

    def render(self):
        # XFCE may apply the saved HDMI mode after desktop autostart has
        # already begun.  Follow later XRandR changes so a temporary boot
        # resolution cannot leave a permanently undersized, white-bordered
        # frame.
        self._refresh_fullscreen_size()
        # In the Qt5 backend getWindowImageRect() reports the centered image
        # viewport, not the native full-screen window.  Reading it in
        # full-screen mode would shrink the next frame and recreate the white
        # border.  It remains useful for a user-resizable window.
        if not self.headless and not self.fullscreen:
            try:
                _, _, detected_w, detected_h = cv2.getWindowImageRect(
                    WINDOW_NAME)
                if detected_w > 100 and detected_h > 100:
                    self.width, self.height = detected_w, detected_h
            except cv2.error:
                pass
        image = Image.new("RGB", (self.width, self.height), BG)
        draw = ImageDraw.Draw(image)
        margin = max(12, int(min(self.width, self.height) * 0.022))
        gap = max(12, int(margin * 0.9))
        header_h = max(48, int(self.height * 0.075))
        self.draw_header(draw, margin, header_h)
        content_top = margin + header_h + gap
        content_h = self.height - content_top - margin
        panel_width = max(340, min(460, int(self.width * 0.345)))
        if self.width < 900:
            panel_width = max(300, int(self.width * 0.39))
        page_width = self.width - margin * 2 - gap - panel_width
        page_card = (margin, content_top, page_width, content_h)
        panel = (margin + page_width + gap, content_top, panel_width, content_h)
        if self.active_tab == "camera":
            self.draw_camera_card(image, draw, page_card)
        else:
            self.draw_map_card(draw, page_card)
        self.draw_panel(draw, panel)
        if not self.headless:
            cv2.imshow(
                WINDOW_NAME,
                cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR))
        return image

    def process_events(self):
        if self.headless:
            return not self.exit_requested
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or self.exit_requested:
            return False
        if key in (13, 32):
            self.trigger_start()
        if key in (ord("r"), ord("R")):
            self.trigger_reset()
        if key == 9:
            self.active_tab = (
                "camera" if self.active_tab == "map" else "map")
        # Xfwm temporarily unmaps full-screen windows while applying a new
        # display mode.  Treating that transition as a close event makes the
        # autostart dashboard exit just as HDMI reaches its final resolution.
        if self.fullscreen:
            return True
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
