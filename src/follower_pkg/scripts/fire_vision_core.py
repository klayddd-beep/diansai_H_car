#!/usr/bin/env python3
"""OpenCV-only red target detector shared by the ROS node and unit tests."""

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass
class DetectionResult:
    """Describe one processed frame's selected target."""

    detected: bool
    center_x_px: float = 0.0
    center_y_px: float = 0.0
    error_x_norm: float = 0.0
    error_y_norm: float = 0.0
    area_ratio: float = 0.0


def orient_frame(
    frame: np.ndarray,
    rotate_cw_deg: int,
    flip_horizontal: bool,
    flip_vertical: bool,
) -> np.ndarray:
    """Apply installation orientation before coordinate calculations."""
    if rotate_cw_deg == 0:
        oriented = frame
    elif rotate_cw_deg == 90:
        oriented = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif rotate_cw_deg == 180:
        oriented = cv2.rotate(frame, cv2.ROTATE_180)
    elif rotate_cw_deg == 270:
        oriented = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    else:
        raise ValueError("rotate_cw_deg must be one of 0, 90, 180, 270")

    if flip_horizontal and flip_vertical:
        return cv2.flip(oriented, -1)
    if flip_horizontal:
        return cv2.flip(oriented, 1)
    if flip_vertical:
        return cv2.flip(oriented, 0)
    return oriented


class RedTargetDetector:
    """Detect a red target without assuming a fixed shape."""

    def __init__(
        self,
        *,
        hue_low_1: int = 0,
        hue_high_1: int = 12,
        hue_low_2: int = 168,
        hue_high_2: int = 179,
        saturation_min: int = 70,
        value_min: int = 45,
        red_min: int = 65,
        red_over_green_min: int = 25,
        red_over_blue_min: int = 20,
        min_area_ratio: float = 0.0002,
        max_area_ratio: float = 0.20,
        morphology_open_px: int = 3,
        morphology_close_px: int = 7,
        roi: Sequence[float] = (0.0, 0.0, 1.0, 1.0),
        aim_x_ratio: float = 0.5,
        aim_y_ratio: float = 0.5,
        track_memory_frames: int = 5,
    ) -> None:
        """Store and validate segmentation and tracking parameters."""
        self.hue_low_1 = int(hue_low_1)
        self.hue_high_1 = int(hue_high_1)
        self.hue_low_2 = int(hue_low_2)
        self.hue_high_2 = int(hue_high_2)
        self.saturation_min = int(saturation_min)
        self.value_min = int(value_min)
        self.red_min = int(red_min)
        self.red_over_green_min = int(red_over_green_min)
        self.red_over_blue_min = int(red_over_blue_min)
        self.min_area_ratio = float(min_area_ratio)
        self.max_area_ratio = float(max_area_ratio)
        self.open_px = self._kernel_size(morphology_open_px)
        self.close_px = self._kernel_size(morphology_close_px)
        self.roi = tuple(float(value) for value in roi)
        self.aim_x_ratio = float(aim_x_ratio)
        self.aim_y_ratio = float(aim_y_ratio)
        self.track_memory_frames = max(0, int(track_memory_frames))
        self._last_center: Optional[Tuple[float, float]] = None
        self._missed_frames = 0
        self._validate()

    @staticmethod
    def _kernel_size(value: int) -> int:
        value = max(0, int(value))
        if value > 0 and value % 2 == 0:
            value += 1
        return value

    def _validate(self) -> None:
        byte_values = (
            self.hue_low_1,
            self.hue_high_1,
            self.hue_low_2,
            self.hue_high_2,
            self.saturation_min,
            self.value_min,
            self.red_min,
        )
        if not all(0 <= value <= 255 for value in byte_values):
            raise ValueError("HSV and channel thresholds must be in [0, 255]")
        if not (
            0 <= self.hue_low_1 <= self.hue_high_1 <= 179
            and 0 <= self.hue_low_2 <= self.hue_high_2 <= 179
        ):
            raise ValueError(
                "OpenCV hue ranges must be ordered and in [0, 179]"
            )
        if len(self.roi) != 4:
            raise ValueError("roi must be [xmin, ymin, xmax, ymax] ratios")
        if not (
            0.0 <= self.roi[0] < self.roi[2] <= 1.0
            and 0.0 <= self.roi[1] < self.roi[3] <= 1.0
        ):
            raise ValueError("roi ratios must form a non-empty box in [0, 1]")
        if not (
            0.0 <= self.aim_x_ratio <= 1.0
            and 0.0 <= self.aim_y_ratio <= 1.0
        ):
            raise ValueError("aim ratios must be in [0, 1]")
        if not (0.0 < self.min_area_ratio < self.max_area_ratio <= 1.0):
            raise ValueError("area ratio limits are invalid")

    def _make_mask(self, frame: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_1 = cv2.inRange(
            hsv,
            (self.hue_low_1, self.saturation_min, self.value_min),
            (self.hue_high_1, 255, 255),
        )
        mask_2 = cv2.inRange(
            hsv,
            (self.hue_low_2, self.saturation_min, self.value_min),
            (self.hue_high_2, 255, 255),
        )
        red_mask = cv2.bitwise_or(mask_1, mask_2)

        blue, green, red = cv2.split(frame)
        red_i16 = red.astype(np.int16)
        dominance = (
            (red >= self.red_min)
            & ((red_i16 - green.astype(np.int16)) >= self.red_over_green_min)
            & ((red_i16 - blue.astype(np.int16)) >= self.red_over_blue_min)
        )
        red_mask = cv2.bitwise_and(
            red_mask, np.where(dominance, 255, 0).astype(np.uint8)
        )

        height, width = frame.shape[:2]
        x0 = int(round(self.roi[0] * width))
        y0 = int(round(self.roi[1] * height))
        x1 = int(round(self.roi[2] * width))
        y1 = int(round(self.roi[3] * height))
        roi_mask = np.zeros_like(red_mask)
        roi_mask[y0:y1, x0:x1] = 255
        red_mask = cv2.bitwise_and(red_mask, roi_mask)

        if self.open_px:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self.open_px, self.open_px)
            )
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        if self.close_px:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self.close_px, self.close_px)
            )
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        return red_mask

    def detect(
        self, frame: np.ndarray
    ) -> Tuple[DetectionResult, np.ndarray, Optional[np.ndarray]]:
        """Return the selected red target, binary mask, and contour."""
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("frame must be a non-empty BGR image")

        mask = self._make_mask(frame)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        height, width = frame.shape[:2]
        image_area = float(width * height)
        candidates: List[Tuple[np.ndarray, float, float, float, float]] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            ratio = area / image_area
            if ratio < self.min_area_ratio or ratio > self.max_area_ratio:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] <= 0.0:
                continue
            center_x = moments["m10"] / moments["m00"]
            center_y = moments["m01"] / moments["m00"]
            candidates.append((contour, area, ratio, center_x, center_y))

        if not candidates:
            self._missed_frames += 1
            if self._missed_frames > self.track_memory_frames:
                self._last_center = None
            return DetectionResult(False), mask, None

        if self._last_center is None:
            chosen = max(candidates, key=lambda item: item[1])
        else:
            last_x, last_y = self._last_center
            chosen = min(
                candidates,
                key=lambda item: (
                    (item[3] - last_x) ** 2
                    + (item[4] - last_y) ** 2
                ),
            )

        contour, _, ratio, center_x, center_y = chosen
        self._last_center = (center_x, center_y)
        self._missed_frames = 0
        aim_x = self.aim_x_ratio * width
        aim_y = self.aim_y_ratio * height
        result = DetectionResult(
            detected=True,
            center_x_px=center_x,
            center_y_px=center_y,
            error_x_norm=(center_x - aim_x) / max(width / 2.0, 1.0),
            error_y_norm=(center_y - aim_y) / max(height / 2.0, 1.0),
            area_ratio=ratio,
        )
        return result, mask, contour

    def annotate(
        self,
        frame: np.ndarray,
        result: DetectionResult,
        contour: Optional[np.ndarray],
    ) -> np.ndarray:
        """Draw the configured aim point and current detection."""
        output = frame.copy()
        height, width = output.shape[:2]
        aim = (
            int(round(self.aim_x_ratio * width)),
            int(round(self.aim_y_ratio * height)),
        )
        cv2.drawMarker(
            output, aim, (0, 255, 255), cv2.MARKER_CROSS, 24, 2, cv2.LINE_AA
        )
        if contour is not None:
            cv2.drawContours(output, [contour], -1, (0, 255, 0), 2)
        if result.detected:
            center = (
                int(round(result.center_x_px)),
                int(round(result.center_y_px)),
            )
            cv2.circle(output, center, 6, (255, 255, 0), -1)
            cv2.line(output, aim, center, (255, 255, 0), 2)
            label = (
                f"ex={result.error_x_norm:+.3f} "
                f"ey={result.error_y_norm:+.3f} area={result.area_ratio:.4f}"
            )
        else:
            label = "target lost"
        cv2.putText(
            output,
            label,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0) if result.detected else (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return output
