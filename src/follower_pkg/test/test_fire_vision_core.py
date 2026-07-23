"""Unit tests for fire target segmentation and image-coordinate conventions."""

import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from fire_vision_core import RedTargetDetector, orient_frame  # noqa: E402


def make_frame(center=None, color=(0, 0, 210), radius=22):
    """Create a pale background with an optional filled target."""
    frame = np.full((480, 640, 3), 225, dtype=np.uint8)
    if center is not None:
        cv2.circle(frame, center, radius, color, -1)
    return frame


def detect_at(center, color=(0, 0, 210)):
    """Detect one synthetic target at the requested location."""
    detector = RedTargetDetector()
    result, _, _ = detector.detect(make_frame(center, color=color))
    assert result.detected
    return result


def test_detects_printed_and_illuminated_red():
    """Both low-value printed red and bright LED-like red are accepted."""
    printed = detect_at((320, 240), color=(20, 30, 120))
    illuminated = detect_at((320, 240), color=(10, 10, 255))
    assert abs(printed.error_x_norm) < 0.01
    assert abs(printed.error_y_norm) < 0.01
    assert illuminated.area_ratio > 0.0


def test_rejects_gray_blue_and_small_noise():
    """Non-red regions and sub-threshold red speckles are rejected."""
    detector = RedTargetDetector()
    frame = make_frame()
    cv2.circle(frame, (180, 200), 30, (180, 180, 180), -1)
    cv2.circle(frame, (450, 200), 30, (255, 0, 0), -1)
    cv2.circle(frame, (320, 240), 2, (0, 0, 255), -1)
    result, _, _ = detector.detect(frame)
    assert not result.detected


def test_error_signs_match_vehicle_control_convention():
    """Normalized error signs match the documented vehicle directions."""
    assert detect_at((320, 100)).error_y_norm < 0.0  # high -> forward
    assert detect_at((320, 380)).error_y_norm > 0.0  # low -> reverse
    assert detect_at((120, 240)).error_x_norm < 0.0  # left -> positive yaw
    assert detect_at((520, 240)).error_x_norm > 0.0  # right -> negative yaw


def test_tracking_prefers_previous_candidate_over_new_larger_candidate():
    """A locked target wins over a newly appearing larger distractor."""
    detector = RedTargetDetector(min_area_ratio=0.0001)
    first = make_frame((140, 240), radius=24)
    result, _, _ = detector.detect(first)
    assert result.detected

    second = make_frame()
    cv2.circle(second, (145, 240), 12, (0, 0, 220), -1)
    cv2.circle(second, (500, 240), 38, (0, 0, 220), -1)
    result, _, _ = detector.detect(second)
    assert result.detected
    assert result.center_x_px < 200


def test_clockwise_rotation_and_flips_are_exact():
    """Orientation operations preserve exact pixel placement."""
    frame = np.zeros((2, 3, 3), dtype=np.uint8)
    frame[0, 0] = (1, 2, 3)
    rotated = orient_frame(frame, 90, False, False)
    assert rotated.shape == (3, 2, 3)
    assert np.array_equal(rotated[0, 1], (1, 2, 3))

    flipped = orient_frame(rotated, 0, True, True)
    assert np.array_equal(flipped[-1, 0], (1, 2, 3))


def test_calibrated_aim_pixel_changes_normalized_error():
    """Configured aim ratios define the zero-error pixel."""
    detector = RedTargetDetector(aim_x_ratio=0.4, aim_y_ratio=0.6)
    result, _, _ = detector.detect(make_frame((256, 288)))
    assert result.detected
    assert abs(result.error_x_norm) < 0.01
    assert abs(result.error_y_norm) < 0.01
