import cv2
import numpy as np
from rclpy.qos import HistoryPolicy, ReliabilityPolicy

from fire_video_receiver_pkg.receiver_core import (
    LinkState,
    decode_jpeg,
    link_state,
)
from fire_video_receiver_pkg.video_receiver import make_video_qos


def test_decode_jpeg_returns_bgr_frame():
    source = np.zeros((24, 32, 3), dtype=np.uint8)
    source[:, :, 2] = 255
    ok, encoded = cv2.imencode(".jpg", source)
    assert ok

    decoded = decode_jpeg(encoded.tobytes())

    assert decoded is not None
    assert decoded.shape == source.shape
    assert decoded.dtype == np.uint8
    assert decoded[:, :, 2].mean() > 240


def test_decode_jpeg_drops_empty_or_invalid_data():
    assert decode_jpeg(b"") is None
    assert decode_jpeg(b"not a jpeg frame") is None


def test_link_state_uses_local_receive_time_and_strict_timeout():
    assert link_state(None, now=100.0, timeout_s=2.0) == LinkState.WAITING
    assert link_state(98.0, now=100.0, timeout_s=2.0) == LinkState.LIVE
    assert link_state(97.999, now=100.0, timeout_s=2.0) == LinkState.LOST


def test_video_qos_is_best_effort_keep_last_one():
    qos = make_video_qos()
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 1
