"""ROS-independent helpers for decoding and monitoring the video stream."""

from enum import Enum

import cv2
import numpy as np


class LinkState(Enum):
    WAITING = "waiting"
    LIVE = "live"
    LOST = "lost"


def decode_jpeg(data):
    """Decode a ROS CompressedImage byte sequence into a BGR image."""
    if not data:
        return None
    encoded = np.frombuffer(data, dtype=np.uint8)
    try:
        return cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    except cv2.error:
        return None


def link_state(last_frame_at, now, timeout_s):
    """Return the local receive state without using the remote ROS stamp."""
    if last_frame_at is None:
        return LinkState.WAITING
    if now - last_frame_at > timeout_s:
        return LinkState.LOST
    return LinkState.LIVE
