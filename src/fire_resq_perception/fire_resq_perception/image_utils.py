"""ROS Image -> numpy, for the encodings real and simulated cameras use. No cv_bridge needed."""
from __future__ import annotations

import numpy as np


def _rows(msg, dtype, channels: int) -> np.ndarray:
    itemsize = np.dtype(dtype).itemsize
    row = msg.width * channels * itemsize
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.step and msg.step != row:                           # drop row padding
        buf = buf.reshape(msg.height, msg.step)[:, :row]
    a = buf.reshape(msg.height, row).view(dtype)
    return a.reshape(msg.height, msg.width, channels) if channels > 1 else a.reshape(msg.height, msg.width)


def to_rgb(msg) -> np.ndarray:
    """rgb8 / bgr8 -> H x W x 3 uint8 in RGB order."""
    if msg.encoding == 'rgb8':
        return _rows(msg, np.uint8, 3).copy()
    if msg.encoding == 'bgr8':
        return _rows(msg, np.uint8, 3)[..., ::-1].copy()
    raise ValueError(f'unsupported colour encoding: {msg.encoding}')


def to_depth_metres(msg) -> np.ndarray:
    """32FC1 (metres, Gazebo) or 16UC1 (millimetres, RealSense) -> H x W float32 metres."""
    if msg.encoding == '32FC1':
        return _rows(msg, np.float32, 1).copy()
    if msg.encoding == '16UC1':
        return _rows(msg, np.uint16, 1).astype(np.float32) * 0.001
    raise ValueError(f'unsupported depth encoding: {msg.encoding}')
