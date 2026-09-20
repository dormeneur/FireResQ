"""Plain data types shared by the perception library. No ROS, no Gazebo, no camera driver."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


@dataclass(frozen=True)
class CameraModel:
    """Pinhole intrinsics. Pixel coordinates are CONTINUOUS: pixel index i covers [i, i+1), so the
    image centre of a W-wide image is at u = W/2 (which is what CameraInfo's cx means)."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_k(cls, k: Sequence[float], width: int, height: int) -> 'CameraModel':
        return cls(fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]), width=int(width), height=int(height))

    def ray(self, u: float, v: float) -> np.ndarray:
        """Direction through pixel (u, v) in the optical frame (z forward, x right, y down), z = 1."""
        return np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])


@dataclass(frozen=True)
class Transform:
    """Rigid transform: p_out = R @ p_in + t (maps points FROM the source frame TO the target frame)."""
    R: np.ndarray
    t: np.ndarray

    @classmethod
    def identity(cls) -> 'Transform':
        return cls(np.eye(3), np.zeros(3))

    @classmethod
    def from_quaternion(cls, x: float, y: float, z: float, w: float, translation: Sequence[float]) -> 'Transform':
        n = math.sqrt(x * x + y * y + z * z + w * w)
        x, y, z, w = x / n, y / n, z / n, w / n
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return cls(R, np.asarray(translation, dtype=float))

    def apply(self, p: np.ndarray) -> np.ndarray:
        return self.R @ np.asarray(p, dtype=float) + self.t

    def rotate(self, v: np.ndarray) -> np.ndarray:
        return self.R @ np.asarray(v, dtype=float)

    def inverse(self) -> 'Transform':
        return Transform(self.R.T, -self.R.T @ self.t)

    @property
    def yaw(self) -> float:
        return math.atan2(self.R[1, 0], self.R[0, 0])


@dataclass
class Detection2D:
    """A detection in the image: what it is, how sure, where in pixels. Camera-agnostic."""
    class_id: str
    confidence: float
    x: int                  # bounding box, top-left pixel index
    y: int
    w: int
    h: int
    cx: float               # blob centroid, continuous pixel coordinates
    cy: float
    area: float
    truncated: bool         # touches the image border: its extent (and any position from it) is biased
    mask: np.ndarray        # bool, cropped to the bounding box


@dataclass(frozen=True)
class SpatialEstimate:
    position: np.ndarray    # in base_link (z up, level with the robot)
    range_m: float          # horizontal distance from the camera
    method: str


@dataclass(frozen=True)
class DepthFrame:
    depth: np.ndarray       # H x W float32 metres, registered to the RGB image; invalid = 0 / nan / inf
    stamp: float


@dataclass
class Detection3D:
    """A 2D detection plus, when it could be computed and trusted, a position."""
    det: Detection2D
    position_base: Optional[np.ndarray] = None
    range_m: float = 0.0
    backend: str = ''
    valid: bool = False
    reason: str = ''        # why there is no position (diagnostics; never branched on above perception)
