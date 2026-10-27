"""Camera calibration file -> the numbers a CameraInfo message carries. No ROS, so it is unit-testable.

The file is the standard ROS calibration YAML, i.e. exactly what `camera_calibration`'s COMMIT button writes, so
a real calibration replaces the nominal one by swapping a file (docs/HOW_TO_MAKE_THE_COMPLETE_ROBOT.md).
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List

import yaml


@dataclass(frozen=True)
class Calibration:
    width: int
    height: int
    distortion_model: str
    d: List[float]
    k: List[float]          # 3x3 row-major
    r: List[float]          # 3x3 row-major
    p: List[float]          # 3x4 row-major

    def scaled_to(self, width: int, height: int) -> 'Calibration':
        """The same lens at another resolution OF THE SAME ASPECT RATIO (the sensor is binned/scaled, not cropped).

        Refuses a different aspect ratio: the C270's 16:9 modes crop the sensor, and scaling intrinsics across a
        crop silently misplaces every detection.
        """
        if (width, height) == (self.width, self.height):
            return self
        if width * self.height != height * self.width:
            raise ValueError(f'calibration is {self.width}x{self.height}; cannot scale it to {width}x{height} '
                             f'(different aspect ratio = a different sensor crop). Calibrate at that resolution.')
        s = width / self.width
        k, p = list(self.k), list(self.p)
        for i in (0, 2, 4, 5):                                   # fx, cx, fy, cy
            k[i] *= s
        for i in (0, 2, 3, 5, 6, 7):                             # fx, cx, Tx, fy, cy, Ty
            p[i] *= s
        return Calibration(width, height, self.distortion_model, list(self.d), k, list(self.r), p)


def _matrix(doc: dict, key: str, n: int) -> List[float]:
    try:
        data = [float(v) for v in doc[key]['data']]
    except (KeyError, TypeError) as e:
        raise ValueError(f"calibration file has no '{key}.data'") from e
    if len(data) != n:
        raise ValueError(f"'{key}.data' has {len(data)} values, expected {n}")
    return data


def load_calibration(path) -> Calibration:
    doc = yaml.safe_load(Path(path).read_text())
    if not isinstance(doc, dict):
        raise ValueError(f'{path}: not a calibration file')
    try:
        width, height = int(doc['image_width']), int(doc['image_height'])
    except KeyError as e:
        raise ValueError(f'{path}: missing {e}') from e
    k = _matrix(doc, 'camera_matrix', 9)
    if k[0] <= 0.0 or k[4] <= 0.0:
        raise ValueError(f'{path}: focal lengths must be positive')
    d = [float(v) for v in (doc.get('distortion_coefficients') or {}).get('data', [])]
    r = _matrix(doc, 'rectification_matrix', 9) if 'rectification_matrix' in doc else [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    p = _matrix(doc, 'projection_matrix', 12) if 'projection_matrix' in doc else [k[0], 0.0, k[2], 0.0, 0.0, k[4], k[5], 0.0, 0.0, 0.0, 1.0, 0.0]
    return Calibration(width, height, str(doc.get('distortion_model', 'plumb_bob')), d, k, r, p)
