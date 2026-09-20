"""MVP detector: OpenCV HSV thresholds + connected components.

Why this and not YOLO: no pretrained model knows "small metal rescue block", and the simulated
victims/fire are deliberately distinct in hue (docs/Implementation_Plan.md, Phase 3 colour
contract: every non-target pixel has saturation <= 18). The hard part of this project's perception
is SPATIAL estimation, not classification.

No morphological opening on purpose: a fire's apex is only a few pixels wide and opening would
erode the top edge, which the known-height estimator depends on.

TODO(phase-5b / hardware): YoloDetector behind the same interface if real lighting defeats hue
    thresholds (colour blobs are brittle on real cameras).
TODO(future): calibrated confidence / uncertainty (this score is size-based, NOT a probability).
TODO(future): obstacle and safe-zone detection (obstacles come from the scan / costmaps for now).
"""
from __future__ import annotations

import math
from typing import List, Sequence

import cv2
import numpy as np

from ..config import ClassConfig
from ..types import Detection2D
from .base import Detector


class ColorBlobDetector(Detector):
    name = 'color_blob'

    def __init__(self, classes: Sequence[ClassConfig], max_per_class: int = 10):
        if not classes:
            raise ValueError('ColorBlobDetector needs at least one class')
        self.classes = list(classes)
        self.max_per_class = max_per_class

    def detect(self, rgb: np.ndarray) -> List[Detection2D]:
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError(f'expected an H x W x 3 RGB image, got shape {rgb.shape}')
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        height, width = rgb.shape[:2]
        out: List[Detection2D] = []
        for cfg in self.classes:
            mask = np.zeros((height, width), np.uint8)
            for lo, hi in cfg.hsv_ranges:
                mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
            found = []
            for i in range(1, n):
                area = int(stats[i, cv2.CC_STAT_AREA])
                if area < cfg.min_area_px:
                    continue
                x, y = int(stats[i, cv2.CC_STAT_LEFT]), int(stats[i, cv2.CC_STAT_TOP])
                w, h = int(stats[i, cv2.CC_STAT_WIDTH]), int(stats[i, cv2.CC_STAT_HEIGHT])
                truncated = x == 0 or y == 0 or x + w >= width or y + h >= height
                conf = 1.0 - math.exp(-area / cfg.area_ref_px)
                found.append(Detection2D(
                    class_id=cfg.name, confidence=conf * (0.5 if truncated else 1.0), x=x, y=y, w=w, h=h,
                    cx=float(cents[i][0]) + 0.5, cy=float(cents[i][1]) + 0.5,      # pixel index -> continuous
                    area=float(area), truncated=truncated, mask=(labels[y:y + h, x:x + w] == i)))
            found.sort(key=lambda d: d.area, reverse=True)
            out.extend(found[:self.max_per_class])
        out.sort(key=lambda d: d.area, reverse=True)
        return out
