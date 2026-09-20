"""The detector interface. Anything that turns an RGB image into 2D detections implements this;
the OpenCV colour-blob detector is the MVP and a YOLO-family detector can replace it later without
touching spatial estimation or anything above perception."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List

import numpy as np

from ..types import Detection2D


class Detector(ABC):
    name = 'detector'

    @abstractmethod
    def detect(self, rgb: np.ndarray) -> List[Detection2D]:
        """rgb: H x W x 3 uint8, RGB order. Detections are sorted largest first."""
