"""Per-class perception configuration (colour + object geometry priors)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

Hsv = Tuple[int, int, int]


@dataclass(frozen=True)
class ClassConfig:
    """What one detectable class looks like and how big it is.

    Colour drives DETECTION; the geometry priors drive RGB-only SPATIAL estimation and the
    correction from a seen surface to the object's axis. They are facts about the OBJECTS
    (like a victim's height), never about where anything is.
    TODO(hardware): these must match the real victims / fire stand-in (Hardware.md TODOs).
    """
    name: str
    hsv_ranges: Tuple[Tuple[Hsv, Hsv], ...]     # inclusive (lo, hi) OpenCV HSV ranges; several allow hue wrap
    min_area_px: int = 30
    area_ref_px: float = 300.0                  # blob size at which the size score reaches 63 %
    top_height_m: float = 0.0                   # height of the blob's TOP above the floor
    bottom_height_m: float = 0.0                # height of the blob's lowest visible row above the floor
    axis_offset_m: float = 0.0                  # depth: seen surface -> vertical axis, at the blob centroid
    bottom_axis_offset_m: float = 0.0           # bottom anchor: front rim -> vertical axis


def class_config(name: str, hsv_lo: Sequence[int], hsv_hi: Sequence[int], **kw) -> ClassConfig:
    lo, hi = tuple(int(v) for v in hsv_lo), tuple(int(v) for v in hsv_hi)
    if len(lo) != 3 or len(hi) != 3:
        raise ValueError(f"class '{name}': hsv_lo/hsv_hi need 3 values each")
    return ClassConfig(name=name, hsv_ranges=((lo, hi),), **kw)


def by_name(configs: Sequence[ClassConfig]) -> Dict[str, ClassConfig]:
    return {c.name: c for c in configs}


_FIELDS = ('min_area_px', 'area_ref_px', 'top_height_m', 'bottom_height_m', 'axis_offset_m', 'bottom_axis_offset_m')


def classes_from_dict(names: Sequence[str], params: Dict[str, dict]) -> List[ClassConfig]:
    """Build class configs from the nested mapping the ROS parameter file provides:
    `{name: {hsv_lo, hsv_hi, min_area_px, ...}}`. Missing colour ranges are an error, not a default:
    a detector that silently matches everything is worse than one that will not start."""
    out = []
    for n in names:
        if n not in params:
            raise ValueError(f"class '{n}' is listed in class_names but has no settings")
        p = params[n]
        for key in ('hsv_lo', 'hsv_hi'):
            if key not in p:
                raise ValueError(f"class '{n}' is missing '{key}'")
        out.append(class_config(n, p['hsv_lo'], p['hsv_hi'], **{k: p[k] for k in _FIELDS if k in p}))
    return out
