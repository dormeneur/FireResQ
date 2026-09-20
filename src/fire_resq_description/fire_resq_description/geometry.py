"""Read the numeric properties from parameters.xacro."""
from __future__ import annotations

import re
from pathlib import Path
from typing import Dict, Optional


def find_parameters_xacro() -> Path:
    """The installed description if the workspace is sourced, otherwise the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory('fire_resq_description')) / 'urdf' / 'parameters.xacro'
        if p.is_file():
            return p
    except Exception:
        pass
    return Path(__file__).resolve().parents[1] / 'urdf' / 'parameters.xacro'


def read_properties(path: Optional[Path] = None) -> Dict[str, float]:
    """Every numeric `<xacro:property name=... value=...>`; comments are ignored."""
    text = Path(path or find_parameters_xacro()).read_text()
    text = re.sub(r'<!--.*?-->', '', text, flags=re.S)
    out: Dict[str, float] = {}
    for name, val in re.findall(r'<xacro:property\s+name="(\w+)"\s+value="([^"]+)"', text):
        try:
            out[name] = float(val)
        except ValueError:
            pass
    return out
