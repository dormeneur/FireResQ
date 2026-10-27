"""Wire format of firmware/base_controller (the two must change together). Pure: bytes and tuples only."""
from __future__ import annotations

from typing import Optional, Sequence, Tuple

PROTOCOL = 2
MOTORS = 4
MAX_POWER = 255
BAUD = 115200

Powers = Tuple[int, int, int, int]
STOP: Powers = (0, 0, 0, 0)


def format_drive(powers: Sequence[int]) -> bytes:
    """Signed power per shield terminal M1..M4. Refuses anything the firmware would reject, so a bug upstream
    is an error here rather than a silently ignored command."""
    if len(powers) != MOTORS or any(int(p) != p or abs(p) > MAX_POWER for p in powers):
        raise ValueError(f'need {MOTORS} integer powers within +-{MAX_POWER}, got {list(powers)}')
    return ('M ' + ' '.join(str(int(p)) for p in powers) + '\n').encode('ascii')


def parse_banner(line: str) -> Optional[int]:
    """'FIRERESQ_BASE 2 wd=400' -> 2; None for anything else (another sketch, line noise)."""
    parts = line.split()
    if len(parts) >= 2 and parts[0] == 'FIRERESQ_BASE' and parts[1].isdigit():
        return int(parts[1])
    return None


def parse_status(line: str) -> Optional[Powers]:
    """'S 120 -120 0 0' -> the powers the firmware is applying; None for anything else."""
    parts = line.split()
    if len(parts) != MOTORS + 1 or parts[0] != 'S':
        return None
    try:
        powers = tuple(int(p) for p in parts[1:])
    except ValueError:
        return None
    return powers if all(abs(p) <= MAX_POWER for p in powers) else None
