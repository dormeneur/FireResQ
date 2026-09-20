"""Make the simulation package importable without a build (unit tests run from a bare checkout)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'simulation'))
