"""Make the workspace's pure-Python packages importable without a build (unit tests run from a bare
checkout)."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for rel in ('simulation', 'src/fire_resq_description', 'src/fire_resq_navigation', 'src/fire_resq_perception', 'src/fire_resq_world_model', 'src/fire_resq_cognition'):
    sys.path.insert(0, str(ROOT / rel))
