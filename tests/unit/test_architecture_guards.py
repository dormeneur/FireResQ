"""Standing guards for the project's hard rules (CLAUDE.md). Cheap, and they fail loudly the day
someone breaks a rule - which is the point of writing them before there is anything to break."""
import re
from pathlib import Path

SRC = Path(__file__).resolve().parents[2] / 'src'
CODE = ('*.py', '*.xml', '*.launch.py', '*.yaml', '*.xacro')


def _files():
    for pat in CODE:
        yield from (p for p in SRC.rglob(pat) if '__pycache__' not in p.parts)


def _code(path: Path) -> str:
    """File text with comments and docstrings removed: a comment that NAMES the simulation
    package is documentation, not a dependency, and must not trip a guard."""
    text = path.read_text(errors='ignore')
    text = re.sub(r'<!--.*?-->', '', text, flags=re.S)           # xml / xacro comments
    text = re.sub(r'(\"\"\"|\'\'\').*?\1', '', text, flags=re.S)   # python docstrings
    return re.sub(r'(?m)#.*$', '', text)                         # python / yaml comments


def test_robot_code_never_reads_the_scenario():
    """Scenario coordinates are simulator ground truth. Robot code learns the world through
    perception -> world model, exactly as on hardware (PRD: no hardcoded victim locations)."""
    bad = [str(p) for p in _files()
           if re.search(r'fire_resq_simulation|config/scenarios|scenarios/\w+\.yaml', _code(p))]
    assert not bad, f'robot code must not touch the scenario: {bad}'


def test_robot_code_never_consumes_gazebo_ground_truth():
    bad = [str(p) for p in _files()
           if re.search(r'dynamic_pose|/pose/info|gz\.msgs\.Pose_V', _code(p))]
    assert not bad, f'ground-truth poses are for tests/evaluation only: {bad}'


def test_cognition_and_planning_do_not_import_hardware_or_gazebo():
    for pkg in ('fire_resq_cognition', 'fire_resq_planning'):
        for p in (SRC / pkg).rglob('*.py'):
            text = _code(p)
            assert 'fire_resq_hardware' not in text, p
            assert not re.search(r'^\s*(import|from)\s+(gz|ignition|ros_gz|RPi|serial)', text, re.M), p
