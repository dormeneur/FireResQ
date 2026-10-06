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


def test_control_stays_hardware_and_simulator_agnostic():
    """The Phase 9 magnet abstraction (magnet_backend.py, magnet_node.py) talks only to two
    standard ROS topics; the Gazebo-specific driver on the other end lives in fire_resq_simulation
    (outside src/, so it can legitimately read ground truth), never here."""
    for p in (SRC / 'fire_resq_control').rglob('*.py'):
        text = _code(p)
        assert 'fire_resq_hardware' not in text and 'fire_resq_simulation' not in text, p
        assert not re.search(r'^\s*(import|from)\s+(gz|ignition|ros_gz|RPi|serial)', text, re.M), p


def test_perception_stays_camera_and_simulator_agnostic():
    """Perception runs unchanged on the simulated and the real robot: stable ROS topics and TF only."""
    for p in (SRC / 'fire_resq_perception').rglob('*.py'):
        text = _code(p)
        assert not re.search(r'^\s*(import|from)\s+(gz|ignition|ros_gz|RPi|serial|pyrealsense2)', text, re.M), p
        assert 'fire_resq_hardware' not in text, p


def test_cognition_and_planning_cannot_branch_on_which_camera_backend_produced_a_position():
    """`source_backend` is provenance for logs and evaluation. Cognition must not be able to tell RGB from RGB-D
    (Architecture.md section 4), so it may not read the field."""
    for pkg in ('fire_resq_cognition', 'fire_resq_planning', 'fire_resq_world_model'):
        for p in (SRC / pkg).rglob('*.py'):
            assert 'source_backend' not in _code(p), p


def test_planning_talks_to_the_other_layers_only_through_ros_interfaces():
    """The rescue FSM consumes messages and services - never another layer's implementation (Architecture.md: layers are
    replaceable). It must not import perception, cognition or world-model code, nor navigation internals, nor simulation code."""
    for p in (SRC / 'fire_resq_planning').rglob('*.py'):
        text = _code(p)
        assert not re.search(r'^\s*(import|from)\s+(fire_resq_perception|fire_resq_cognition|fire_resq_world_model|fire_resq_navigation'
                             r'|fire_resq_simulation|fire_resq_hardware)', text, re.M), p


def test_the_rescue_fsm_never_calls_the_wall_clock_or_the_simulator_directly():
    """fsm.py is a pure function of its Observations: the same inputs give the same mission (unit-tested), which needs it to read
    no clock and to move nothing itself."""
    text = _code(SRC / 'fire_resq_planning' / 'fire_resq_planning' / 'fsm.py')
    assert not re.search(r'\b(time\.time|time\.monotonic|datetime|random\.|os\.environ|subprocess|rclpy)\b', text)
