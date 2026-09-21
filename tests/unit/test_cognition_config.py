"""Cognition configuration and boundary rules that must not silently regress (no ROS graph, no simulator)."""
import importlib.util
import re
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
PKG = REPO / 'src' / 'fire_resq_cognition'
CFG = PKG / 'config' / 'prioritizer.yaml'
PURE = ('types.py', 'config.py', 'factors.py', 'prioritizer.py', '__init__.py')      # the decision logic: no ROS at all


def params():
    return yaml.safe_load(CFG.read_text())['prioritizer_node']['ros__parameters']


def code(path):
    """File text with comments and docstrings removed: naming a forbidden thing in documentation is not depending on it."""
    text = re.sub(r'("""|\'\'\').*?\1', '', path.read_text(), flags=re.S)
    return re.sub(r'(?m)#.*$', '', text)


def load(path):
    spec = importlib.util.spec_from_file_location(path.stem.replace('.', '_'), path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_the_shipped_config_is_valid_complete_and_matches_the_code_defaults():
    from fire_resq_cognition.config import DEFAULT_WEIGHTS, PrioritizerConfig
    p = params()
    cfg = PrioritizerConfig.from_dict({k: v for k, v in p.items() if k.startswith('weights.') or k in
                                       ('extent_m', 'fire_hazard_radius_m', 'fire_risk_decay_m', 'approach_standoff_m')})
    assert {k[8:]: v for k, v in p.items() if k.startswith('weights.')} == DEFAULT_WEIGHTS == cfg.weights
    assert p['decision_model'] in ('weighted_utility', 'nearest')
    default = PrioritizerConfig()
    assert (cfg.extent_m, cfg.fire_hazard_radius_m, cfg.fire_risk_decay_m, cfg.approach_standoff_m) == \
        (default.extent_m, default.fire_hazard_radius_m, default.fire_risk_decay_m, default.approach_standoff_m)


def test_the_node_declares_every_parameter_the_config_file_sets():
    """A typo in the YAML would otherwise be silently ignored by ROS."""
    src = (PKG / 'fire_resq_cognition' / 'prioritizer_node.py').read_text()
    declared = set(re.findall(r"\('(\w+)', ", src)) | {f'weights.{k}' for k in ('risk', 'fire_proximity', 'reach', 'accessibility',
                                                                                'confidence', 'cost')}
    declared |= set(re.findall(r"'(\w+)': [\d.]+", src[src.index('_DEFAULTS_SCALAR'):]))
    assert set(params()) <= declared, sorted(set(params()) - declared)


def test_the_decision_logic_is_pure_python_with_no_ros_and_only_two_modules_touch_ros():
    for name in PURE:
        f = PKG / 'fire_resq_cognition' / name
        assert not re.search(r'^\s*(import|from)\s+(rclpy|geometry_msgs|nav_msgs|nav2_msgs|std_msgs|action_msgs|tf2_ros|'
                             r'fire_resq_interfaces)', f.read_text(), re.M), f
    ros = {f.name for f in (PKG / 'fire_resq_cognition').glob('*.py')
           if re.search(r'^\s*(import|from)\s+(rclpy|nav2_msgs|fire_resq_interfaces)', f.read_text(), re.M)}
    assert ros == {'nav2_path_query.py', 'prioritizer_node.py'}


def test_cognition_stays_independent_of_gazebo_hardware_cameras_detectors_the_scenario_and_navigation_internals():
    for f in list((PKG / 'fire_resq_cognition').glob('*.py')) + list((PKG / 'launch').glob('*.py')):
        c = code(f)
        assert not re.search(r'^\s*(import|from)\s+(gz|ignition|ros_gz|RPi|serial|cv2|sensor_msgs|cv_bridge|pyrealsense2|'
                             r'fire_resq_perception|fire_resq_hardware|fire_resq_world_model|fire_resq_navigation)', c, re.M), f
        assert not re.search(r'fire_resq_simulation|scenarios/|dynamic_pose|/pose/info|source_backend|/camera|/scan\b|cmd_vel', c), f


def test_no_victim_id_coordinate_or_rescue_order_is_written_into_the_decision_code():
    """Rescue order, victim ids and positions must come from the world model, never from the code (PRD, CLAUDE.md)."""
    for f in (PKG / 'fire_resq_cognition').glob('*.py'):
        c = code(f)
        assert not re.search(r"""['"](victim|V)_?\d+['"]""", c), f'{f.name}: a victim id literal'
        assert not re.search(r'rescue_order|RESCUE_ORDER|priority_list|PRIORITY_LIST|fixed_order', c), f
        assert not re.search(r'\(\s*-?\d+\.\d+\s*,\s*-?\d+\.\d+\s*\)', c), f'{f.name}: a coordinate pair literal'


def test_no_weight_is_a_literal_in_the_scoring_code():
    """The weights are parameters. The scoring code may only read them from the configuration."""
    c = code(PKG / 'fire_resq_cognition' / 'prioritizer.py')
    assert not re.search(r'\b\d+\.\d+\s*\*\s*f\[', c) and not re.search(r'f\[[^\]]+\]\s*\*\s*\d+\.\d+', c)
    for name in ('risk', 'fire_proximity', 'reach', 'accessibility', 'confidence', 'cost'):
        assert f"w['{name}']" in c, f'{name} is not weighted by the configuration'


def test_cognition_never_reads_the_pose_of_the_camera_or_the_robots_actuators():
    """It consumes WorldState and asks a planner: no image, depth, TF lookup or velocity topics."""
    src = (PKG / 'fire_resq_cognition' / 'prioritizer_node.py').read_text()
    subscribed = set(re.findall(r"create_subscription\((\w+)", src))
    assert subscribed == {'WorldState'}


def test_the_cognition_launch_is_opt_in_and_started_once_by_the_navigation_launch():
    pytest.importorskip('launch')
    from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
    entities = load(REPO / 'simulation' / 'launch' / 'arena_nav.launch.py').generate_launch_description().entities
    args = {e.name: e for e in entities if isinstance(e, DeclareLaunchArgument)}
    assert args['cognition'].default_value[0].text == 'false' and args['decision_model'].default_value[0].text == 'weighted_utility'

    def includes(group, launch_file):
        return [a for a in group.get_sub_entities() if isinstance(a, IncludeLaunchDescription)
                and launch_file in ''.join(getattr(p, 'text', '') for p in a.launch_description_source._LaunchDescriptionSource__location)]
    groups = [g for g in entities if isinstance(g, GroupAction) and includes(g, 'prioritizer.launch.py')]
    assert len(groups) == 1 and groups[0].condition is not None and groups[0]._GroupAction__scoped
    passed = dict(includes(groups[0], 'prioritizer.launch.py')[0]._IncludeLaunchDescription__launch_arguments)
    assert passed['use_sim_time'] == 'true' and 'decision_model' in passed
