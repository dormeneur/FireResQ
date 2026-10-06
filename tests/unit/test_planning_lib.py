"""The rescue procedure's pure library: config, geometry, coverage, and the guards that keep its numbers tied to the sources
of truth (the description, the victim model, the world model's safe zone, cognition's approach contract, the sim magnet)."""
import math
import re
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
for pkg in ('fire_resq_planning', 'fire_resq_world_model', 'fire_resq_description', 'fire_resq_cognition'):
    sys.path.insert(0, str(REPO / 'src' / pkg))
sys.path.insert(0, str(REPO / 'simulation'))
sys.path.insert(0, str(REPO / 'tests' / 'unit'))

from fsm_harness import arena_grid  # noqa: E402

from fire_resq_cognition.config import PrioritizerConfig  # noqa: E402
from fire_resq_cognition.factors import approach_pose as cognition_approach_pose  # noqa: E402
from fire_resq_description.geometry import read_properties  # noqa: E402
from fire_resq_planning import geometry as G  # noqa: E402
from fire_resq_planning.config import PlanningConfig  # noqa: E402
from fire_resq_planning.coverage import CoveragePlanner  # noqa: E402
from fire_resq_planning.fsm import ALLOWED, RescueFSM  # noqa: E402
from fire_resq_planning.types import State, TERMINAL  # noqa: E402
from fire_resq_simulation.magnet_geometry import magnet_contact_distance_m  # noqa: E402

CFG = PlanningConfig()
PROPS = read_properties()


# ------------------------------------------------------------------------------------------------------ config
def test_the_defaults_are_valid_and_unknown_or_unsafe_values_are_refused():
    PlanningConfig()
    with pytest.raises(ValueError, match='unknown planning parameters'):
        PlanningConfig.from_dict({'victim_x': 1.0})
    with pytest.raises(ValueError, match='velocity limits'):
        PlanningConfig(align_speed_mps=0.5)
    with pytest.raises(ValueError, match='standoff'):
        PlanningConfig(approach_standoff_m=0.10)
    with pytest.raises(ValueError, match='align_stop_gap'):
        PlanningConfig(align_stop_gap_m=0.01)


def test_from_dict_keeps_int_parameters_int():
    c = PlanningConfig.from_dict({'max_target_attempts': 3, 'align_speed_mps': 0.05})
    assert c.max_target_attempts == 3 and isinstance(c.max_target_attempts, int) and c.align_speed_mps == 0.05


def test_no_coordinate_victim_id_or_order_lives_in_the_planning_code():
    """PRD: never hardcode victim coordinates, rescue order or paths."""
    for p in (REPO / 'src' / 'fire_resq_planning' / 'fire_resq_planning').glob('*.py'):
        text = re.sub(r'(\"\"\"|\'\'\').*?\1', '', p.read_text(), flags=re.S)
        text = re.sub(r'(?m)#.*$', '', text)
        assert not re.search(r"['\"](V\d+|victim_\d+)['\"]", text), f'{p.name} names a victim'
        assert not re.search(r'victims?\s*=\s*\[|order\s*=\s*\[', text), f'{p.name} carries a victim list or order'


# ------------------------------------------------------------------------------- the numbers match their sources
def test_the_ring_radius_matches_the_victim_model():
    sdf = (REPO / 'simulation/models/victim/model.sdf').read_text()
    assert CFG.victim_ring_radius_m == pytest.approx(float(re.search(r'<cylinder><radius>([\d.]+)</radius><length>0.055</length>', sdf)[1]))


def test_the_magnet_reach_and_speed_limits_match_the_robot_description():
    assert CFG.magnet_reach_m == pytest.approx(PROPS['magnet_x'] + PROPS['magnet_length'] / 2)
    assert CFG.max_linear_mps == pytest.approx(PROPS['max_linear_velocity'])
    assert CFG.max_angular_rps == pytest.approx(PROPS['max_angular_velocity'])


def test_contact_distance_agrees_with_the_simulations_magnet_and_phase_8s_standoff_guard():
    sim = magnet_contact_distance_m(PROPS['magnet_x'], PROPS['magnet_length'])
    assert CFG.contact_center_dist_m == pytest.approx(sim) == pytest.approx(0.131, abs=1e-3)


def test_the_safe_zone_size_matches_the_world_models_configuration():
    cfg = yaml.safe_load((REPO / 'src/fire_resq_world_model/config/world_model.yaml').read_text())['world_model_node']['ros__parameters']
    assert CFG.safe_zone_size_x == cfg['safe_zone.size_x'] and CFG.safe_zone_size_y == cfg['safe_zone.size_y']


def test_the_approach_pose_is_the_contract_cognition_fixes():
    assert CFG.approach_standoff_m == PrioritizerConfig().approach_standoff_m
    for robot, victim in (((0.0, 0.0), (2.0, 1.0)), ((3.0, -1.0), (0.5, 0.5)), ((1.0, 1.0), (1.2, 1.0))):
        mine = G.approach_pose(robot, victim, CFG.approach_standoff_m)
        x, y, yaw = cognition_approach_pose(robot, victim, CFG.approach_standoff_m)
        assert (mine.x, mine.y, mine.yaw) == pytest.approx((x, y, yaw))


def test_the_status_codes_match_the_interface_definition():
    from fire_resq_planning import types as T
    msg = (REPO / 'src/fire_resq_interfaces/msg/VictimState.msg').read_text()
    for name in ('UNKNOWN', 'DETECTED', 'TARGETED', 'CARRIED', 'RESCUED', 'UNREACHABLE'):
        assert int(re.search(rf'uint8 {name}=(\d+)', msg)[1]) == getattr(T, name)


# ---------------------------------------------------------------------------------------------------- geometry
def test_contact_gap_is_zero_flush_and_the_carried_position_is_dead_ahead():
    for yaw in (0.0, 1.0, -2.5):
        pose = G.Pose2(1.0, 2.0, yaw)
        v = G.expected_carried_xy(pose, CFG)
        assert G.contact_gap(pose, v, CFG) == pytest.approx(0.0, abs=1e-9)
        assert G.dist(v, pose.xy) == pytest.approx(CFG.contact_center_dist_m)


def test_zone_membership_and_depth():
    z = G.Zone(1.0, 1.0, math.pi / 4, 1.0, 1.0)
    assert z.contains((1.0, 1.0)) and not z.contains((2.0, 2.0))
    assert z.depth((1.0, 1.0)) == pytest.approx(0.5)
    assert z.depth(z.world((0.5, 0.0))) == pytest.approx(0.0, abs=1e-9)
    assert z.depth(z.world((0.7, 0.0))) < 0
    assert z.contains(z.world((0.4, 0.0)), margin=0.05) and not z.contains(z.world((0.48, 0.0)), margin=0.05)


def test_release_spots_are_generated_from_the_zone_and_keep_clear_of_its_centre():
    z = G.Zone(3.0, -2.0, 0.6, 1.0, 1.0)
    spots = G.release_spots(z, CFG)
    assert len(spots) == 2 * CFG.release_spot_count             # two rings
    for s in spots:
        assert z.contains(s, margin=CFG.release_margin_m)
        assert G.dist(s, z.world((0, 0))) >= 0.25, 'the return-leg planner query and the next approach need the centre free'


def test_each_victim_gets_its_own_release_spot_and_the_robot_stands_inside_the_zone():
    z = G.Zone(0.0, 0.0, 0.3, 1.0, 1.0)
    used, robot = [], (2.0, 2.0)
    for _ in range(3):
        plan = G.plan_release(z, robot, used, CFG)
        assert plan is not None
        assert z.contains(plan.pose.xy, margin=0.05)
        assert G.release_ready(z, plan.pose, used, CFG)[0], 'the planned pose must satisfy its own release condition'
        assert G.dist(G.expected_carried_xy(plan.pose, CFG), plan.spot) < 1e-9
        assert all(G.dist(plan.spot, u) >= CFG.release_spot_min_sep_m for u in used)
        used.append(plan.spot)
    assert len({tuple(round(v, 3) for v in u) for u in used}) == 3


def test_release_ready_refuses_a_victim_that_would_land_outside_or_on_top_of_another():
    z = G.Zone(0.0, 0.0, 0.0, 1.0, 1.0)
    ok, why = G.release_ready(z, G.Pose2(0.30, 0.0, 0.0), [], CFG)          # victim would be 0.43 m out: only 7 cm inside
    assert not ok and 'inside the zone edge' in why
    ok, why = G.release_ready(z, G.Pose2(-0.1, 0.0, 0.0), [(0.05, 0.0)], CFG)
    assert not ok and 'already released' in why


def test_the_zone_running_out_of_room_is_reported_not_papered_over():
    z = G.Zone(0.0, 0.0, 0.0, 1.0, 1.0)
    used = G.release_spots(z, CFG)
    assert G.plan_release(z, (2.0, 0.0), used, CFG) is None


# ---------------------------------------------------------------------------------------------------- coverage
def test_coverage_grows_with_looks_and_a_far_viewpoint_is_preferred_over_a_useless_one():
    cp = CoveragePlanner(arena_grid(), CFG)
    assert cp.coverage() == 0.0
    new = cp.mark_looked(0.8, 0.8)
    assert new > 0 and 0.0 < cp.coverage() < 1.0
    pick = cp.next_viewpoint((0.8, 0.8))
    assert pick is not None and pick[2] >= CFG.min_gain_cells
    before = cp.coverage()
    cp.mark_looked(*pick[1])
    assert cp.coverage() > before


def test_a_look_does_not_credit_cells_too_close_to_place():
    cp = CoveragePlanner(arena_grid(block=False), CFG)
    vis = cp.visible_from(2.5, 2.5)
    r, c = cp.grid.cell(2.5, 2.5)
    assert not vis[r, c]
    r2, c2 = cp.grid.cell(2.9, 2.5)                      # 0.4 m away: a blob there touches the image border
    assert not vis[r2, c2]
    r3, c3 = cp.grid.cell(3.9, 2.5)
    assert vis[r3, c3]


def test_walls_block_a_look_and_unknown_space_counts_as_a_wall():
    grid = arena_grid(block=False)
    cp = CoveragePlanner(grid, CFG)
    vis = cp.visible_from(0.8, 0.8)
    assert not vis[grid.data == -1].any() and not vis[grid.data == 100].any()
    grid2 = arena_grid(block=True)                       # the partition hides what lies behind it
    cp2 = CoveragePlanner(grid2, CFG)
    r, c = grid2.cell(1.2, 3.8)
    assert cp.visible_from(1.2, 1.0)[r, c] and not cp2.visible_from(1.2, 1.0)[r, c]


def test_search_is_exhausted_once_the_free_space_has_been_looked_at():
    cp = CoveragePlanner(arena_grid(), CFG)
    guard = 0
    while (pick := cp.next_viewpoint((0.8, 0.8))) is not None and guard < 40:
        cp.mark_looked(*pick[1])
        guard += 1
    assert cp.coverage() >= CFG.coverage_goal - 0.05 and guard < 40


# ------------------------------------------------------------------------------------------------- the FSM's table
def test_every_state_is_declared_reachable_and_terminal_states_have_no_exits():
    assert set(ALLOWED) == set(State)
    seen, todo = {State.INIT}, [State.INIT]
    while todo:
        for nxt in ALLOWED[todo.pop()]:
            if nxt not in seen:
                seen.add(nxt)
                todo.append(nxt)
    assert seen == set(State), f'unreachable: {set(State) - seen}'
    for s in TERMINAL:
        assert not ALLOWED[s]
    for s in State:
        assert hasattr(RescueFSM, '_tick_' + s.value.lower()), f'{s} has no handler'
    for s in State:
        if s not in TERMINAL:
            assert State.MISSION_ABORTED in ALLOWED[s], f'{s} cannot reach the fatal state'


def test_the_zone_fills_from_the_far_side_so_a_released_victim_never_blocks_the_entrance():
    """Measured live: the first victim put down on the entry side made the next return leg abort ("collision ahead")."""
    z = G.Zone(0.0, 0.0, 0.0, 1.0, 1.0)
    robot = (2.0, 0.0)                                          # arriving from +x
    first = G.plan_release(z, robot, [], CFG)
    assert first.spot[0] < -0.2, f'the first victim should go deep (far side), not at {first.spot}'


def test_three_victims_go_to_the_deep_inner_ring_before_the_outer_one():
    """Measured live: a victim planned onto the outer ring (~10 cm inside the edge) landed outside after 17 cm of
    localisation error; the inner ring holds three victims at well over the minimum separation."""
    z = G.Zone(0.0, 0.0, -math.pi / 4, 1.0, 1.0)
    used = []
    for robot in [(1.5, 1.0), (1.2, 1.4), (0.8, 1.6)]:
        plan = G.plan_release(z, robot, used, CFG)
        assert G.dist(plan.spot, (0.0, 0.0)) <= CFG.release_spot_radius_m + 1e-6, f'{plan.spot} is on the outer ring'
        assert z.depth(plan.spot) >= 0.2
        used.append(plan.spot)
