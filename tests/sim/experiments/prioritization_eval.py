"""Deterministic prioritisation evaluation on the project's scenario: the MVP weighted utility vs the nearest baseline.

  python3 tests/sim/experiments/prioritization_eval.py [--sweep]

No simulator, no ROS. The WORLD is the scenario's ground truth (three victims, the fire, the safe zone, the robot's start
pose), all known and all DETECTED at confidence 1.0, so the comparison isolates the decision models from perception
noise; the PATHS come from tests/scenario_paths.py (A* over the true geometry). This is EVALUATION tooling: it reads the
scenario, which no robot code may. The live version - the real world model and the real Nav2 planner - is
tests/sim/test_cognition.py.

It reports what each model decided and every factor value behind it. It scores nothing and declares no winner.
"""
import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
for rel in ('simulation', 'src/fire_resq_cognition', 'tests'):
    sys.path.insert(0, str(REPO / rel))
from fire_resq_cognition.config import DEFAULT_WEIGHTS, PrioritizerConfig  # noqa: E402
from fire_resq_cognition.prioritizer import make_prioritizer  # noqa: E402
from fire_resq_cognition.types import DETECTED, PlanningContext, VictimView, WorldView  # noqa: E402
from fire_resq_simulation import load_scenario, resolve_scenario  # noqa: E402
from scenario_paths import ScenarioPathOracle  # noqa: E402


def world_from_scenario(sc, confidence=1.0):
    victims = tuple(VictimView(v.id, v.x, v.y, confidence, DETECTED) for v in sc.victims)
    return WorldView(0.0, victims, True, (sc.fire.x, sc.fire.y), (sc.safe_zone.x, sc.safe_zone.y),
                     (sc.robot_start.x, sc.robot_start.y), sc.robot_start.yaw)


def table(decision):
    lines = [f"  model {decision.model}: selects {decision.selected.victim_id if decision.selected else None}   weights {decision.weights}",
             f"  {'victim':<10}{'utility':>9}  {'d_fire':>6} {'risk':>5} {'prox':>5} {'trip':>6} {'straight':>8} {'direct':>6} {'cost':>6}   contributions"]
    for s in decision.ranked:
        f = s.factors
        if s.eligible:
            c = ' '.join(f'{k}={v:+.2f}' for k, v in s.contributions.items())
            lines.append(f"  {s.victim_id:<10}{s.utility:>+9.3f}  {f['distance_to_fire_m']:>6.2f} {f['fire_risk']:>5.2f} {f['fire_proximity']:>5.2f} "
                         f"{f['robot_path_m']:>6.2f} {f['straight_distance_m']:>8.2f} {f['accessibility']:>6.2f} {f['rescue_cost_m']:>6.2f}   {c}")
        else:
            lines.append(f'  {s.victim_id:<10}excluded: {s.exclusion}')
    return '\n'.join(lines)


def evaluate(sc, config=None, oracle=None):
    oracle = oracle or ScenarioPathOracle(sc)
    w, ctx = world_from_scenario(sc), PlanningContext(oracle)
    return {m: make_prioritizer(m, config).select(w, ctx) for m in ('weighted_utility', 'nearest')}, w, oracle


def sweep(sc, oracle):
    """How the MVP's choice depends on its own parameters (measured, not judged)."""
    w, ctx = world_from_scenario(sc), PlanningContext(oracle)
    print('\nSensitivity of the weighted-utility choice (the nearest baseline has no parameters to sweep):')
    print('  weights.risk        ->', ', '.join(f"{r:g}:{make_prioritizer('weighted_utility', PrioritizerConfig(weights={**DEFAULT_WEIGHTS, 'risk': r})).select(w, ctx).selected.victim_id}"
                                                for r in (0, 0.5, 1, 2, 3, 4, 6, 8, 12)))
    print('  fire_hazard_radius_m->', ', '.join(f"{r:g}:{make_prioritizer('weighted_utility', PrioritizerConfig(fire_hazard_radius_m=r)).select(w, ctx).selected.victim_id}"
                                                for r in (0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0)))
    print('  fire_risk_decay_m   ->', ', '.join(f"{r:g}:{make_prioritizer('weighted_utility', PrioritizerConfig(fire_risk_decay_m=r)).select(w, ctx).selected.victim_id}"
                                                for r in (0.25, 0.5, 1.0, 2.0, 4.0)))
    flip = None
    prev = None
    for r in [x / 20 for x in range(0, 400)]:
        cur = make_prioritizer('weighted_utility', PrioritizerConfig(weights={**DEFAULT_WEIGHTS, 'risk': r})).select(w, ctx).selected.victim_id
        if prev is not None and cur != prev:
            flip = (r, prev, cur)
            break
        prev = cur
    print(f'  first change of choice as weights.risk rises from 0 in steps of 0.05: {flip}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sweep', action='store_true')
    args = ap.parse_args()
    sc = load_scenario(resolve_scenario('default'))
    decisions, w, oracle = evaluate(sc)
    s = sc.robot_start
    print(f'Scenario {sc.name!r}: robot start ({s.x:.2f}, {s.y:.2f}), fire ({sc.fire.x:.2f}, {sc.fire.y:.2f}), '
          f'safe zone ({sc.safe_zone.x:.2f}, {sc.safe_zone.y:.2f}); all victims known, DETECTED, confidence 1.0; paths: A* over the true geometry')
    print('victims (ground-truth ids, for evaluation only): ' + ', '.join(f'{v.id} ({v.x:.2f}, {v.y:.2f})' for v in sc.victims))
    for d in decisions.values():
        print('\n' + table(d))
    a, b = (decisions[m].selected.victim_id for m in ('weighted_utility', 'nearest'))
    print(f'\nWeighted utility selects {a}; nearest-first selects {b}: ' + ('they DIFFER.' if a != b else 'they AGREE.'))
    if args.sweep:
        sweep(sc, oracle)
    return 0


if __name__ == '__main__':
    sys.exit(main())
