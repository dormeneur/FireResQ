"""Phase 5 perception through the NAVIGATION launch (arena_nav.launch.py, 87 deg camera), judged against ground truth.

Its own module because it needs its own simulation; the 60 deg arena runs are in test_perception.py.
"""
from perceptionlib import FIRE_TOL_M, VICTIM_TOL_M, collect, seen, valid


def test_arena_nav_launch_runs_perception_with_the_87_degree_camera(percsim_nav, truth):
    bot = percsim_nav.bot
    bot.stop(1.0)
    dets = valid(collect(bot, 3.0))
    errs = seen(truth, dets)
    assert {'fire', 'victim_1', 'victim_2'} <= set(errs), sorted(errs)
    for e, v in errs.items():
        assert v <= (FIRE_TOL_M if e == 'fire' else VICTIM_TOL_M), f'{e}: {100 * v:.1f} cm at 87 deg'
    print('\n87 deg (median error, cm):', {e: round(100 * v, 1) for e, v in sorted(errs.items())})
