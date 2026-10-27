#!/usr/bin/env bash
# One-command FireResQ demo: SLAM map + the whole unattended rescue mission + a results folder.
#   ./run_demo.sh [--random SEED | --scenario FILE.yaml] [--gui] [--model nearest] [--map SAVED.yaml]
# Options: tests/sim/experiments/demo_mission.py --help. Results: results/<scenario>_<model>_<time>/report.md
set -e
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
[ -f install/setup.bash ] || colcon build --symlink-install    # a fresh checkout builds once (~20 s)
source install/setup.bash
exec python3 tests/sim/experiments/demo_mission.py "$@"
