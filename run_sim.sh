#!/usr/bin/env bash
# FireResQ quick demo: a small random arena (3 x 3 m), one fire, three victims. The robot looks, ranks the victims by
# distance to the fire, and carries them to the safe zone closest-to-fire first. ~2 min, Gazebo window included.
#   ./run_sim.sh          a new random layout (its seed is printed)
#   ./run_sim.sh 42       layout 42 again
#   HEADLESS=1 ./run_sim.sh
set -e
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
[ -f install/setup.bash ] || colcon build --symlink-install
source install/setup.bash
export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA      # GPU rendering under WSL (see CLAUDE.md)

SEED=${1:-$RANDOM}
OUT=/tmp/fire_resq_quick
mkdir -p "$OUT"

# A leftover simulation would fight this one for the same topics.
pkill -f '[g]z sim' || true; pkill -f '[r]os2 launch' || true; pkill -f '[i]nstall/fire_resq' || true
pkill -f '[o]pt/ros/jazzy/lib/' || true
sleep 1

echo "== layout (seed $SEED) =="
python3 quick_demo/make_scenario.py --seed "$SEED" --out "$OUT/scenario.yaml"

GUI=true; [ -n "$HEADLESS" ] && GUI=false
setsid ros2 launch fire_resq_simulation arena.launch.py scenario:="$OUT/scenario.yaml" \
  base_world:="$PWD/simulation/worlds/demo_world.sdf" gui:=$GUI camera_hfov:=1.518 \
  perception:=true perception_frame:=odom spatial_backend:=depth magnet:=true > "$OUT/sim.log" 2>&1 &
SIM=$!
trap 'kill -INT -$SIM 2>/dev/null; sleep 3; kill -9 -$SIM 2>/dev/null; true' EXIT
echo "== simulation starting (log: $OUT/sim.log) =="

if [ "$GUI" = true ]; then
  # Frame the arena from the south, looking down at ~50 degrees.
  ( for _ in $(seq 60); do
      gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 1000 \
        --req 'pose: {position: {x: 0, y: -2.4, z: 2.7}, orientation: {w: 0.644, x: -0.291, y: 0.291, z: 0.644}}' \
        2>/dev/null | grep -q true && break
      sleep 1
    done ) &
fi

python3 quick_demo/rescue.py
if [ "$GUI" = true ] && [ -t 0 ]; then read -r -p "Press Enter to close the simulation. "; fi
