# How to run the complete simulation

Short version: install → build → make a map once → run the mission → watch it rescue all three victims.
Everything is copy-paste. Details live in [CLAUDE.md](CLAUDE.md) and [docs/Implementation_Plan.md](docs/Implementation_Plan.md); this page only gets you running.

**What you will see:** the robot looks around, picks which victim to save (the one next to the fire first, not simply the nearest), drives to it, picks it up with the magnet, carries it to the green safe zone, puts it down, and repeats. It searches the room for the hidden victim behind the wall. It stops when everything is rescued. That takes about 5 minutes.

---

## 0. What you need

- Ubuntu **24.04**: native, or WSL2 on Windows 11.
- About 8 GB RAM, 15 GB disk. A GPU helps but isn't required.

---

## 1. Install (once)

```bash
# ROS 2 Jazzy apt repo
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list

# Gazebo Harmonic apt repo
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list

sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz gz-harmonic \
  python3-gz-transport13 python3-gz-msgs10 \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-depthimage-to-laserscan ros-jazzy-xacro ros-jazzy-teleop-twist-keyboard \
  python3-colcon-common-extensions python3-opencv python3-pytest python3-yaml
```

Verified versions are in [docs/Environment.md](docs/Environment.md) §2.

---

## 2. Get the code and build it

```bash
git clone https://github.com/dormeneur/FireResQ.git
cd FireResQ
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install          # ~20 s, 10 packages
source install/setup.bash
```

**Every new terminal needs this first:**

```bash
cd ~/FireResQ && source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

**On WSL2 with an NVIDIA GPU**, also run the line below (optional: it's faster, but the default works too; see [CLAUDE.md](CLAUDE.md) → "Environment"):

```bash
export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
```

---

## 3. Quick look: just the arena (optional)

```bash
ros2 launch fire_resq_simulation arena.launch.py
```

Gazebo opens with the robot, the orange fire, three blue victims, obstacles and the green safe zone. Close it with Ctrl+C.

---

## 4. Make the map (once)

The mission navigates on a saved map. Pick one of these two ways to make it.

**Option A: automatic (easiest).** Run one rescue test. It drives a mapping lap by itself and saves the map:

```bash
python3 -m pytest tests/sim/test_rescue_loop.py -q
M=$(ls -td /tmp/fire_resq_map_* | head -1)    # the newest saved map
cp $M/arena.yaml $M/arena.pgm ~/              # keep a copy: ~/arena.yaml
```

**Option B: drive it yourself.**

Terminal 1:

```bash
ros2 launch fire_resq_simulation arena_nav.launch.py navigation:=false
```

Terminal 2: drive a slow lap around the whole room with the keyboard (`i` forward, `j`/`l` turn, `k` stop):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> **Drive forward to map; spinning in place adds nothing to the map.** The reason is in [CLAUDE.md](CLAUDE.md) → "Navigation".

Terminal 3: save the map, then close everything:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/arena --ros-args -p use_sim_time:=true
```

---

## 5. Run the complete rescue mission

```bash
ros2 launch fire_resq_simulation arena_nav.launch.py \
  localization:=amcl map:=$HOME/arena.yaml \
  perception:=true spatial_backend:=depth world_model:=true \
  cognition:=true magnet:=true rescue:=true
```

The mission starts by itself. Add `use_rviz:=true` to see the map and the robot's path; add `gui:=false` to hide Gazebo.

---

## 6. Watch what it is doing (second terminal)

```bash
# current state + everything that happened so far (JSON)
ros2 topic echo /fire_resq/mission_state --once --field data

# the decision: which victim, and why
ros2 topic echo /fire_resq/rescue_target --once
```

The launch terminal also prints every state change, for example:

```
[ 32.8s] SELECT_TARGET -> NAVIGATE_TO_APPROACH: target V2 chosen by cognition
[ 63.8s] ATTACH -> VERIFY_CARRY: the magnet holds V2
[104.7s] RELEASE -> VERIFY_RELEASE: the magnet reports the victim released
...
[290.0s] SEARCH -> MISSION_COMPLETE: ... rescued ['V2', 'V1', 'V3']
```

What every state means: [docs/Implementation_Plan.md](docs/Implementation_Plan.md) → Phase 10 → "Transition table".

---

## 7. Try the variations

```bash
# the "dumb" baseline: always the nearest victim (compare the order with step 5)
ros2 launch fire_resq_simulation arena_nav.launch.py localization:=amcl map:=$HOME/arena.yaml \
  perception:=true spatial_backend:=depth world_model:=true cognition:=true magnet:=true rescue:=true \
  decision_model:=nearest

# the decision maths only, no simulator (seconds)
python3 tests/sim/experiments/prioritization_eval.py --sweep

# a different room layout: copy and edit the scenario file, then pass it in
cp simulation/config/scenarios/default.yaml /tmp/my.yaml
ros2 launch fire_resq_simulation arena.launch.py scenario:=/tmp/my.yaml
```

The scenario file format is documented in its own header (`simulation/config/scenarios/default.yaml`). Each new layout needs its own map (step 4).

---

## 8. Prove it works (tests)

```bash
python3 -m pytest tests/unit -q                       # seconds: logic, rules, the mission FSM
python3 -m pytest tests/sim/test_rescue_loop.py -q -rP    # ~6 min: one full mission, checked against Gazebo's real positions
python3 -m pytest tests/sim/test_rescue_faults.py -q -rP  # ~8 min: failed pickup, lost victim, dropped victim
python3 -m pytest tests -q -rPx                       # everything: ~48 min (488 tests)
```

The sim tests start their own simulation, so close any running one first.

---

## 9. If something goes wrong

| Problem | Fix |
| --- | --- |
| "stray simulation processes are running" | `pkill -f "gz sim"; pkill -f ros2` and try again |
| Robot doesn't move at mission start | Wait ~10 s: AMCL and Nav2 are still starting. The map must be the one made in step 4. |
| Everything is very slow | Check the speed: `gz topic -e -t /stats -n 5 \| grep real_time_factor` should be ≈ 1.0. On WSL2, `wsl --shutdown` (from Windows) and retry. |
| Robot drives forever after you stop a command | Send a stop: `ros2 topic pub --times 5 -r 10 /cmd_vel geometry_msgs/msg/Twist "{}"` |
| A test fails once | Rerun that file. Known flaky cases and how to judge them: [CLAUDE.md](CLAUDE.md) → "Test reliability" |

All commands, one per line: [CLAUDE.md](CLAUDE.md) → "Commands".
