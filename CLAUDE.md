# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Current state

**Phases 0–5 complete** — the ROS 2 workspace builds (ten packages); the simulated robot drives, reports odometry and joint states and publishes RGB/depth; a deterministic 5 m × 5 m rescue arena (fire, three victims, safe zone, obstacles) launches from a scenario file; and a navigation stack (depth → `/scan`, SLAM Toolbox or AMCL for `map→odom`, Nav2) maps the arena and drives to supplied goals; and a perception node finds the fire and victims by colour and places them in a TF frame (depth or, RGB-only, from object-height priors) on `/fire_resq/detections`. All of it is covered by a regression suite that checks against Gazebo ground truth (`tests/`). The world model, cognition, rescue planning and the electromagnet are not implemented; those packages are skeletons. See [docs/Implementation_Plan.md](docs/Implementation_Plan.md) for the phase roadmap and what each phase may touch.

Design docs remain the source of truth:

- [PRD.md](PRD.md) — scope, MVP checklist, evaluation metrics, out-of-scope, definition of done.
- [Architecture.md](Architecture.md) — ROS 2 layers, custom interfaces, package layout, sim→hardware rule.
- [Hardware.md](Hardware.md) — component list, build phases, sim-to-hardware mapping.
- [docs/Implementation_Plan.md](docs/Implementation_Plan.md) — phased roadmap, per-phase definition of done.
- [docs/Environment.md](docs/Environment.md) — verified toolchain versions and measured sim performance.
- [docs/Cognitive_Model.md](docs/Cognitive_Model.md) — world state and the victim-priority model.
- [docs/Simulation.md](docs/Simulation.md) — the 10-stage simulation bring-up ladder.
- [docs/Hardware_Integration.md](docs/Hardware_Integration.md) — the high/low-level interface boundary.

When a design decision changes, update the owning doc in the same change as the code — the docs are meant to stay authoritative, not become stale.

## What the project is

FireResQ: a ROS 2 cognitive search-and-rescue robot (academic prototype, not a real firefighting system). A differential-drive robot perceives a ~5×5 m arena holding a fire, three lightweight metal victims, obstacles, and a safe zone; maps it with SLAM; maintains a world model; decides *which victim to rescue next*; navigates with Nav2; lifts the victim with an electromagnet; releases it in the safe zone; then reassesses.

Loop: Perception → SLAM/Localization → World Model → Reasoning → Decision → Planning → Action → Reassessment.

The interesting part is the decision layer. Everything else exists to feed it a world model and execute what it picks.

## Environment

WSL2 on Ubuntu 24.04; full toolchain installed. Versions and measured sensor rates are recorded in [docs/Environment.md](docs/Environment.md) — read that rather than re-probing.

The one non-obvious thing: **default OpenGL is software rasterization (llvmpipe)** despite an RTX 3050 being present. Hardware acceleration needs to be asked for explicitly, and without the adapter hint Mesa picks the Intel iGPU:

```bash
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
```

Both paths hold real-time factor 1.0; the accelerated one cuts user CPU from ~14% to ~4%, which matters once SLAM and Nav2 are competing for cores. Software rendering is a working fallback.

Not installed, deliberately: `ultralytics` (MVP detector is OpenCV-based), `rtabmap_slam`, `ros2_control`, `joint_state_publisher_gui`.

## Commands

All verified working as of Phase 3.

```bash
# every new shell
source /opt/ros/jazzy/setup.bash

# build (from repo root) — 10 packages, ~19 s clean
colcon build --symlink-install
source install/setup.bash                 # after every build

# one package
colcon build --packages-select fire_resq_cognition
colcon test  --packages-select fire_resq_cognition
colcon test-result --verbose              # see failures
```

`colcon` scans the workspace recursively, so `simulation/` builds as the `fire_resq_simulation` package despite sitting outside `src/`.

Once test files exist, prefer plain pytest while iterating — it skips the colcon cycle:

```bash
python3 -m pytest src/fire_resq_cognition/test/test_priority.py -k victim_scoring -v
```

### Running things

```bash
# robot description alone — no simulator. RViz + full TF tree.
ros2 launch fire_resq_description display.launch.py

# THE RESCUE ARENA (use this): generates the world from a scenario YAML, then runs sim.launch.py
ros2 launch fire_resq_simulation arena.launch.py                      # default scenario, Gazebo GUI
ros2 launch fire_resq_simulation arena.launch.py gui:=false           # headless (server only)
ros2 launch fire_resq_simulation arena.launch.py use_rviz:=true       # + RViz (RGB, depth, TF, odom)
ros2 launch fire_resq_simulation arena.launch.py use_depth:=false     # RGB-only robot: no /camera/depth/*
ros2 launch fire_resq_simulation arena.launch.py scenario:=/path/to/other.yaml
ros2 launch fire_resq_simulation arena.launch.py overview_camera:=true  # DEBUG top-down view on /overview/image_raw

# scenario tooling (no simulator needed)
ros2 run fire_resq_simulation generate_world.py --ascii               # print the layout
ros2 run fire_resq_simulation generate_world.py --output /tmp/w.sdf   # write the world SDF

# THE NAVIGATION STACK (Phase 4): arena + depth->/scan + SLAM (or AMCL) + Nav2. 87 deg camera.
ros2 launch fire_resq_simulation arena_nav.launch.py                    # scan-matching SLAM + Nav2, Gazebo GUI
ros2 launch fire_resq_simulation arena_nav.launch.py gui:=false use_rviz:=true   # headless sim + the navigation RViz view
ros2 launch fire_resq_simulation arena_nav.launch.py slam_preset:=odom_only      # NOT SLAM: odometry-only mapping baseline
ros2 launch fire_resq_simulation arena_nav.launch.py localization:=none navigation:=false    # only depth -> /scan
# Map once, then navigate on the SAVED map with AMCL (the mapped-arena mode; see "Navigation" below):
ros2 run nav2_map_server map_saver_cli -f /tmp/arena                    # while SLAM is running and has mapped
ros2 launch fire_resq_simulation arena_nav.launch.py localization:=amcl map:=/tmp/arena.yaml
#   ...then give the start pose: RViz "2D Pose Estimate", or publish /initialpose. Send goals with RViz "2D Goal
#   Pose" (/goal_pose) or the /navigate_to_pose action.

# PERCEPTION (Phase 5): opt-in on either launch; publishes /fire_resq/detections
ros2 launch fire_resq_simulation arena.launch.py perception:=true                       # positions in `odom` (no SLAM here)
ros2 launch fire_resq_simulation arena.launch.py use_depth:=false perception:=true       # RGB-only robot: known-height estimator
ros2 launch fire_resq_simulation arena_nav.launch.py perception:=true                    # positions in `map` (needs localization != none)
ros2 launch fire_resq_perception perception.launch.py target_frame:=odom spatial_backend:=known_height   # the node alone (also what hardware runs)
ros2 topic echo /fire_resq/detections --once

# robot-only bringup in an EMPTY world (what arena.launch.py wraps)
ros2 launch fire_resq_simulation sim.launch.py                        # Gazebo GUI
ros2 launch fire_resq_simulation sim.launch.py gui:=false             # headless (server only)
ros2 launch fire_resq_simulation sim.launch.py use_rviz:=true         # + RViz (RGB, depth, TF, odom)
ros2 launch fire_resq_simulation sim.launch.py use_depth:=false       # RGB-only robot: no /camera/depth/*
ros2 launch fire_resq_simulation sim.launch.py world:=/abs/path.sdf x:=1.0 y:=0.5 yaw:=1.57
```

### Tests

Three tiers (config in `pytest.ini`). Source the workspace first (`source install/setup.bash`).

```bash
python3 -m pytest tests/unit -q                       # <1 s: scenario, robot params, Nav2 params, launch/config rules, architecture guards
python3 -m pytest tests/node -q                       # ~25 s: the perception node in-process against synthetic ROS traffic (no Gazebo)
python3 -m pytest tests/sim -m "not slow" -q          # ~20 min: launches the real sim (one per module) - robot, TF, sensors, arena, scan, SLAM, AMCL, Nav2
python3 -m pytest tests -q -rPx                       # EVERYTHING incl. slow (RGB-only, determinism): 217 passed + 1 xfail in ~26 min (134 unit, 8 node, 76 sim); -rPx prints the MEASURED values
python3 -m pytest tests/sim/test_robot.py -k spin -v  # one module / one case
```

**Reproducible experiments** (not tests; pytest does not collect them) live in `tests/sim/experiments/`: `slam_fov_spike.py` (the Phase 4 Step 0 SLAM viability/tuning measurements), `nav2_localization_diag.py` (SLAM/odometry error vs velocity while Nav2 drives), `nav_performance.py` (rates, map-update cadence, CPU, RTF), `perception_accuracy.py` (Phase 5: position error per estimator and range, and what the motion gate costs and buys).

The sim tests launch and tear down their own simulation (own process group; no strays), so don't have another sim running - the suite refuses to start on top of one. They judge the robot against Gazebo ground truth, which is why they exist: odometry alone cannot see a robot that isn't moving. They read parameters (wheel radius, limits, camera FOV) from the xacro files, so they can't drift from the source of truth. Unit tests include guards that robot code never reads the scenario, never consumes Gazebo ground truth, and cognition/planning never import hardware.

### Driving the robot

```bash
# drive forward 5 s at 0.2 m/s
ros2 topic pub --times 100 -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
# ...and STOP it. Gazebo's diff drive holds the last command indefinitely, so a publisher
# that exits leaves the robot driving. (This is why the ESP32 needs a cmd_vel watchdog.)
ros2 topic pub --times 5 -r 10 /cmd_vel geometry_msgs/msg/Twist "{}"
# keyboard: ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Speed limits (0.30 m/s, 1.50 rad/s) are enforced by the simulator from `parameters.xacro`.

### Inspecting

```bash
ros2 interface show fire_resq_interfaces/msg/WorldState
xacro src/fire_resq_description/urdf/fire_resq.urdf.xacro   # expand URDF
ros2 run tf2_tools view_frames                              # TF tree → .gv/.pdf
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
gz model --list                                             # what is in the sim world
gz model -m fire_resq                                       # TRUE pose (ground truth), not odometry
ros2 topic hz /camera/image_raw                             # expect ~22-26 Hz; ~30 requested
```

**Don't trust odometry alone when checking motion.** Odometry is derived from wheel-joint motion, so it reports a robot that is driving even if the body is stuck. In Phase 2 the robot spent a while rested on its magnet with the wheels hovering, and odometry cheerfully said it had moved 1.39 m while `gz model` said 0. Compare against `gz model -m fire_resq`. Ground truth is for tests and evaluation only — no ROS node may consume it.

## Layout

```text
src/fire_resq_interfaces/     msg, srv, action — definitions only, no logic
src/fire_resq_description/    URDF/xacro + a tiny Python reader of parameters.xacro. REUSABLE: no Gazebo content, shared with hardware
src/fire_resq_navigation/     depth->/scan, SLAM Toolbox / AMCL / Nav2 config + launch. Shared with hardware; no Gazebo, no scenario
src/fire_resq_perception/     Phase 5 ✅ colour detector + depth / known-height spatial estimation -> /fire_resq/detections. Shared with hardware
src/fire_resq_world_model/    Phase 6   sole owner of believed state
src/fire_resq_cognition/      Phase 8   VictimPrioritizer implementations
src/fire_resq_planning/       Phase 10  rescue FSM + Nav2 client
src/fire_resq_control/        Phase 9   magnet abstraction, velocity plumbing
src/fire_resq_hardware/       Phase 12  ESP32 bridge — the ONLY hardware-aware package
simulation/                   fire_resq_simulation: worlds, models/{victim,fire}, config/scenarios, launch, gz wrapper,
                              and fire_resq_simulation/ (pure-Python scenario loader/validator/world generator)
tests/unit, tests/sim         see "Tests" above
```

Each skeleton package's `__init__.py` docstring states its responsibility and carries the phase TODOs. Read it before adding code there.

**Robot dimensions live in exactly one place:** `src/fire_resq_description/urdf/parameters.xacro` — including the drive speed/acceleration limits. Wheel radius and separation configure both the Gazebo differential drive and, later, the ESP32 odometry — they must agree, so change them there and nowhere else.

**Frame conventions that odometry depends on** (each was a real bug in Phase 2):
- `base_link` is the **midpoint of the drive axle** (wheels at x = 0). Differential-drive odometry reports that point as `base_link`; anywhere else and every turn produces phantom motion. Everything else is positioned relative to the axle.
- Nothing but the wheels and caster may reach below the wheel contact plane. The magnet's lowest point must stay above `-wheel_radius`, or the robot rests on it and the wheels can't drive.
- Wheel *collision* is a cylinder in the description (faithful) but a single-contact **sphere** in simulation (`wheel_collision` xacro arg, set by the sim wrapper). A flat cylinder turns about its inner edge and the sim robot over-rotates by 18%.

**Simulation runs at a 4 ms physics step** (`empty_world.sdf`). Gazebo's joint-state publisher has no rate limit, so 1 ms meant a 1 kHz `/joint_states`; 4 ms gives 250 Hz. Motion accuracy was verified at 4 ms.

## Navigation (Phase 4)

```text
depth image ──depthimage_to_laserscan──> /scan ──┬─ SLAM Toolbox ─┐
 (87 deg, camera_link, 4 rows)                   └─ AMCL (saved map)─┴──> /map + TF map->odom ──> Nav2 ──> /cmd_vel
                                                                        (exactly ONE publisher of map->odom)
```

`fire_resq_navigation` is shared with hardware: it uses only stable ROS interfaces (depth image + CameraInfo, `/odom`, TF) and exposes `/scan`, `/map`, `map→odom`, the Nav2 actions and costmaps, so the mapping/localization backend can change without touching cognition. Simulation composes it with the arena in `simulation/launch/arena_nav.launch.py` (which sets the camera to 87° via `camera_hfov`; the Phase 2–3 launches stay at 60° so their tests are unchanged).

**Read this before touching SLAM — it was measured, and it overturned the original plan.** SLAM Toolbox scan-matching on the depth-derived wedge *diverges* when it matches every scan (≥ 41° yaw, ≥ 1.3 m after three in-place spins, even at 87°, on noise-free odometry). Skipping **rotation-only scans** makes it work (`minimum_travel_heading: 3.1`), and a **0.3 m translation gate** makes it good. Numbers, the tuning table and the reproduction command are in [docs/Implementation_Plan.md](docs/Implementation_Plan.md) Phase 4. Consequences you must not forget:
- **An in-place spin adds nothing to the map** under the scan-matching preset. The robot must *translate* to map. (Asserted by a test so it cannot be forgotten.)
- `slam_toolbox` needs `base_frame: base_link` (this robot has no `base_footprint`) and `minimum_travel_distance > 0` only for the translation gate; a gate of 0 integrates rotation and diverges.
- **Mapping with trusted simulator odometry is NOT scan-matching SLAM.** `slam_preset:=odom_only` exists only as a labelled baseline (its config header says "NOT SLAM", and a unit test guards that). Never report a result from it as SLAM. Simulated odometry is noise-free, so the SLAM errors here are the matcher's own; hardware will differ.
- Tighter matcher settings and a smaller scan buffer were *worse*, not better; the shipped values were confirmed by repeats.

**Mapped-arena mode (how to navigate).** Map first (SLAM, translation-based route), save the map, then localize with AMCL and run Nav2. Running Nav2 *while scan-matching SLAM is still localizing* is a **known limitation**: SLAM can inject a heading jump (~44° measured; odometry and ground truth agree throughout) at the first scan after a large in-place rotation, which Nav2's rotate-to-heading produces. Odometry-only mapping with the same goals succeeds, so Nav2's configuration is sound. `test_known_limitation_nav2_under_scan_matching_slam` is an `xfail` that will announce itself if this ever gets fixed. The start pose for AMCL is the mission start pose (the map origin), given on `/initialpose`.

**Nav2 is composed from four standard servers** (controller, planner, behaviours, BT navigator + lifecycle manager), not the stock ten-server bringup: Regulated Pure Pursuit controller, NavFn planner, behaviours `spin`/`wait` only, a BT that replans only if the path becomes invalid. **There is deliberately no BackUp, no reversing and no recovery** — the depth wedge only looks forward, so the robot is blind behind itself. Recovery from failed navigation is a later, explicit TODO. Global costmap is a **rolling 10 × 10 m window**: a map-sized one failed every plan at start because the SLAM map only covers what has been *seen* and the robot's own position was just outside it. Costmaps use a **circle** at the robot's circumscribed radius: with the true footprint polygon the planner (a point) and controller (footprint) disagreed and the controller aborted with "collision ahead".

**Robot geometry is not restated in Nav2 config.** `nav2_params.py` renders the footprint radius, speed and acceleration limits and inflation at launch from `parameters.xacro`; `robot_params.py` derives the footprint as the convex hull of chassis, wheels, magnet, camera and caster. Change the robot in the description and navigation follows.

**Not implemented on purpose** (explicit TODOs in code): autonomous exploration (tests supply goals; exploration belongs to the rescue loop), recovery behaviours, sensor noise, loop closure (it made every narrow-FOV configuration worse), an RGB-only visual-SLAM alternative, RealSense integration, dynamic environments. **Encoder-only heading drift** is a recorded hardware risk (Hardware.md TODO); an IMU is deferred, not required.

## Perception (Phase 5)

```text
RGB (+ depth) + CameraInfo + TF ──> ColorBlobDetector ──> 2D detections ──> DepthEstimator | KnownHeightEstimator ──> /fire_resq/detections (target frame)
                                                                  MotionGate: no position while the camera turns
```

One node (`perception_node`), a library behind two interfaces (`Detector`, `SpatialEstimator`); `fire_resq_perception` is shared with hardware and knows nothing of Gazebo or the scenario. The class colours and the size priors live in `config/perception.yaml`; the floor offset is read from `parameters.xacro` at launch. Numbers and the full limitations list are in [docs/Implementation_Plan.md](docs/Implementation_Plan.md) Phase 5. What you must not forget:
- **The RGB-only estimator uses the blob's TOP edge, not its base.** The camera is 8.75 cm above the floor, so a base ray meets the floor at a grazing angle: measured 13–44 cm error on victims (57 cm on the fire) against 1–4 cm for the top edge. Don't "fix" it back to a ground plane. Its error grows with range² (1 px ≈ 4 cm at 3 m).
- **Positions are withheld while the camera turns (> 0.1 rad/s) or drives fast (> 0.3 m/s)** because RGB, depth and TF are offset in time; the 2D detection is still published with `position_valid=false`. Measured, the harm of not gating is small (~1–2 cm at 0.4 rad/s for these objects), so the thresholds are parameters, not law.
- **A blob touching the image border gets no position**, so objects closer than ~0.5–1 m are 2D-only. The rescue approach must use the world model's stored position for the last metre, not live perception.
- **`Detection.source_backend` is provenance only.** Cognition/planning/world model must not read it (unit guard). Depth vs RGB-only is invisible above perception.
- **Colour is the whole MVP detector** (the Phase 3 contract). Priors (heights, axis offsets) describe the current victim/fire models and are unit-tested against the SDFs; the fire's axis offset was tuned from measurement (0.085 left a −3 cm bias). A new fire model needs new priors.
- The node is single-threaded with the TF listener on its own thread. A `MultiThreadedExecutor` cost 1.24 cores in rclpy wait-set bookkeeping; this costs ~0.5.
- Positions are verified in `odom` only (no SLAM in the arena launch). `map` is the same code with `target_frame:=map`; not yet run against a live `map→odom`.
- Not implemented on purpose (explicit TODOs): position uncertainty, active perception, `YoloDetector`, occlusion handling, obstacle/safe-zone detection, sensor noise, real-camera calibration.

## The rescue arena (Phase 3)

**The scenario YAML is the only place object placement lives** (`simulation/config/scenarios/default.yaml`; schema documented in its header, unknown keys rejected). It is simulator ground truth: **no robot node may read it** (a unit-test guard enforces this) — the robot learns the world through perception → world model. The world SDF is *generated* from it at every launch (byte-identical output for identical YAML), by injecting the scenario into the shared base world so physics/plugins are defined once. Layout rules (wall/obstacle clearance, victim spacing > the world model's association gate, nothing overlapping, no victim in the safe zone) are validated at load, so a bad generated scenario fails loudly. To try another layout, write another YAML.

World frame: arena centred on the origin, +x right, +y up, yaw CCW; the robot's `odom` frame starts at its spawn pose (`Scenario.world_to_odom` / `odom_to_world`).

| Entity | World (x, y) | Role in the design |
| --- | --- | --- |
| robot start | (−1.9, −1.9), yaw π/4 | inside the safe zone, facing the arena |
| safe zone | centre (−1.9, −1.9), 1 × 1 m | green floor slab, visual only |
| fire | (0.9, 0.6) | orange emissive cone, static obstacle |
| `victim_1` | (−0.7, −1.3) | **nearest** the robot, **far** from the fire |
| `victim_2` | (1.3, 1.6) | **farthest** from the robot, **next to** the fire |
| `victim_3` | (−1.5, 1.4) | mid-distance, behind a partition — hidden from the start, needs a detour |
| `obstacle_1/2/3` | partition 1.4×0.15 at (−1.3, 0.5); blocks 0.5×0.5 at (0.1, −1.1), 0.6×0.3 at (1.4, −1.2) | navigation complexity |

Nearest-first and risk-aware prioritisation give *different answers* here (victim_1 vs victim_2), so the Phase 8/11 comparison has something to bite on. `victim_N` ids are ground truth for evaluation; all victims look identical (like the physical ones), so the world model tells them apart by position.

**Perception colour contract** (measured from rendered frames; every non-target pixel has saturation ≤ 18, so nothing else can match): victim = blue, hue 109–110 (torso/head, dark: V 100–205); fire = orange, hue 13–21, S 255, V ≥ 236, emissive so lighting-independent; safe zone = green, hue 68, S ~153. Class masks used by the tests: fire `H≤30,S≥200,V≥200`; victim `100≤H≤125,S≥100,V≥40`; safe zone `55≤H≤80,S≥100,V≥60`. The fire model is replaceable — the scenario names it (`fire: {model: fire}`) and only this colour/obstacle contract is relied on.

**Each victim has a steel attachment ring** (bottom 55 mm, r 56 mm) at the electromagnet's height, so the magnet meets metal from any approach bearing; single dynamic link `base_link` for the Phase 9 detachable joint. Victims are dynamic bodies and were verified not to drift, sink or topple.

**RGB and depth are separate sensors whose content is offset by ~40 ms** (measured: ~10 px at 0.4 rad/s even when timestamps match; the depth stream is exactly time-aligned to the true pose, so it is the RGB image that is late). Never fuse a single depth pixel with an RGB blob from a *moving* camera — it can land on the wall behind a thin object. The tests look while stationary (`survey()`: turn a step, stop, sample). Phase 5 perception does the same (see "Perception" below).

**The description/simulation split is load-bearing.** `fire_resq_description` has no `<gazebo>` tags; `simulation/urdf/fire_resq_gazebo.urdf.xacro` includes it and layers simulator content on top. Keep it that way — it is what makes the sim-to-hardware swap a driver change.

## Build order

**Next up: Phase 6** — the world model (stable beliefs from noisy detections: entity registry, association, smoothing, confidence decay, `/fire_resq/world_state`; it depends on Phases 4 and 5, so it will also need positions in `map`). Phases 0–5 are done; [docs/Implementation_Plan.md](docs/Implementation_Plan.md) §13 defines each phase's scope, verification, and definition of done. Do not implement ahead of the current phase — the TODO markers in each package mark where later work attaches.

Work down the simulation ladder in [docs/Simulation.md](docs/Simulation.md) — each stage is verifiable on its own and later stages assume earlier ones work: robot spawns and drives via `/cmd_vel` → simulated camera/depth + odometry → SLAM map and localization → perception detections → world model ingests detections → decision engine picks a victim dynamically → Nav2 drives to it → simulated electromagnet attach/release → return to safe zone → mark rescued and select the next target.

Simulation must reach the [PRD.md](PRD.md) definition of done before any hardware work. Hardware then follows the six phases in [Hardware.md](Hardware.md) (base → odometry → camera → rescue mechanism → navigation → full integration).

## Hard rules (from the docs)

- **Never hardcode** victim coordinates, rescue order, or rescue paths. Target selection must come from the current world model. The MVP scorer is a transparent weighted utility over fire risk, distance to fire, robot-to-victim distance, accessibility, rescue cost, and perception confidence — behind an interface that allows swapping in other decision models. Evaluation compares it against a nearest-victim baseline.
- **Layer separation:** cognition/planning never touch GPIO or specific hardware. Moving from sim to hardware swaps drivers under stable ROS-level interfaces (velocity commands, odometry, TF, camera/depth streams, electromagnet ATTACH/RELEASE + state).
- **Camera independence:** perception exposes camera-agnostic detections; RGB and RGB-D (RealSense) backends are interchangeable, and cognition must not care which one produced spatial estimates.
- **Frames:** `map`, `odom`, `base_link`, camera frame(s) via TF2; no hardcoded coordinates in cognitive code.
- **No LiDAR dependency.** Laptop runs ROS 2/perception/cognition; ESP32 handles motors, encoders, and the electromagnet MOSFET/relay.
- **Rescue task sequence:** SELECT VICTIM → NAVIGATE → ALIGN → PICK UP → RETURN → RELEASE → MARK RESCUED → REASSESS.
- **Extensions stay as explicit `TODO`s in code**, not implementations, until the MVP works: uncertainty propagation, Bayesian belief updates, active perception, dynamic fire risk, battery/resource awareness, algorithm comparison, recovery from failed detection/pickup/navigation. A watchdog/failsafe is required before autonomous physical testing.
- **Avoid unnecessary complexity until the MVP works** — this is stated as a project constraint, not just a style preference.
