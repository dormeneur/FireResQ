# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Current state

Design-docs only — no packages, code, or build system exist yet. `src/`, `simulation/`, and `tests/` are planned but not created. The docs are the source of truth:

- [PRD.md](PRD.md) — scope, MVP checklist, evaluation metrics, out-of-scope, definition of done.
- [Architecture.md](Architecture.md) — ROS 2 layers, custom interfaces, package layout, sim→hardware rule.
- [Hardware.md](Hardware.md) — component list, build phases, sim-to-hardware mapping.
- [docs/Cognitive_Model.md](docs/Cognitive_Model.md) — world state and the victim-priority model.
- [docs/Simulation.md](docs/Simulation.md) — the 10-stage simulation bring-up ladder.
- [docs/Hardware_Integration.md](docs/Hardware_Integration.md) — the high/low-level interface boundary.

When a design decision changes, update the owning doc in the same change as the code — the docs are meant to stay authoritative, not become stale.

## What the project is

FireResQ: a ROS 2 cognitive search-and-rescue robot (academic prototype, not a real firefighting system). A differential-drive robot perceives a ~5×5 m arena holding a fire, three lightweight metal victims, obstacles, and a safe zone; maps it with SLAM; maintains a world model; decides *which victim to rescue next*; navigates with Nav2; lifts the victim with an electromagnet; releases it in the safe zone; then reassesses.

Loop: Perception → SLAM/Localization → World Model → Reasoning → Decision → Planning → Action → Reassessment.

The interesting part is the decision layer. Everything else exists to feed it a world model and execute what it picks.

## Environment

This checkout is on WSL2 (Ubuntu 24.04) and the toolchain is already installed and usable here:

- ROS 2 Jazzy at `/opt/ros/jazzy` (includes Nav2, SLAM Toolbox, `ros_gz_sim`)
- Gazebo (`gz sim`) 8.15.0 — Harmonic, via `gz_tools_vendor`
- `colcon`, `python3`, OpenCV (`cv2`) 4.6.0

Not installed: `ultralytics` (or any YOLO runtime) — install it in a venv when perception work starts.

## Commands

Standard ROS 2 Jazzy / colcon workflow. Nothing is built yet, so these are no-ops until `src/` has packages; don't invent project-specific wrappers that don't exist.

```bash
source /opt/ros/jazzy/setup.bash          # every new shell
colcon build --symlink-install            # from repo root
source install/setup.bash                 # after each build

colcon build --packages-select fire_resq_cognition   # one package
colcon test  --packages-select fire_resq_cognition
colcon test-result --verbose                         # see failures

# one test / one case (faster than colcon during iteration)
python3 -m pytest src/fire_resq_cognition/test/test_priority.py -k victim_scoring -v
```

Launch files land in `simulation/launch/` and run with `ros2 launch <pkg> <file>`. GUI tools (Gazebo, RViz) need a working X/Wayland display from WSL2.

## Planned layout

```text
src/fire_resq_{interfaces,perception,world_model,cognition,planning,control,hardware}/
simulation/{worlds,models,config,launch}/
tests/
docs/
```

`fire_resq_interfaces` holds the semantic messages: Detection, VictimState, WorldState, RescueTarget, RescueAction (definitions may evolve). Prefer custom interfaces over stuffing semantics into stock message types.

## Build order

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
