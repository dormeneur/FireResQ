# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Current state

Design-docs only — no code, packages, or build system exist yet. The source of truth is:
- [PRD.md](PRD.md) — scope, MVP, definition of done, out-of-scope.
- [Architecture.md](Architecture.md) — ROS 2 layers, interfaces, planned package layout.
- [Hardware.md](Hardware.md) — components, build phases, sim-to-hardware mapping.
- [docs/](docs/) — cognitive model, simulation stages, hardware integration boundary.

When code is added, update this file with real build/test commands. Don't invent commands in the meantime.

## What the project is

FireResQ: a ROS 2 cognitive search-and-rescue robot (academic prototype). A differential-drive robot perceives a ~5×5 m arena with a fire and three metal victims, maps it with SLAM, maintains a world model, picks which victim to rescue next, navigates with Nav2, lifts the victim with an electromagnet, drops it in the safe zone, then reassesses.

Loop: Perception → SLAM/Localization → World Model → Reasoning → Decision → Planning → Action → Reassessment.

Target stack: Ubuntu 24.04, ROS 2 Jazzy, Gazebo, RViz, Nav2, SLAM Toolbox (or equivalent), TF2, OpenCV, a YOLO-family detector. Simulation must be complete before hardware work. (This checkout is on Windows; ROS 2 builds/runs are expected on Ubuntu.)

## Planned layout

```text
src/fire_resq_{interfaces,perception,world_model,cognition,planning,control,hardware}/
simulation/{worlds,models,config,launch}/
tests/
docs/
```

Custom interfaces planned in `fire_resq_interfaces`: Detection, VictimState, WorldState, RescueTarget, RescueAction (definitions may evolve).

## Hard rules (from the docs)

- **Never hardcode** victim coordinates, rescue order, or rescue paths. Target selection must come from the current world model. The MVP scorer is a transparent weighted utility over fire risk, distance to fire, robot-to-victim distance, accessibility, rescue cost, and perception confidence — behind an interface that allows swapping in other decision models. Evaluation compares it against a nearest-victim baseline.
- **Layer separation:** cognition/planning never touch GPIO or specific hardware. Moving from sim to hardware swaps drivers under stable ROS-level interfaces (velocity commands, odometry, TF, camera/depth streams, electromagnet ATTACH/RELEASE + state).
- **Camera independence:** perception exposes camera-agnostic detections; RGB and RGB-D (RealSense) backends are interchangeable, and cognition must not care which one produced spatial estimates.
- **Frames:** `map`, `odom`, `base_link`, camera frame(s) via TF2; no hardcoded coordinates in cognitive code.
- **No LiDAR dependency.** Laptop runs ROS 2/perception/cognition; ESP32 handles motors, encoders, and the electromagnet MOSFET/relay.
- **Rescue task sequence:** SELECT VICTIM → NAVIGATE → ALIGN → PICK UP → RETURN → RELEASE → MARK RESCUED → REASSESS.
- **Extensions stay as explicit `TODO`s in code**, not implementations, until the MVP works: uncertainty propagation, Bayesian belief updates, active perception, dynamic fire risk, battery/resource awareness, algorithm comparison, recovery from failed detection/pickup/navigation. A watchdog/failsafe is required before autonomous physical testing.
