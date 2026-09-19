# FireResQ

**Cognitive Search-and-Rescue Robot for Intelligent Victim Prioritization**

FireResQ is a ROS 2 cognitive robotics project in which an autonomous mobile robot perceives a controlled fire-emergency environment, builds a spatial representation, reasons about multiple victims, selects a rescue target, navigates to it, retrieves it using an electromagnet, returns it to a safe zone, and reassesses the situation.

## Core Loop
```text
Perception
→ SLAM / Localization
→ World Model
→ Reasoning
→ Decision
→ Planning
→ Action
→ Reassessment
```

## Documents
- `PRD.md` — complete scope and requirements.
- `Architecture.md` — software architecture and ROS 2 boundaries.
- `Hardware.md` — physical components and integration.
- `docs/Cognitive_Model.md` — cognition design.
- `docs/Simulation.md` — simulation plan.
- `docs/Hardware_Integration.md` — simulation-to-hardware transition.

## Important Rule
Do not hardcode victim coordinates, rescue order, or a predefined rescue path in the final system. The robot must make decisions from its current world model.

## Development Strategy
1. ROS 2 environment.
2. Gazebo rescue world.
3. Differential-drive robot.
4. Sensors/perception.
5. SLAM/localization.
6. World model.
7. Cognitive decision engine.
8. Nav2 planning.
9. Simulated electromagnet.
10. Complete rescue loop.
11. Physical hardware integration.

## Cognitive Roadmap
MVP:
- Perception.
- SLAM/localization.
- World model.
- Multi-factor victim prioritization.
- Autonomous navigation.
- Rescue/reassessment.

Future:
- TODO: Bayesian uncertainty.
- TODO: active perception.
- TODO: information-gain decisions.
- TODO: battery-aware reasoning.
- TODO: decision algorithm comparison.
