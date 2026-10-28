# FireResQ: demo day cheat sheet

## The two commands

**Simulation.** Open the Ubuntu (WSL) terminal and run:

```bash
cd ~/FireResQ && ./run_sim.sh
```

A Gazebo window opens. The robot looks around, ranks the three victims, and carries each one to the green safe zone, closest to the fire first. It takes about 2 minutes. Run `./run_sim.sh 42` to replay layout 42; a plain `./run_sim.sh` makes a new random layout.

**If the terminal shows progress but no Gazebo window appears**, WSL's window bridge has stalled. Run `wsl --shutdown` in PowerShell, reopen Ubuntu, and run the command again. Do this once before presenting.

**Real robot.** Open Windows PowerShell and run (the password is `pi`):

```powershell
ssh -t pi@pi.local "cd ~/victimbot && python3 -u rescue3.py"
```

- **Before you run it:**
  - Put **fresh AAs** in the motor pack. On weak cells the wheels spin in the air but the car cannot roll.
  - Place the robot on the start spot. The red box and all three yellow-green V-note victims must be in view.
- **While it runs:**
  - Live view, map and the robot's reasoning are at **http://pi.local:8090**.
  - Each time it comes back and stops for 5 s, take the victim off the magnet.
  - Ctrl+C stops it.
- **Rehearsal:** add `--look` at the end of the command. The robot only looks, ranks and explains; nothing moves.

## What makes it cognitive: the loop

**Perceive → build a world model → reason / decide → act → check the result → reassess.**

Nothing is hard-coded. Victim positions, the fire and the rescue order all come from what the camera sees at run time. Move the objects and the plan changes.

## The decision (the "cognition")

**Rule: rescue the victim in the most danger first.** Danger is measured by distance to the fire, which is triage.

In the full ROS 2 system (`fire_resq_cognition`) this is a transparent weighted utility. Every candidate victim gets a score:

```text
U = 3.0·risk + 1.0·fire_proximity + 1.0·reach + 0.5·accessibility + 0.5·confidence − 1.0·cost
```

| Term | Formula | Meaning |
| --- | --- | --- |
| risk | 1 if d_fire ≤ 1 m, else e^−(d_fire − 1)/1 | immediate danger, decays exponentially |
| fire_proximity | 1 − d_fire / D | nearer the fire is more urgent (D = arena diagonal 7.07 m) |
| reach | 1 − path(robot→victim) / D | a shorter trip out is cheaper |
| accessibility | straight-line ÷ planned path | 1 is a direct route; smaller means a detour |
| confidence | perception score | how sure we are it is a victim |
| cost | path(robot→victim→safe zone) / 2D | the whole rescue trip |

- **Weights are priority tiers:** safety first (3), practicality second (1), tie-breakers third (0.5).
- **Which victim:** the one with the highest U. Every term's contribution is logged, so each decision is explainable.
- **Baseline for comparison:** nearest-victim-first. In the default arena the two pick different victims; the utility saves the one beside the fire.
- **Eligibility:** only victims the planner can reach out *and* back are scored.
- **Reassessment:** after every rescue, or whenever the world changes, it decides again.

The demos use the same rule in its simplest form: sort the victims by distance to the fire, ascending.

## Algorithms: simulation (`quick_demo/`)

| Step | Algorithm |
| --- | --- |
| Detect | HSV colour segmentation: victim = blue (H 100–125), fire = orange (H ≤ 30, S ≥ 200) |
| Locate | Depth back-projection: X = (u − cx)·Z / fx, Y = (v − cy)·Z / fy, then TF to the odometry frame |
| Withhold | No position while the camera turns (> 0.1 rad/s), because RGB and depth are ~40 ms apart |
| World model | Greedy clustering: a detection joins a track within 0.25 m (running mean); a track needs ≥ 3 sightings |
| Decide | Sort the victims by ‖victim − fire‖ |
| Move | Rotate, then go. Heading P-control ω = 3·e_θ, speed v = min(v_max, 2·distance left) |
| Pick up | Magnet contact distance = magnet_x + magnet_len/2 + ring_radius = 0.131 m. The magnet attaches only within 2 cm of contact |
| Verify | Success is the magnet's own *attached* state, not "command sent". After release, Gazebo's ground truth confirms the victim is inside the zone |
| Layout | Random, but validated: every victim is visible, and every drive path is clear of the fire and the other victims |

## Algorithms: real robot (`victimbot/`, Pi 3B + Arduino, no encoders)

| Step | Algorithm |
| --- | --- |
| Detect | HSV thresholds: victim = yellow-green V sticky note (H 30–62); fire = vivid red (H 0–4 or 174–179, S ≥ 170) |
| Reflections | The glossy floor mirrors each note just below it. A smaller blob directly under a bigger one is dropped |
| Bearing | θ = atan((W/2 − u) / f), with f = (W/2) / tan(HFOV/2), HFOV = 41.4° |
| Distance (monocular) | Ground-plane geometry: r = h_cam · f / (row_contact − row_horizon) |
| Self-calibration | All victims are the same size, so row_horizon = row_contact − (h_cam / h_victim) · h_pixels, re-measured every look. If the camera tilts, the distances stay right |
| Sensor fusion | The sonar is aimed at the fire with the servo. Its range replaces the camera's only if they agree (within 30 % on the box, within 12 cm on a victim) |
| Decide | Rank by the distance from each victim to the fire box's base (point-to-segment distance) |
| Align | Turn pulses until the victim is within ±4° of the image centre. The pulse length is learned: longer if the victim didn't move in the picture, shorter if it overshot |
| Approach | Hop forward and re-centre. Speed is learned from vision: v ← ½v + ½·Δr / (t − t_dead). The motor dead time is learned too: a hop that didn't shrink the range lengthens it |
| Pick up | Drive the last few cm onto the magnet; the sonar stops it once something is on the magnet |
| Verify | Back off 10 cm and look. A note still standing ahead means a miss, so it tries again |
| Return | Replay its own moves in reverse to the start square: dead reckoning by action replay, since there are no encoders |

## What we used

| Part | What it is | Job |
| --- | --- | --- |
| Raspberry Pi 3B | the robot's computer (Python, OpenCV) | vision, decisions, web dashboard |
| Arduino Uno + L293D motor shield | motor controller, USB serial to the Pi | drives 4 DC motors; stops them if the Pi goes silent for 400 ms (safety watchdog) |
| OV5647 ribbon camera | the only camera (no depth) | finds victims and fire; mounted sideways, rotated back in software |
| HC-SR04 ultrasonic on an MG90S servo | sonar the servo can point | checks distances; tells when something is on the magnet |
| Magnet at the front | permanent magnet | picks up the victims (toy DC motors, so metal) |
| 8×AA (6 V) | motor batteries | the Pi has its own supply |
| Web dashboard, port 8090 | `http://pi.local:8090` | live camera with detection boxes, map, actions, the robot's reasoning |
| ROS 2 Jazzy + Gazebo Harmonic, on Ubuntu 24.04 (WSL2) | the simulation | the full system: SLAM Toolbox, AMCL, Nav2, perception, world model, cognition, mission state machine |

## Viva Q&A

**Why is the simulation so simple, with no noise?**
We also built a full 5 × 5 m arena with walls, obstacles and a victim hidden behind a partition (`./run_demo.sh`). It maps the arena with SLAM, plans paths with Nav2 and rescues all three, but it takes several minutes. For the demo we use a small 3 × 3 m arena that shows the same decision in about 2 minutes. You're right that the sensors are noise-free in simulation; adding noise is on our to-do list.

**What if there were walls and obstacles?**
That's the full version: SLAM builds a map and Nav2 plans a route around obstacles. Its decision also counts how reachable each victim is (path length, detours).

**What if something else looked like a victim, e.g. another blue object?**
Our detector only looks at colour, so it would be fooled. We reduce false hits: an object must show up in several pictures, small specks are ignored, and on the real robot anything behind the fire is ignored. The proper fix is a trained object detector (YOLO is planned); we haven't trained one.

**Why do you need a robot? Can't the victims run away?**
Real victims may be unconscious, injured or trapped, or the area may be full of smoke where people can't go. Our car is a small model. The same idea (see, rank by danger, rescue, re-check) applies to rescue robots or drones in building and forest fires.

**How do you measure distance without a depth camera?**
With pinhole-camera geometry. The camera sits at a known height, so the lower an object's base is in the picture, the closer it is: r = h_cam · f / (row − horizon). The horizon row comes from the victims themselves: they're all the same size, so their height in pixels tells us the scale. The sonar double-checks it.

**How does it decide whom to save first?**
Most danger first: it ranks the victims by distance to the fire, saves the nearest-to-fire one, comes back to the safe zone, and looks again. The full system uses the weighted score U above; the simple robot just compares fire distance.

**Does the real robot do SLAM?**
No, only the simulation does. The real car has no wheel encoders. It turns until the victim is centred, measuring each turn with the camera, and it returns by replaying its moves backwards.

**How do you know a pick-up worked?**
It checks with its sensors instead of assuming. In simulation, the magnet reports "attached". The real robot backs off and looks; if the victim is still on the floor, it tries again.

**What stops it from crashing or running away?**
The Arduino watchdog stops the motors if commands stop for 400 ms. Ctrl+C stops it.

**How did you test it?**
The simulation has about 490 automated tests that check the robot against Gazebo's true positions. The quick demo was checked the same way on 5 layouts: all 3 victims rescued, in the right order.

**What would you add next?**
Sensor noise and uncertainty (Bayesian beliefs), a trained victim detector, moving to get a better view (active perception), and fire that spreads.
