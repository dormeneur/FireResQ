# FireResQ: demo day cheat sheet

## The two commands

**Simulation.** Open the Ubuntu (WSL) terminal and run:

```bash
cd ~/FireResQ && ./run_sim.sh
```

A Gazebo window opens. The robot looks around, ranks the three victims, and carries each one to the green safe zone, closest to the fire first. It takes about 2 minutes. Run `./run_sim.sh 42` to replay layout 42; a plain `./run_sim.sh` makes a new random layout.

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

## Short answers to likely questions

- **Why cognitive, not just reactive?**
  - It keeps a world model (victims, fire, safe zone), reasons over it with an explicit utility, and acts.
  - It checks the outcome and re-plans after each rescue.
- **Why fire distance?** Triage: the victim nearest the fire is in the most danger. A nearest-first robot would save the easy one first.
- **How do you know a pick-up worked?** From the sensors, not the command: the magnet state in simulation, and the back-off-and-look check on the real robot.
- **No LiDAR, no encoders?**
  - Distance comes from camera geometry calibrated by known object size, plus the sonar.
  - Motion is closed-loop on vision.
- **What would you add next?**
  - Bayesian belief updates.
  - Uncertainty in the decision.
  - Active perception (moving to see better).
  - A dynamic fire-spread model.
