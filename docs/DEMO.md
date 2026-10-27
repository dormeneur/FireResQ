# The bottle demo

The robot finds bottles with its camera, drives to the nearest one, waits 5 seconds, then goes to the next, until it has visited every bottle it can find. Everything it sees and decides is shown live on a dashboard on the PC.

```text
./demo.sh             run it on the real robot
./demo.sh dry         same, but the motors are never commanded (safe on a desk)
./demo.sh rehearse    no robot at all: a pretend robot in a pretend room
./demo.sh motors      one-time wiring check, wheels OFF the ground
./demo.sh deploy      send driver or config changes to the robot
./demo.sh stop        stop everything, PC and robot
```

Run it from the `FireResQ` folder. It asks for the Pi's password once, then reuses the connection for 10 minutes.

---

## Status (2026-10-13): what has and has not been tried

| | |
| --- | --- |
| Finding the black bottle with the real camera | ✅ tried, 0.40 m, 7–8 fps |
| The first version driving the real robot | ✅ run on the floor by the maintainer ("working well"), with three complaints: it should pivot instead of spin, left turns were weak, and it stopped too far from the bottle |
| The changes made for those complaints (turning styles, harder left turns, the closer stop) | ⚠️ in rehearsal and tests, and the style switch on the real driver with the motors idle. **Not yet driven on the floor** |
| Coloured bottles, and obstacles, with the real camera | ❌ not tried by the developer: only the black bottle was in front of the camera |

So run it on the floor once more before the audience arrives, and pick the turning style then (step 2).

---

## 1. Once: tell the robot which motor is which wheel

Nobody knows yet which shield terminal drives which wheel, or which way each motor turns.

1. **Lift the robot so all four wheels are off the ground.** Put the AA batteries in.
2. `./demo.sh motors`
3. Each motor spins in turn. Answer which wheel moved (`fl`, `fr`, `rl`, `rr`) and whether it rolled forward (`y` / `n`).

The answers are saved to `src/fire_resq_hardware/config/robot.yaml` and sent to the robot.

## 2. Check it drives the right way

1. Wheels still off the ground: `./demo.sh`
2. The dashboard opens at <http://localhost:8080>. Both pills at the top must be green (`camera`, `robot connected`).
3. Hold the arrow keys (or the on-screen arrows). ▲ must roll all wheels forward; ◀ must turn the robot to its left.
4. Put it on the floor and try again. If it does not move, raise **Drive speed**. If it will not turn on the spot, raise **Turn power**.
5. **Pick the turning style** (the drop-down under the arrows; it switches at once, try each with ◀ and ▶):

| Turning style | To turn left, the wheels do this (right is the mirror image) |
| --- | --- |
| Pivot: inner side backward, outer side stopped | left wheels backward, right wheels stopped |
| Pivot: outer side forward, inner side stopped | right wheels forward, left wheels stopped |
| Rear wheels only, front wheels stopped | rear-left backward, rear-right forward |
| Front wheels only, rear wheels stopped | front-left backward, front-right forward |
| Spin: one side backward, the other forward | left wheels (M1, M2) backward, right wheels (M3, M4) forward: exactly the old `F/B/L/R` sketch. **The default** ("as set on the robot") |

The choice is remembered. If left turns are still weaker than right turns, raise **Left turns: extra push** (1.25 = 25 % harder).

Four AA cells through this motor shield leave little torque for turning on the spot. If it cannot turn even at full turn power, use fresh batteries or a smoother floor.

## 3. Run the demo

1. Place the bottles (see the rules below) and put the robot down **facing at least one of them**, with clear floor straight in front of it.
2. `./demo.sh`, then press **START** (or Enter).
3. **STOP** (or Space) stops it at once. Ctrl+C in the terminal stops the demo and the robot.

What you will see:

- **Camera:** a box round every bottle with its colour and distance; the one it is going to has white corners. Orange dashes mark where the floor ends (walls, obstacles); they turn red when something is in the robot's way. A big number counts down the 5 seconds.
- **Map:** the robot (white arrow), where it has been (blue line), the bottles it remembers, a tick on the visited ones, and the walls and obstacles it has seen.
- **Bottle cards and the log:** every decision in words ("A closer bottle: going to the blue one").
- **voice** (top right): the browser reads the log aloud. Not tested; try it before relying on it.

When it has visited everything it can find, it turns a full circle to look for more, gives a small wiggle, and stops.

---

## Rules for the bottles and the room

- **Every bottle a different colour.** The robot tells bottles apart by colour. Good: red, yellow, green, blue, pink, black. Bad together: red + orange, orange + yellow.
- **Solid colours.** Transparent, white, grey and shiny steel bottles are not seen. Wrap them in coloured paper.
- **Standing up, on the floor, 10–40 cm tall.**
- **Not one behind another** as seen from the start, and at least 40 cm apart.
- **A plain floor**, a different colour from the bottles. Nothing else brightly coloured or black at floor level (shoes, bags, chair legs).
- **Good, steady light.** In dim light the camera drops to 7 fps and dark walls start to look like black bottles.
- **Same Wi-Fi as today** for the PC and the Pi (a phone hotspot is the safe choice). College Wi-Fi often blocks the kind of traffic ROS uses to find the robot.

## Tuning on the day

All on the dashboard, saved automatically (`demo_settings.json`; delete it to go back to the defaults).

| If you see | Change |
| --- | --- |
| A coloured bottle has no box | lower **Colour: how vivid counts as coloured** |
| The wall or floor gets a coloured box | raise it |
| The black bottle has no box | raise **Black: how dark counts as black** |
| Shadows or dark walls get a box | lower it, or switch **Look for black bottles** off |
| A grey box with a word on it | that is a rejected candidate and the word is why (`shape`, `too big`, `not on the floor`, `half out of view`) |
| Distances are clearly wrong | measure the lens height and set **Camera lens height**; move **Horizon line** so the dashed line sits where a far-away floor would end |
| It stops too far from the bottle | raise **How close to get** (how much of the picture's width the bottle must fill). For closer than the camera can judge, add a little **Then drive on blind for** |
| It bumps the bottle | lower **How close to get**, or **Drive speed** |
| Left turns weaker than right turns | raise **Left turns: extra push** |
| Orange/red dashes all over a clear floor | press **Re-learn floor** with clear floor in front; raise the two **Obstacle:** tolerances; or switch **Avoid obstacles** off |
| It keeps turning away from the bottle | the wiring check is wrong: `./demo.sh motors` again |

## If something goes wrong

| Symptom | Cause and fix |
| --- | --- |
| `Could not start the drivers` | Pi off, different Wi-Fi, or a new address: `ROBOT=pi@<address> ./demo.sh` |
| `no camera` / `robot not answering` pills stay red | wait 15 s after starting; then `./demo.sh deploy`. On another network the PC may not be able to find the robot at all |
| Robot does not move, pills green | batteries; raise **Drive speed**; make sure it is not `./demo.sh dry` (yellow `DRY RUN` pill) |
| Page says `Demo not running` | the terminal running `./demo.sh` was closed: run it again |
| Port 8080 busy | `./demo.sh stop`, then run again |

## If the robot will not behave on the day

`./demo.sh rehearse` shows the same dashboard and the same behaviour with a pretend robot: four bottles (one behind the start, one hidden behind a box) and an obstacle to go round. It needs no hardware. `./demo.sh dry` shows the real camera and real detection with the robot standing still, and you can carry the robot to the bottles by hand.

---

## How it works, and what it is not

- **Bottles** are saturated blobs (grouped by hue) or very dark blobs that are upright, solid and standing on the floor. **Distance** comes from where the bottle's base is in the picture, assuming a flat floor. It depends on the lens height and tilt, which are estimates.
- **Obstacles and the stopping distance now come from two IR sensors** (2026-10-16; wired to the Pi, one pointing forward-left, one forward-right), not the camera: each is a plain "something is close" switch, no distance, no colour. They decide whether something is in the way and whether the target has been reached; the camera still decides WHICH coloured object to go to. If the IR sensors stop reporting, the robot falls back to the camera's floor-colour obstacle test (below) so losing them makes it worse, not blind.
- **The camera's own obstacle test (fallback only)** is whatever does not look like the floor just in front of the robot. It is a colour test, not a range sensor: a floor-coloured object is invisible to it, and a large dark mark on the floor counts as an obstacle. The robot also cannot see beside or behind itself.
- **The map is not SLAM.** The robot has no encoders, lidar or depth camera. Its position is dead reckoning from the power sent to the motors, with its turning corrected by watching the bottles. Position errors are never corrected, so the map drifts. For real SLAM the robot needs a lidar or a depth camera ([Hardware.md](Hardware.md) §5).
- **It learns how the robot turns** while running: the first turn pulses are a guess, later ones are scaled by how far the bottle moved in the picture. If left and right are swapped it notices and reverses its turns, and says so. If a turn pulse leaves the picture unchanged, it makes the pulses longer and then stronger.
- **A remembered bottle it can no longer see** (something is in front of it) is not given up at once: after a full turn without seeing it, the robot drives to where it remembers it, going round what is in the way, and looks again.

Code: `src/fire_resq_demo` (start with its `__init__.py`). Tests: `tests/unit/test_demo_lib.py`.
