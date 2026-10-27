# How to make the complete robot

Step by step, from the robot on your desk to the full autonomous rescue. Short on purpose: wherever more detail exists, this page links to it. Rewritten 2026-10-13 for the robot as built ([Hardware.md](Hardware.md)).

```text
PC ("station")  ── Wi-Fi, ROS 2 ──  Raspberry Pi 3B  ── USB ──  Arduino Uno + L293D motor shield ── 4 motors (4× AA)
perception, decisions,              camera + motor drivers       base_controller firmware
navigation, RViz                    └─ USB ── Logitech C270
```

---

## Where things stand (read this first)

| Part of the system | Status |
| --- | --- |
| All the "brain" software (perception, world model, decision, mission, Nav2 setup) | ✅ Done; verified in simulation |
| ROS 2 on both computers, talking over Wi-Fi | ✅ Done (Docker; steps 1–2) |
| Arduino firmware: per-motor speed, watchdog | ✅ Flashed and tested on the board |
| Pi drivers: camera → PC, `/cmd_vel` → motors, `/odom` | ✅ Running; tested with the **motors unpowered** |
| Camera → perception on the PC | ✅ Running (`/fire_resq/detections`); colours and sizes not tuned for real objects yet |
| Which motor is which wheel; real speeds | ❌ **Your next step** (steps 3–4): needs the batteries in and your eyes |
| Knowing where the robot is (`map`) | ❌ **No sensor for it yet.** No encoders, no depth camera. Needs a decision: [Hardware.md](Hardware.md) §5 |
| Picking victims up | ❌ No electromagnet fitted: [Hardware.md](Hardware.md) §6 |

Until the last two exist the robot can be **driven from the PC** and can **see and place victims and fire relative to itself**. It cannot run the autonomous mission.

**What it can do autonomously today is the bottle demo** (find coloured bottles, visit each one, live dashboard): one script, `./demo.sh`, explained in [DEMO.md](DEMO.md). `./demo.sh motors` also does step 3 below for you.

---

## 1. One-time setup

**PC** (needs Docker; nothing else is installed on the host):

```bash
cd FireResQ
docker build -f docker/station.Dockerfile -t fire_resq/station .     # ~7 GB, once
docker/station.sh colcon build --symlink-install                     # builds the workspace inside the container
```

**Pi** (Raspberry Pi OS; needs Docker and your user in the `docker` group):

```bash
sudo apt install -y docker.io && sudo usermod -aG docker $USER       # then log out and in
```

**Arduino** — flash the base firmware (from the PC, with `arduino-cli`; the Uno stays plugged into the Pi):

```bash
arduino-cli lib install "Adafruit Motor Shield library"
arduino-cli compile -b arduino:avr:uno --output-dir /tmp/fw src/fire_resq_hardware/firmware/base_controller
scp /tmp/fw/base_controller.ino.hex pi@<robot-ip>:
ssh pi@<robot-ip> 'sudo apt install -y avrdude; docker stop fire_resq_robot; \
    avrdude -p atmega328p -c arduino -P /dev/ttyUSB0 -b 115200 -U flash:w:base_controller.ino.hex:i'
```

The sketch that was on the Uno before (`fixed_test_movement`, the `F/B/L/R/S` one) is saved byte-for-byte in [`firmware/backup/arduino_original_flash.hex`](../src/fire_resq_hardware/firmware/backup/arduino_original_flash.hex). To go back, flash that file with the same `avrdude` command. **The old Pi scripts (`autonomous.py` …) do not work with the new firmware**: it speaks a different protocol at a different baud rate, on purpose (step 6).

---

## 2. Start the robot

```bash
# PC: copy the driver packages to the Pi, build them there, start them (first time ~10 min, then ~30 s)
docker/deploy_robot.sh pi@<robot-ip>

# PC: a shell that talks to the robot
docker/station.sh
ros2 topic list                                   # /camera/image_raw/compressed  /cmd_vel  /odom  /tf ...
ros2 launch fire_resq_hardware station.launch.py  # decoding + robot TF + perception
```

On the Pi the drivers run in a container: `docker logs -f fire_resq_robot` to watch, `docker stop fire_resq_robot` to stop (do that before using the camera or the Arduino from anything else). It does not start by itself at boot.

**Sim and robot cannot mix.** `docker/station.sh` is ROS domain 42, where the robot lives. `FIRE_RESQ_SIM=1 docker/station.sh` is domain 0, this PC only: run the simulation and the test suite there.

**Checks (measured on this robot, in a dim room):** the camera arrives at 7–8 fps (15 in good light), about 50–60 ms old; `/odom` at 20 Hz.

---

## 3. Find out which motor is which wheel

Known: shield terminals **M1, M2 = front motors, M3, M4 = back motors**. Not known: which of each pair is left, and which way each one turns.

1. **Lift the wheels off the table.** Put the AA batteries in.
2. On the Pi:

```bash
docker stop fire_resq_robot
docker run --rm -it --privileged -v /dev:/dev fire_resq/robot ros2 run fire_resq_hardware motor_check
```

It spins one terminal at a time and asks which wheel moved and whether it rolled forward, then prints three lines.

3. Paste those lines into [`src/fire_resq_hardware/config/robot.yaml`](../src/fire_resq_hardware/config/robot.yaml) (`left_motors`, `right_motors`, `motor_signs`) and run `docker/deploy_robot.sh` again.

---

## 4. Drive it and measure it

Still on blocks for the first try:

```bash
docker/station.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`i` forward, `,` back, `j`/`l` turn, `k` stop. Let go and it stops within about 0.6 s; kill the driver or pull the USB cable and the firmware stops the motors within 0.4 s.

Then on the floor, measure the three numbers that stand in for encoders (all in `robot.yaml`, each marked `TODO(hardware)`):

| Number | How |
| --- | --- |
| `min_power` | Lower it until a forward command no longer moves the robot; use a bit more than that |
| `full_speed_mps` | Time a straight 1 m run at full speed |
| `track_slip_factor` | Command one full turn on the spot; scale by how far `/odom` says it turned vs. how far it really did |

Also measure the wheel separation and where the camera sits (forward of the wheel centre, height above the floor) for [`parameters.xacro`](../src/fire_resq_description/urdf/parameters.xacro). **Ask first:** that file is shared with the simulated robot ([Hardware.md](Hardware.md) §13).

If the robot cannot turn on the spot at all, that is the motors, not the software: four AAs through an L293D leave little torque for skid steering.

---

## 5. Make it see your victims and fire

1. Make three **blue** victims and one **orange** "fire". Matte, saturated colours; nothing else in the arena that colour.
2. Calibrate the camera once (print a checkerboard with 9×7 squares, measure one square):

```bash
docker/station.sh ros2 launch fire_resq_hardware station.launch.py perception:=false     # terminal 1
docker/station.sh ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
    --ros-args -r image:=/camera/image_raw -p camera:=/camera                              # terminal 2
```

Save the result over [`config/camera_c270.yaml`](../src/fire_resq_hardware/config/camera_c270.yaml) (same format) and redeploy. Until then the file holds datasheet values and every distance is approximate.

3. Set your objects' colours (`hsv_lo`/`hsv_hi`) and heights (`top_height_m`) in [`perception.yaml`](../src/fire_resq_perception/config/perception.yaml). How the estimate works: [CLAUDE.md](../CLAUDE.md) → "Perception".
4. `ros2 topic echo /fire_resq/detections` — `victim` / `fire` with positions in front of the robot (`x` forward, `y` left, metres). `position_valid: false` while the robot is turning is normal.

Fix the lighting before tuning colours: the camera's automatic exposure and white balance move the hues, and in dim light it halves its frame rate (`v4l2_controls` in `robot.yaml`).

---

## 6. Later: the full autonomous robot

In order; each one is checkable on its own ([Hardware.md](Hardware.md) §11).

| # | Step | You'll know it works when |
| --- | --- | --- |
| 1 | Steps 3–5 above | teleop drives straight; a victim 1 m ahead reads about 1 m |
| 2 | Fix the Pi's power (it reports under-voltage: [Hardware.md](Hardware.md) §7) | `vcgencmd get_throttled` stays `0x0` |
| 3 | **Decide and build the localisation source** ([Hardware.md](Hardware.md) §5) | something publishes `map → odom` and it stays put when the robot stands still |
| 4 | Fit the electromagnet + MOSFET module + flyback diode; write its driver node | `ros2 service call /fire_resq/set_magnet fire_resq_interfaces/srv/SetMagnet "{attach: true}"` lifts a victim |
| 5 | Run the mission: the same launch files as the simulation guide, without Gazebo | the robot rescues all victims by itself |

---

## Safety rules

- Wheels off the table for every first test.
- The firmware stops the motors 0.4 s after commands stop, and the driver refuses to talk to an Arduino that is not running this firmware. Do not remove either ([Hardware.md](Hardware.md) §8).
- Never run a simulation in the robot's shell (`docker/station.sh` without `FIRE_RESQ_SIM=1`): its `/cmd_vel` would drive the real motors.
- Flyback diode on the electromagnet, always. Never drive it from a bare pin.
