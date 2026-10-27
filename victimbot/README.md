# Victimbot: a fire-rescue robot on the real car

A separate project from the ROS 2 FireResQ stack. It is one Python program on the Raspberry Pi, plus the Uno firmware.
It reuses the earlier rescue bot's ideas (`~/rescue` on the Pi): its dashboard page, still-picture homing, camera-measured turning and the "closest to the fire first" rule.
It uses the robot's current motor configuration.

## The robot

| Part | What |
| --- | --- |
| Computer | Raspberry Pi 3B, `ssh pi@pi.local` |
| Motors | Arduino Uno + L293D shield, 4 DC motors, USB serial `/dev/ttyUSB0`, 115200 baud |
| Camera | OV5647 ribbon camera, mounted turned 90° clockwise (every frame is turned back) |
| Sonar | HC-SR04, **TRIG = A3, ECHO = A4** (found with `firmware/pinfinder`) |
| Servo | MG90S on **SERVO 1** (D10). It pans the sonar; straight ahead is **128°** |
| Magnet | At the front, in front of the camera (the dark arc at the bottom of the picture). It picks a victim up by touching it |

**Motors.** The table comes from `~/fireresq/motor_config.md`, with front/back and left/right swapped, because the camera end is the front.

| Move | M1 rear-left | M2 rear-right | M3 front-right | M4 front-left |
| --- | --- | --- | --- | --- |
| Forward | − | − | − | − |
| Backward | + | + | + | + |
| Turn left | 0 | 0 | −255 | +255 |
| Turn right | −255 | +255 | 0 | 0 |

A negative power drives a wheel forward.
The left turn is the controller's "turn right" row, and the right turn is its "turn left" row.

**The sonar pins.** A4 is the only pin the sensor holds low, so it is ECHO.
A pulse on A3 gets an echo 8 times out of 8 even with A2 held low.
A pulse on A2 gets nothing once A3 is held low, so A2 only appeared to work through crosstalk onto the neighbouring wire.
A0 and A1 are shorted together on the shield; neither is used.

## What it does

It runs until Ctrl+C:

1. **Wander, mostly turning.** It turns 40° on the spot and looks, then does it again. After every 2–4 turns it makes one short 25 cm hop. The hop goes towards whichever way the sonar finds most room. When the robot is more than 2 m from the start, hops lean back towards it.
2. **Choose the most dangerous victim.** Every look updates a map of victims and fires. The victim closest to a fire's base edge is rescued first. With no fire known, the nearest victim goes first. The Thinking panel explains each choice.
3. **Pick it up.** The robot drives a route that keeps clear of fires and other victims. It then homes in on still pictures: centre the victim, hop, look again. When the victim reaches the magnet's edge of the picture, the robot creeps so the magnet touches it.
4. **Check the pick-up.** The robot backs off 10 cm and looks. A victim still standing in front of it was missed; it gets one more try, then that spot is left alone.
5. **Carry it home.** The robot drives back to the start (the safe area) and stops for 5 seconds. Take the victim off the magnet then.
6. **Repeat.** Victims seen inside the safe area count as already saved.

## How it sees

| Object | Rule |
| --- | --- |
| Victim | A toy DC motor carrying a **V-shaped yellow-green sticky note** (hue 37–39, saturation ~177; nothing else in the room has that colour). The robot aims at the point of the V, which is right over the motor. The motor's floor contact is 0.45 × the note's height below the note |
| Fire | Every saturated red blob (the red box and the red bag) |
| Distance | From how far below the horizon the object meets the floor |

The horizon is re-measured in every look from the victims themselves.
They are all the same motor, so (floor contact row − horizon) = (camera height ÷ can height) × (can height in pixels).
Tilting the camera therefore does not break the distances.
The sonar is only used for obstacles.

**Moving.** There are no encoders.
Turns are pulses, and the camera measures how far each one really turned.
Straight legs hold their heading with the camera.
Distance is time × speed, and the speed is learned from how fast a victim's range shrinks while homing on it.

## The simple mission (`rescue3.py`): use this one

From the start square, all three victims and the fire are in view; the robot never wanders.

1. **Look.** Six photos. The sonar is pointed at the red box, and its range scales every camera distance.
2. **Rank.** The victims are ranked closest to the fire first.
3. **For each victim:**
   - line up with small turns until its note's point is within 4° of the centre of the picture;
   - hop forward, re-centring between hops;
   - push the last few cm onto the magnet;
   - back off 10 cm and check that it is gone from the floor (one more try if not);
   - replay its moves backwards to the start square;
   - stop 5 s: take the victim off the magnet.

```bash
python3 -u rescue3.py --look     # look and rank only: nothing moves
python3 -u rescue3.py            # the mission
```

## Teleop demo (`teleop_demo.py`): driven by hand, operator narration

A standalone script that imports nothing from the bot. Nothing in it decides anything: the operator drives and picks the captions.

- **`http://pi.local:8000` (control):**
  - hold W/A/S/D or the arrow keys to drive; Space or STOP stops it;
  - small camera preview;
  - narration buttons: scanning, fire detected, 3 victims found, then going / aligning / picked up / saved for each of victims A, B and C, mission complete, and a free-text caption.
- **`http://pi.local:8090` (viewer, for a second laptop):** the FireResQ dashboard. The camera shows the fire box, the victims (labelled A, B and C from left to right) and the chosen caption. The Actions and Thinking panels show the longer texts.

The robot stops within 0.5 s if the control page stops sending.

```bash
python3 -u teleop_demo.py
```

## Run it

```bash
./deploy.sh                                   # on the PC: copy to the Pi and run the self-test there
ssh pi@pi.local
cd ~/victimbot
python3 -u victimbot.py --plan                # look, decide and explain; never moves
python3 -u victimbot.py --moves               # WHEELS OFF THE GROUND: forward, back, left, right, 1 s each
python3 -u victimbot.py --dry                 # the whole mission, but no motor command is ever sent
python3 -u victimbot.py                       # THE MISSION (5 s countdown)
python3 -u victimbot.py --view                # camera + detection only
python3 -u victimbot.py --sonar               # sonar sweep, prints distances
```

**Dashboard:** http://pi.local:8090. It is the same page as the earlier rescue bot, with four panels: live camera, map, actions and thinking.

## To measure (constants at the top of `victimbot.py`)

| Constant | Status |
| --- | --- |
| `VICTIM_HEIGHT_M`, `CAM_HEIGHT_M` | 0.025 m (a toy motor's can) and 0.10 m are guesses. Their ratio sets every distance |
| Lighting | The motors' colour was measured in dim room light. Keep the same light for the demo; a much brighter light may make the cans look grey rather than blue-grey |
| `MAGNET_REACH_M` | 0.08 m (how far ahead of the lens the magnet's face is): a guess. Raise it if the robot stops short; lower it if it pushes the victim away |
| `CRUISE`, `CRUISE_MPS` | 200 power and 0.20 m/s at first. The speed is re-learned while homing |
| `SONAR_LEFT_SIGN` | +1: a higher servo angle points left. The sonar readings matched the camera |
