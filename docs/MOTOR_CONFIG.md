# Motor configuration that works

Confirmed working on the floor by the maintainer on 2026-10-13, after several wrong guesses. If the robot is rewired, or the motors start misbehaving, come back to this page before changing anything.

## Which terminal is which wheel

Arduino Uno + L293D motor shield (Adafruit Motor Shield v1), terminals M1–M4, four AA cells.

| Shield terminal | Wheel | Side |
| --- | --- | --- |
| M1 | front-right | right |
| M2 | front-left | left |
| M3 | back-left | left |
| M4 | back-right | right |

The front pair is wired the opposite way round from the back pair. That is the fact every earlier attempt got wrong.

All four motors have **negative** polarity for driving: a negative power drives the robot forward, a positive power drives it backward. (Changed 2026-10-17 at the maintainer's request: forward and backward were reversed, because what this page used to call "forward" took the robot away from the end the sensors face. Turns are unchanged.)

## What each movement sends

Power is −255 to 255 per terminal; **negative is forward**, positive is backward, 0 is released.

| Movement | M1 front-right | M2 front-left | M3 back-left | M4 back-right |
| --- | --- | --- | --- | --- |
| Forward | −168 | −168 | −168 | −168 |
| Backward | +168 | +168 | +168 | +168 |
| Turn left on the spot | +255 | −255 | −255 | +255 |
| Turn right on the spot | −255 | 0 | 0 | −255 |
| Curve left while driving (tightest) | −226 | 0 | 0 | −226 |
| Curve right while driving (tightest) | 0 | −226 | −226 | 0 |

- **Left turn** uses all four wheels: the left side (M2, M3) at −255, the right side (M1, M4) at +255. These are the turns as measured on 2026-10-13, when the sign meant the opposite direction of travel; the turn rows were not changed in the reversal. If "left" and "right" now look swapped on the floor, swap the two turn rows.
- **Right turn** uses only the two right wheels (M1 and M4), at −255. The left wheels are off.
- **Curving while driving** keeps both sides going forward at different speeds; the table shows the extreme, where the inner side stops. These two rows are the old ones with the sign flipped to match the reversed forward; they have not been tried on the floor.

Forward and backward are 168 in size at the current drive speed (0.12). Both turns are at full power.

## Why the right turn is different

A right turn that also drove the left wheels forward struggled or did not happen at all. M2 (front-left) was the motor spoiling it; with only M2 switched off it still struggled, so M3 is off as well. The cause has not been found: check M2's wiring and its shield channel when there is time.

## Where this lives

In [`src/fire_resq_hardware/config/robot.yaml`](../src/fire_resq_hardware/config/robot.yaml), under `base_node`:

> The values below are **not** updated for the reversal: `motor_signs` flips every movement including the turns, so it cannot express "forward and backward swapped, turns unchanged". The ROS driver (`base_node`) therefore still treats positive as forward. The stand-alone wander bot (`src/fire_resq_hardware/wander/wander_bot.py`) uses the table above.

```yaml
left_motors: [2, 3]
right_motors: [1, 4]
motor_signs: [1, 1, 1, 1]
front_motors: [1, 2]
turn_style: skid
right_turn_idle: [2, 3]     # terminals left unpowered in a right turn on the spot
full_speed_mps: 0.3         # placeholder, not measured
min_power: 110              # placeholder, not measured
```

After editing that file, send it to the robot with `./demo.sh deploy`.

The dashboard settings that go with it are saved in `demo_settings.json` in the `FireResQ` folder:

| Setting | Value |
| --- | --- |
| Drive speed | 0.12 |
| Turn power | 2.98 |
| Left turns: extra push | 3.0 |
| Right turns: extra push | 3.0 |

Turn power × push is far past what the motors can give, so both turns run at full power (255). Raising either number further changes nothing.

## What did not work

Kept so nobody tries them again.

| Tried | What happened |
| --- | --- |
| M1+M3 as the left side, M2+M4 as the right | The two front wheels fought the turn |
| M1+M2 as one side, M3+M4 as the other (the old `F/B/L/R` sketch) | They are the two axles, not the two sides: a "turn" reversed a whole axle and the robot backed away |
| Pivot turns: one side backward, the other stopped | Did not move the robot, or moved it backward |
| Back wheels only (M3, M4 in opposite directions) | Left turn was perfect; right turn did not happen |
| Front wheels only for the right turn | With M1 and M2 assumed the wrong way round, it turned left |
| Right turn with all four wheels | Struggled |
| Right turn with the left wheels forward and the right wheels stopped | Struggled |
| Right turn with only M2 off | Still struggled |

## Not done

- The wiring check (`./demo.sh motors`, wheels off the ground) has never been run. This page comes from watching the robot on the floor, one change at a time.
- `full_speed_mps` and `min_power` are placeholders, so the odometry and the map built from it are rough.
