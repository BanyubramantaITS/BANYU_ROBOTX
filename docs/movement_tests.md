# Movement tests on the real AUV (`tools/scripts/move/`)

One script per manoeuvre, for checking that the vehicle moves the way it is told before Task 2
runs. They talk MAVLink straight to the Pixhawk through MAVProxy (no ROS) and drive the sticks with
RC override on the `RCMAP_*` channels, as POR did.

| Script | What it does | Mode |
|---|---|---|
| `dive.sh --depth 1.0 --hold 10` | arm, dive, hold the depth, **disarm and float up** | ALT_HOLD |
| `forward.sh --time 5` | forward for N s | STABILIZE at the surface, ALT_HOLD with `--depth` |
| `backward.sh --time 5` | backward for N s | same |
| `sway.sh --dir left\|right --time 5` | sideways for N s | same |
| `yaw.sh --dir left\|right --angle 90` | turn by an angle on the compass (`--time N` = open loop) | same |
| `waypoint.sh --ne 10,0 ...` | drive to RTK GPS waypoints at the surface | STABILIZE |
| `where.sh` | print the GPS fix and heading, no arming (to note waypoints) | – |

Every run ends with neutral sticks and **disarm** (`--no-disarm` keeps it armed at the end of a
normal surface run; not allowed with a dive). It also stops and disarms on Ctrl+C, SSH drop (SIGHUP), SIGTERM, a mode change or disarm
from elsewhere, depth past `--max-depth`, a silent depth sensor, lost GPS or the run time limit.
Disarming makes the vehicle float up, so it must be positively buoyant.

## Before the first run

1. On the Pi: `run/run_mavproxy.sh` (and `rtk-rover` for waypoints; the Pixhawk reads the GPS itself).
2. **Stop `banyu_teleop` and the mission stack** (`sub_bringup.launch.py`, `launch_por.sh`): they
   drive the same sticks and arming. The scripts refuse to start while they run. MAVROS alone
   (`run/launch_mavros.sh`) is fine. With teleop stopped, **SB does not disarm**: the physical kill
   switch, Ctrl+C and QGC disarm are the ways to stop.
3. Run inside `tmux` on the Pi (`tmux new -s move`), so the laptop link can drop without ending the
   run halfway (an SSH drop outside tmux still disarms, via SIGHUP).
4. Vehicle at the surface: the depth datum (Bar30, `SCALED_PRESSURE2`) is taken just before arming.
5. Check first without arming: add `--dry-run` to any command.

Each run asks you to type `go`, counts down 3 s and arms.

## Examples

```bash
cd ~/ros_ws/src/BANYU_ROBOTX/tools/scripts/move

./dive.sh --depth 1.0 --hold 10            # dive to 1 m, hold 10 s, disarm, float up
./dive.sh --depth 2.0 --hold 20 --water salt

./forward.sh --time 5                      # surface, 30 % stick
./backward.sh --time 5 --power 25
./sway.sh --dir left --time 4
./forward.sh --depth 1.0 --time 5          # dive to 1 m first, settle 3 s, then forward

./yaw.sh --dir right --angle 90            # closed loop on the compass
./yaw.sh --dir left --angle 360 --depth 1.0
./yaw.sh --dir right --time 3              # open loop, 3 s of stick

./where.sh                                 # RTK fix and position; Ctrl+C prints a --wp line
./waypoint.sh --ne 10,0                    # 10 m north of the start fix
./waypoint.sh --ne 10,0 --ne 10,10 --return-home      # square-ish loop back to the start
./waypoint.sh --wp -7.2819000,112.7945000 --radius 2
./waypoint.sh --file my_points.txt         # lines 'LAT,LON' or 'ne N,E', # comments
```

## Useful options

| Option | Default | |
|---|---|---|
| `--power P` | 30 (yaw 25) | stick % of full; over 50 needs `--allow-high-power` |
| `--dive-power P` | 35 | max throttle stick % while diving and correcting depth |
| `--depth D` | 0 | do the move at this depth (dive first); over 3 m needs `--allow-deep` |
| `--max-depth D` | target + 1 m | abort and disarm deeper than this |
| `--settle S`, `--coast S`, `--ramp S` | 3, 3, 1 | hold before the move, neutral after it, stick ramp-up |
| `--radius R`, `--wp-hold S` | 1.5 m, 5 s | waypoint arrival radius, neutral hold at each point |
| `--allow-no-rtk` | off | waypoints with a plain 3D fix (default needs RTK float/fixed) |
| `--max-distance M` | 60 m | refuse farther waypoints; geofence abort at +20 m |
| `--mode` | STABILIZE / ALT_HOLD | force a flight mode (MANUAL for raw thrust) |
| `--connect URL` | `udpin:127.0.0.1:14552` | MAVProxy out (the same port as `check/test_hardware.sh`) |

All options: `./dive.sh --help`.

## What you get

The terminal prints a short result, for example:

```
Result forward:
  heading change: min 0.0 / mean 1.0 / max 1.5 deg
  GPS thrust only: 1.11 m (body forward +1.11 m, right +0.04 m), 0.22 m/s over the 5.0 s push
  GPS incl. coast: 1.55 m (body forward +1.55 m, right +0.06 m), ...
```

- **dive:** depth min/mean/max while holding, heading drift, time to reach the depth and to surface
  after disarm.
- **forward/backward/sway:** heading drift and depth during the push; with a GPS fix, the distance
  split into body forward/right (a "forward" that goes right means a thruster direction or mixing
  problem) and the speed, which is the number to put in the mission profiles.
- **yaw:** degrees turned and the error after settling.
- **waypoint:** time to each point, the distance after the hold, and the median difference between
  GPS course over ground and compass heading (a large value means a compass offset or current).

Each run also writes `~/banyu_bags/move_tests/<YYYYmmdd_HHMMSS>_<test>.csv` (10 Hz: phase, mode,
armed, filtered and raw depth, attitude, GPS, battery, the four sticks) and a `.txt` with the
summary. MAVProxy's `.tlog` in `~/banyu_bags/mavproxy/` has the full MAVLink traffic.

## Notes

- Stick directions come from the Pixhawk: `RCMAP_*` and `RCn_REVERSED` are read at start. If a test
  goes the wrong way, fix the Pixhawk parameters or motor directions, not the script.
- The closed-loop yaw releases the stick early by `--yaw-lead` (0.5 s of turn rate); raise it if the
  real vehicle overshoots, lower it if it stops short.
- Tested against ArduSub SITL (vectored frame, `banyu_defaults.parm`): dive 1.0/1.5 m, all four
  directions with the right sign, yaw 45/90/180 deg within 2 deg, three surface waypoints, and the
  depth-limit, SIGTERM and SIGHUP stops. Not yet run on the vehicle: start with `--dry-run`, then a
  short surface test at low power.
