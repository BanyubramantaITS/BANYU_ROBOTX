# Task 2 surface planning: what the trials showed (2026-11-10 and 2026-11-12)

`NavigateSurfaceGps` steers the surfaced UUV to the dive point around the USV's pipeline buoys, Task 4 zones and the
fixed surface obstacles (today the UAV's Task 1 buoy field). This note records the data used to tune the obstacle
handling, the changes it led to and how to re-check them.

## Data

| Source | What it holds |
|---|---|
| `~/banyu_bags/20261110_*` (13 runs, real water) | GPS fix, GPS velocity, compass heading, commanded and measured velocity, `NavigateSurfaceGps` feedback and status. No autonomous transit finished on that day (every `NavigateSurfaceGps` goal was rejected by the arbiter or ended on a safety fault), so these runs give the GPS and hull numbers, not tracking error. |
| `docs/task2_results/20261112_uav_safe_passage.log` (trial, 17.6 min) | Every change of the UAV's `SafePassage` and of the `/banyu/surface/obstacles` built from it on the Pi. The vehicle bags of that day stayed on the Pi. |

## Findings

GPS on the water (Nov 10, 35 000 fixes):

- Fix status 0 (plain GPS) in every message: no RTK. The UAV's buoy positions come from another plain receiver,
  so the two can disagree by about a metre.
- Stationary 20 s windows: spread from the mean median 0.15 m, p90 0.33 m, max 1.04 m.
- Fix interval median 0.10 s; 436 gaps over 1 s, some 4-5 s. The node already drops the fix after 2 s (`kGpsTimeoutS`).
- Speed in manual drives median 0.45 m/s, up to 1.66 m/s (`nav_max_speed` is 0.4).

UAV buoy field (Nov 12):

- The field has 9 physical buoys. The best message is the first one (09:31:49): 8 of them (the ninth was never
  reported). Every later message holds a subset: 5 buoys at 09:39 (B2, B4, B7 missing), then 6 from 09:48 (B2, B5
  missing). B2 (1.280771, 103.855216, the south end, also that message's exit) is in the first message only.
  Each reported position jitters 0.1 m rms (0.26 m max). The entry/exit fields swap or are (0, 0) for most messages.
- The UAV publishes only when its map changes (nothing between 09:31:49 and 09:39:17) with volatile QoS: a UUV stack
  started after the first message never hears B2, and lacks B4 and B7 until 09:48.
- The obstacle list was replaced by each message, so a dropped buoy stopped being an obstacle.
- Neighbouring buoys are 3.0-3.8 m apart: with 1.2 m discs the gaps between them are 0.65-1.3 m, and the planner
  (vehicle as a point) would thread them.

## Replay

`tools/scripts/sim/surface_field_replay.py` replays the recorded stream through a port of the planner (`detourAim`,
the exit from a disc, the speed law), with a 1 s velocity lag and GPS error matched to the Nov 10 numbers plus a
UAV/UUV receiver bias. 56 crossings of the field in four directions, half of them during the minutes when two
buoys were missing. `min clear` is the distance from the hull centre to the nearest true buoy centre. This first table
took the six clusters seen in most messages as the true buoys (now `--truth clusters`; the script has since added a
third start time, so it runs 84 crossings).

| Variant | 0 m bias: reached / min clear | 1.0 m bias | 1.5 m bias |
|---|---|---|---|
| Before (no memory, no margin, no hysteresis) | 56/56, 0.73 m | 56/56, 0.10 m | |
| Memory | 56/56, 0.97 m | 56/56, 0.30 m | |
| Memory + margin 0.6 | 56/56, 1.50 m | 55/56, 0.66 m | |
| Memory + margin 0.75 | 56/56, 1.53 m | 56/56, 0.85 m | 56/56, 0.43 m |
| **Memory + margin 0.75 + hysteresis** | **56/56, 1.70 m** | **56/56, 0.88 m** | **56/56, 0.64 m** |
| Memory + margin 0.9 | 53/56, 1.67 m | 53/56, 1.06 m | 53/56, 0.71 m |
| Memory + margin 0.9 + hysteresis | 54/56, 1.90 m | 56/56, 1.15 m | 56/56, 0.77 m |

With margin 0.9 and no hysteresis the planner dithered between the two sides of nearly touching discs (about 450
side changes per sweep instead of 30-70) and three crossings never arrived.

### With the 8-buoy field (2026-11-13)

The table above took the true buoys as the six clusters seen in most messages. Re-run with the first message as the
truth (`--truth largest`, the default now): 84 crossings (B2 and B5 widen the field), starting during the 8-, 5- and
6-buoy messages.

| Variant | 0 m bias: reached / min clear | 1.0 m bias | 1.5 m bias |
|---|---|---|---|
| Before (no memory, no margin, no hysteresis) | 84/84, 0.14 m | 84/84, 0.03 m | 84/84, 0.01 m |
| Memory + margin 0.6 + hysteresis | 84/84, 1.22 m | 84/84, 0.86 m | 84/84, 0.48 m |
| Memory + margin 0.75 | 84/84, 1.51 m | 84/84, 0.75 m | 84/84, 0.24 m |
| **Memory + margin 0.75 + hysteresis** | **84/84, 1.67 m** | **84/84, 0.92 m** | **84/84, 0.57 m** |
| Memory + margin 0.9 + hysteresis | 83/84, 1.79 m | 82/84, 1.13 m | 82/84, 0.58 m |
| Memory + margin 1.0 + hysteresis | 80/84, 1.79 m | 80/84, 1.14 m | 80/84, 0.78 m |

The tuning holds: 0.75 m is still the largest margin that reaches every crossing (from 0.9 m the planner boxes
itself in against the denser field). Missing the first message is what breaks it: `--skip 1` (the UUV never hears
B2, and B4/B7 only from 09:48) gives 0.02-0.04 m minimum clearance with every variant. **Start the UUV stack (its
`pipeline_buoys_node`) before the UAV maps the field**, or ask the UAV team to republish the full map periodically or
with transient_local durability (a transient_local subscriber would not match their volatile publisher, so this end
cannot fix it alone). The ninth buoy is never reported, so nothing here avoids it.

## Changes

- `pipeline_buoys_node` remembers the field (`UavBuoyField`): a buoy stays an obstacle after the UAV drops it from
  its map, a report within `uav_buoys.merge_m` (1.0 m) moves it, `uav_buoys.forget_s` (0 = never) drops a buoy not
  reported that long, and a new `course_id` clears the field.
- `vehicle_execution_node` adds `surface_obstacles.margin_m` (0.75 m) to every surface obstacle radius, like
  `zones.margin_m` for Task 4 zones.
- `NavigateSurfaceGps` keeps the detour point it chose while the point is still ahead, outside every obstacle and in
  clear view, unless a fresh plan is shorter by more than 2 m (`keepDetourPoint`).
- Unit tests: `UavBuoyField` (the trial's dropout and jitter), `keepDetourPoint`, and crossings of the trial field
  (`TrialBuoyFieldCrossingsStayClearWithDetourHysteresis`).

## Gazebo check (2026-11-12)

`task2_sim_e2e.sh --tier core --no-gate --launch-arg --spawn 0 -16 0` with
`tools/sos_integration/mock_uav_safe_passage.py --domain 40 --north -8`: the trial field 8 m south of the dive point
(the midpoint, at the GPS home), its top buoy right on the straight line, and that buoy plus one more dropped from the
messages once `NavigateSurfaceGps` started. `pipeline_buoys_node` kept all six; the transit went round the west end
of the field in one sweep (21.4 m for 16 m straight), passed 2.25-6.6 m from the buoy centres (keep-out 1.95 m) and
3.35 m from the dropped one on the line, ended 0.43 m from the dive point (`NavigateSurfaceGps` and `AlignHeading`
SUCCESS).

The sim uses the UUV domain (40): `launch_task2.sh --sim`, `launch_mavros.sh --sim` and `task2_sim_e2e.sh` now set
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, so a sim on a laptop on the robot's network cannot reach the real MAVROS.

## Gazebo check with the 8-buoy field (2026-11-13)

`mock_uav_safe_passage.py --domain 41 --north -8` (the trial's first message, then its 5- and 6-buoy subsets once
`NavigateSurfaceGps` runs) and `task2_sim_e2e.sh --tier core --no-gate --launch-arg --spawn --launch-arg 0
--launch-arg -22 --launch-arg 1.5708`: B2 lies on the straight line 5 m ahead of the spawn, and B6-B1-B7 (with B4 and
B5) form a wall of overlapping discs between it and the dive point.

- The transit drove through B2 (0.27 m from its centre). `pipeline_buoys_node` had all 8 buoys, then cleared them
  when the first course arrived (a new `course_id` cleared the field, also the first one after startup); the later
  messages never hold B2 again. On the course the UAV maps the field before the GCS sends the Task 2 course, so this
  would have dropped the first message there too. Fixed: only a change from one known course to another clears the
  field (the boundary already filters it at publish). Test: `FirstCourseKeepsTheUavFieldHeardBeforeIt`.
- It then stopped at y = -6 in front of the wall and swung between x = -1 and +0.9 for the rest of the run (68.7 m of
  track, 6.1 m short of the dive point when the run was stopped) instead of going round one end. Not yet rerun with
  the fix: the run started knowing only 5 buoys, so whether the dithering remains with all 8 known is still open.

## Still open

- Rerun the 8-buoy Gazebo check with the course fix; if the transit still dithers in front of the B6-B1-B7 wall,
  look at `keepDetourPoint` (the detour point leaves clear view behind the wall's corner).

- No real-water autonomous transit is in the data yet: the next trial bag with a `NavigateSurfaceGps` goal running
  gives the tracking error (and the real receiver bias, by driving past a buoy the UAV reports). Re-run the replay with
  `--bias` set to that number and adjust `surface_obstacles.margin_m`.
- The UAV team: the exit buoy is (0, 0) in most messages and entry/exit were swapped once (09:48:59).
