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

- Six physical buoys; each reported position jitters 0.1 m rms (0.26 m max).
- The field in the messages changes size (5, 6, 8 buoys): two real buoys were missing for about 9 min, and two
  others appeared once and never again. The entry/exit fields swap or are (0, 0) for most messages.
- The obstacle list was replaced by each message, so a dropped buoy stopped being an obstacle.
- Neighbouring buoys are 3.0-3.7 m apart: with 1.2 m discs the gaps between them are 0.65-1.3 m, and the planner
  (vehicle as a point) would thread them.

## Replay

`tools/scripts/sim/surface_field_replay.py` replays the recorded stream through a port of the planner (`detourAim`,
the exit from a disc, the speed law), with a 1 s velocity lag and GPS error matched to the Nov 10 numbers plus a
UAV/UUV receiver bias. 56 crossings of the field in four directions, half of them during the minutes when two
buoys were missing. `min clear` is the distance from the hull centre to the nearest true buoy centre.

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

## Still open

- No real-water autonomous transit is in the data yet: the next trial bag with a `NavigateSurfaceGps` goal running
  gives the tracking error (and the real receiver bias, by driving past a buoy the UAV reports). Re-run the replay with
  `--bias` set to that number and adjust `surface_obstacles.margin_m`.
- The UAV team: the exit buoy is (0, 0) in most messages and entry/exit were swapped once (09:48:59).
