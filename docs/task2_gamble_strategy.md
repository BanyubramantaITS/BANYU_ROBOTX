# Task 2: gamble strategy (`--sensors gamble`)

A fallback for when the camera cannot be used: the UUV goes midway between the buoys (or to the start buoy) by GPS, dives to a set
depth on the Bar30, flies timed forward/backward, sway and yaw legs at that depth, surfaces, and reports the start buoy with a **guessed**
light order. It does not look for the pipe, does not read the lights and does not repair. Needs the Bar30 (since 2026-11-13; before
that it made a timed dive and bobbed in STABILIZE with no depth sensor).

## Flow

BT: `banyu_mission/bt/task2_gamble_mission.xml`; parameters: `banyu_bringup/config/task2_sensors_gamble.yaml`.

1. Wait on the surface for the buoy positions (USV `PipelineBuoys`; no fallback to the YAML coordinates), navigate midway between the start and end buoys by GPS (GUIDED, `dive_point: midpoint` as in the test mission, keeping `buoy_keep_out_m` clear of both), then align the heading. With only the start buoy it goes `buoy_standoff_m` inside it. The `dive_point` launch argument (`--dive-point`) defaults to `auto`, which is midpoint for `sensors:=gamble`; `--dive-point buoy` goes `buoy_standoff_m` inside the start buoy instead.
2. Switch to ALT_HOLD and dive to `mission.gamble.depth_m` on the Bar30 (ends within `depth_tolerance_m` for 0.5 s, or after
   `dive_timeout_s`), settle, then fly forward and back, sway left and right, yaw left and right, each leg timed and holding the
   depth, with `settle_s` between legs. The surge and sway legs hold the heading the leg started with (`timed_move.heading_hold`).
3. Rise to the surface by the Bar30 depth (`Surface`). After a Bar30 fault the depth is invalid and `Surface` rises blind
   (`control.surface_blind_rise_rate_mps`) until its time limit.
4. Send the best-effort Core survey: the active buoy position and one RED at the chosen position, GREEN for the rest.

A failed navigation or dive does not stop the report: the tree still surfaces and reports.

## Run

```bash
tools/scripts/run/launch_task2.sh --real --tier core --sensors gamble                 # RED at position 2 or 3 at random (default)
tools/scripts/run/launch_task2.sh --real --tier core --sensors gamble --gamble-red 3  # always position 3
tools/scripts/run/launch_task2.sh --real --tier core --sensors gamble --gamble-red 1,2,3,4
```

`--sim` instead of `--real` runs it in Gazebo. Position 1 is the node closest to the active buoy, position 4 the farthest (the handbook's
report order), so the physical node depends on which buoy is active. Values other than positions 1-4 are refused. The launch argument is
`gamble_red` (`survey.gamble_red_positions`); do not set it in a params file, which would win over the launch argument.

## Sensors

| Sensor | Used for |
|---|---|
| RTK GPS | Navigation to the buoy midpoint |
| Bar30 | Depth for surfacing |
| Pixhawk compass | Heading |
| Pixhawk IMU | Attitude (ALT_HOLD) |
| Bar30 | Dive depth and depth hold, surfacing |
| Bottom camera, hydrophone | Not used for any decision |

It also needs the communications link: the active buoy from the USV or UAV, and the report to RoboCommand.

## Points (handbook, Task 2 Core)

| Item | Points | With this strategy |
|---|---|---|
| Heartbeat | 25 | Yes |
| Correct start point | 25 | Yes, if the USV or UAV reports which buoy is active |
| Light sequence in the correct order | 150 | By chance: 1 in 4 if the RED node is uniformly random |
| Repair | 250 | No |
| **Total** | **200 of 450** | **About 87.5 on average** (50 or 200) |

The average assumes one RED node and all-or-nothing order points; the handbook states neither a RED distribution nor partial credit.
Positions 2 and 3 are no better than 1 in 4 unless the organisers favour the middle nodes, which is a guess with no evidence. If practice
shows a pattern, set a fixed position. Advanced and Disruptive cannot be done: the flash code only appears after a repair, and the UUV points
of those tiers require the repair.

## Sim results

The runs below are from the earlier timed-dive-and-bobbing version (no Bar30).

Gazebo, Core tier, random spawn, judged from Gazebo ground truth; 5 runs, mission about 60-70 s from the navigation start. The first 5 runs went to the start buoy (before the 2026-11-07 change to the midpoint). Midpoint (default `dive_point:=auto`), Core, random spawn, seed 1: target midway between the buoys, `NavigateSurfaceGps` and `AlignHeading` SUCCESS, report GREEN, RED, GREEN, GREEN with node 2 damaged: 200 points.

| Run | Report | RED node | Points |
|---|---|---|---|
| Random 1-4, seed 2 | GREEN, RED, GREEN, GREEN | 1 | 50 |
| Random 1-4, seed 1 | GREEN, GREEN, RED, GREEN | 2 | 50 |
| Random 1-4, seed 5 | RED, GREEN, GREEN, GREEN | 3 | 50 |
| Default 2,3, seed 1 | GREEN, GREEN, RED, GREEN | 2 | 50 |
| `--gamble-red 4`, seed 2 | GREEN, GREEN, GREEN, RED | 1 | 50 |

Maximum depth about 2 m. The default-position run was before the fix that made `--gamble-red` take effect and was not repeated after it;
the default now comes from the same value through the launch argument. Unit tests: RED only at the configured positions, always one.

## Check before using it on the robot

1. **Active buoy:** the robot must be told which buoy is active. Without it, it stands by on the surface after the run start (no fallback to the YAML coordinates) until the operator aborts.
2. **Depth and legs:** all in `banyu_bringup/config/task2_sensors_gamble.yaml` (`mission.gamble.*`; the overlay wins over
   `task2_params.yaml`). Distance per leg is about duration x speed and every pair goes out and back. Keep the legs shorter than the
   distance to the buoys' mooring lines and inside the course fence: nothing is enforced under water (no position).
3. **Bar30:** with no fresh calibrated depth the dive commands no vertical motion and times out at `dive_timeout_s`; check the Bar30
   on the surface before the run.
4. **POR:** `TimedMove.action` gained `run_id` and `epoch` (Task 2 refuses goals with no run identity). Unit tests pass; POR was not re-run in the sim.
5. **Rules:** the report says the lights were surveyed when they were not, and the handbook's 150 points read "UUV must survey the pipeline and report". Ask the organisers before relying on it.
