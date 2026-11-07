# Task 2: gamble strategy (`--sensors gamble`)

A fallback for when the camera or the Bar30 cannot be used: the UUV goes midway between the buoys (or to the start buoy) by GPS, makes a short timed dive and bobs up and
down, surfaces, and reports the start buoy with a **guessed** light order. It does not look for the pipe, does not read the lights and does
not repair. Sim tested only.

## Flow

BT: `banyu_mission/bt/task2_gamble_mission.xml`; parameters: `banyu_bringup/config/task2_sensors_gamble.yaml`.

1. Wait on the surface for the buoy positions (USV `PipelineBuoys`; no fallback to the YAML coordinates), navigate midway between the start and end buoys by GPS (GUIDED, `dive_point: midpoint` as in the test mission, keeping `buoy_keep_out_m` clear of both), then align the heading. With only the start buoy it goes `buoy_standoff_m` inside it. The `dive_point` launch argument (`--dive-point`) defaults to `auto`, which is midpoint for `sensors:=gamble`; `--dive-point buoy` goes `buoy_standoff_m` inside the start buoy instead.
2. Switch to STABILIZE; dive 4 s, then bob three times (1.5 s no thrust, 1 s down). Time and thrust only, no depth sensor.
3. Rise until a fresh GPS fix proves the surface (`calibrated_depth_node depth_source: gps_surface`, `Surface` blind rise).
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
| RTK GPS | Navigation to the buoy midpoint, proof of the surface |
| Pixhawk compass | Heading |
| Pixhawk IMU | Attitude (STABILIZE) |
| Bottom camera, Bar30, hydrophone, DVL | Not used for any decision |

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
2. **Depth:** time and thrust only; the real response differs from the sim (the sim sinks at about 0.75 m/s on 0.05 commanded). Test in a pool first, for hitting the bottom or not sinking at all.
3. **Surface proof:** it trusts a fresh fix on `/mavros/global_position/global`. The sim cuts the GPS under water; on a real Pixhawk this topic may keep publishing with no GPS, and the report could be sent under water. Check in the pool.
4. **POR:** `TimedMove.action` gained `run_id` and `epoch` (Task 2 refuses goals with no run identity). Unit tests pass; POR was not re-run in the sim.
5. **Rules:** the report says the lights were surveyed when they were not, and the handbook's 150 points read "UUV must survey the pipeline and report". Ask the organisers before relying on it.
