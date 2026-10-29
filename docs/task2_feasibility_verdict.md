# Task 2 Core, Advanced, Disruptive: feasibility verdict (simulation, minimal sensors)

Branch `feat/task2-core-disruptive-sim`. Evidence: `task2_results/`, `task2_run_log.md` (every hypothesis H1-H23 and its result),
`task2_core_repair_by_camera.md`, `task2_mode_experiments.md`. Simulation only; nothing here is a hardware result.
The sim is Gazebo Harmonic + ArduSub SITL with the GPS cut under water (lost at 0.1 m), no DVL, Bar30 depth, IMU, a bottom
camera, a mock GCS and a mock UAV. Every result below is judged from Gazebo ground truth (`/robotx/light_node_N/status`
and the vehicle pose), not from the mission log. Stopped by the operator on 2026-10-29; the planned 5 of 5 repeats, the
fisheye and turbid-water detector tests and the other plan items were not reached (list at the end).

## Verdict per tier

| Tier | Verdict | Points in the sim (UUV side) | What was shown |
|---|---|---|---|
| Core | **YES in the sim** | 450 per passing run (25 + 25 + 150 + 250) | Repair RED to GREEN confirmed by Gazebo and the survey order correct on all four damaged nodes, with the UUV at the middle of the course (5 of 5 runs) and with a random spawn 12-20 m away (4 of 4 runs, after four fixes). |
| Advanced | **YES in the sim, narrower evidence** | 450 + 25 for the delivery circle (and it enables the 225 UAV points; the UAV is a mock) | One passing run on each of the four nodes at the middle spawn; with a random spawn node 1 passed and node 3 had a wrong survey order (fixed afterwards by H23, **not re-run**). Circle colours RED and BLUE shown; GREEN not run. |
| Disruptive | **YES in the sim, narrower evidence** | 450 + 25 + 25 for the tin and the circle (enables the 625 UAV points, mock) | One passing run on each of the four nodes at the middle spawn. 5 of the 9 tin/circle pairs run in the sim; the 4 others only in the decoder unit tests. **Not run with a random spawn.** |

"YES" means the tier works end to end in the sim under the stated conditions; it is not a claim about the real robot.

## Success rates (Gazebo ground truth)

| Case | Result |
|---|---|
| Core, spawn at the middle, damaged nodes 1, 2, 3, 4 (light seeds 2, 1, 5, 9) plus node 2 default | 5 of 5 (repair GREEN, survey order correct) |
| Core, random spawn (seeds 2, 5, 9, 1 = nodes 1, 3, 4, 2) | 4 of 4 in the last batch. Earlier batches on older code: 2 of 4 and 3 of 4, the failures were the fixes below |
| Advanced, spawn at the middle, nodes 2, 1, 3, 4 | 1 run each, all pass (survey order, repair, circle) |
| Advanced, random spawn | seed 2 (node 1) pass; seed 5 (node 3) repair and circle right, **order wrong** (red in slot 4): fixed by H23, not re-run |
| Disruptive, spawn at the middle | node 2 (tin RED, circle BLUE), node 1 (RED, GREEN), node 2 (RED, GREEN), node 3 (GREEN, BLUE), node 4 (GREEN, GREEN): all pass |
| Disruptive, colour pairs not run in the sim | BLUE/BLUE, BLUE/GREEN, BLUE/RED, GREEN/RED, RED/RED (seeds 12 18 35 3 43 6 24 34 were queued, not run). The decoder unit tests (14) cover all 9 pairs. |
| Hydrophone, search prior, surface-to-fix, other-end-active cases, S2, S3, S11 | not done |

Each node has only one or two passing runs on the final code, so none of this is a statistical rate. The target of 5 of 5 per node was not reached.

## What the approach is now (what changed from the first plan)

1. **Dive:** a fixed `inspection_depth_m` 12.5 m (Bar30), then the pipe is found and followed by camera at that depth. There is no vertical tracking, so a pipe that rises or falls beyond the camera range cannot be followed (the sim pipe is at about 13.4 m). The handbook asks for variable depth: **open**.
2. **Repair by camera, no fixed depth** (`HoldProbeOnRing` approach mode): centre the RED ring, sink at 0.12 m/s with the ring centred, stop on the lid's apparent size (`observation.size` >= 0.14), creep a little further, hold until the ring stays GREEN for 1.5 s, then climb back (Core) or rise 0.6 m (Advanced and Disruptive).
3. **Flash code:** read from 0.6 m above the node, no centring while reading (centring on the lit lid walked the vehicle off the node; the lid is dark for most of the code), decoder only counts lit rings (confidence >= 0.85) and only flashes after the repair was seen (RED, then steady GREEN).
4. **Reporting:** the tracker keeps ring order whichever end of the pipe is met first, and the ring position includes the sideways offset.
5. **Run robustness:** the navigation to the start buoy is retried for 3 minutes (the start point can arrive after the run start), the depth datum follows the barometer jump after arming while a GPS fix proves the surface (never during the run), the `Surface` deadlock and the report rejection were not seen again in the runs after these fixes (a handful of runs, not proof).

## Failures found and fixed on the way (details in `task2_run_log.md`)

| What | Cause | Fix |
|---|---|---|
| Repair never confirmed | The tip sat at the 0.15 m limit; one green frame counted as a repair | creep to a larger lid size; need 1.5 s of steady GREEN |
| Repair loop (Advanced) | Fixed camera height 1.7 m in the tracker put the ring 1 m off once the vehicle was 0.4 m above it | height from the lid size when it is large (>= 0.08) |
| Survey order all GREEN | The first far sighting of the red node was GREEN and was kept as the original | before the repair, GREEN then RED = originally RED |
| False flash code | The dark half of a flash reads as a weak GREEN (the water); red flicker counted as flashes | weak colour-threshold rings capped at 0.8 and ignored by the decoder; no code before RED then steady GREEN |
| Vehicle left the node while reading | Centring on a neighbouring lit node, then on the lid itself; drift of 0.06-0.15 m/s | rise 0.6 m and hold without centring |
| Tie GREEN/BLUE in Advanced | dark half reads GREEN | a colour that agrees on its own beats GREEN |
| Follow/Search livelock | Both goals ticked every 50 ms displaced each other at the goal arbiter | wait 2 s after a failed reacquire |
| Report rejected (random spawn) | Pipe met from the red end, `ordering_verified` only allowed rising progress | direction fixed by the second ring |
| UUV could not surface (random spawn) | Sim barometer jumps about -0.5 m at arming, datum dropped at the run start | GPS fix as surface proof before the run starts (a stricter safety rule was kept for the run itself; a looser re-calibration was tried, reverted) |
| Mission failed at once (random spawn) | Start-point message arrived 0.8 s after the run start | retry the surface navigation |
| Seed 5 order wrong (Advanced, random spawn) | Ring position without the sideways offset gave one node two records | sideways offset in the position (H23, **untested in the sim**) |

## Sensor limits that decided the result (plan section 13)

- **No position under water.** GUIDED is unsafe once the GPS is gone; ALT_HOLD with RC override held depth and heading. The vehicle drifts 0.06-0.15 m/s whenever it commands nothing (cause not found), which is why the code is read from 0.6 m up.
- **Bottom camera only** for the pipe, the rings and the flash code. A ring is seen from about 1.3 m ahead down to touch distance, but the lid fills the image at contact and the probe rod hides part of it, so offsets there are unreliable; the lid size gives range without a depth sensor, but only when close.
- **Depth sensor** reads about 5% high in the sim and its datum jumps at arming; the pipe depth is not known to the robot (no altimeter).
- **Hydrophone and front camera** were not used (no data on the real ones).
- The real HORUS T10 is fisheye (1920x1080, 25 FPS); the sim camera is a pinhole at 320x240, so the ring thresholds, the lid-size model (`lid_diameter_m` 0.074, `focal_ratio` 0.866) and the contact size 0.14 are **sim values that must be re-tuned after the camera calibration**.

## Not done (open items)

1. Re-run Advanced and Disruptive with a random spawn on the final code (H23 untested); repeat every node to reach 5 of 5.
2. The 4 remaining Disruptive colour pairs and Advanced circle GREEN in the sim.
3. Fisheye distortion, the turbidity plugin and the IR LED in the detector tests; Marina Bay home coordinates (the sim is still at the Long Beach datum, 33.81 N, -118.39 E).
4. Vertical tracking of a pipe at a variable depth; a node on a vertical support (needs a side camera); the drift under zero command.
5. The underwater search prior, surface-to-fix (plan W3), the hydrophone cases (W7, S2, S3, S11), the other-end-active cases (R5-R16).
6. Nothing was pushed since the earlier push of this branch; the work after it is committed locally.

Delegations: none this session (all work done directly); 1 earlier job (`launch_task2.sh`, Sonnet) accepted after verification, 0 rejected.
