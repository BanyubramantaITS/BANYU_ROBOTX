# Task 2: hardware checklist before the real run

Sensors on the competition UUV: bottom camera (HORUS T10, fisheye), Pixhawk (IMU, compass), RTK GPS at the surface
(u-blox F9P, `/dev/gps_rtk`, bridged to the Pixhawk as `GPS_INPUT`), Bar30 (ALT_HOLD depth hold). **No hydrophone, no DVL,
no altimeter.** Everything below is to be done on the vehicle; none of it has been done yet. Sim results:
`task2_feasibility_verdict.md`. Launch: `tools/scripts/run/launch_task2.sh --real` (profile: see section 7).

Run the checks in this order. Each has a pass criterion; do not go to the next one before it passes. Record every check
(`sub_bringup.launch.py` records by default, bags in `~/banyu_bags`).

## 1. Bench (dry, on the stand)

| # | Check | How | Pass |
|---|---|---|---|
| 1.1 | Launch script dry run | `launch_task2.sh --real --dry-run` (and with the planned `--tier`, `--sensors`) | Banner shows `Hydrophone: OFF`, the right tier and sensors; no `DANGER: RC channel overlap` |
| 1.2 | RC override channels vs `banyu_teleop` | Pixhawk `RCMAP_FORWARD/LATERAL/THROTTLE/YAW` = 2/1/3/4 = `vehicle_execution_node.rc_override.channel_*` in `task2_params.yaml`; SB = CH6 (`kill_channel` 5), SC = CH8 (`mode_channel` 7) in `teleop_rcin.yaml` | `--real` does not refuse (the script checks the YAML; check the Pixhawk `RCMAP_*` by hand in QGC). Moving SB Mid -> MANUAL, SB Down -> disarm + ESP32 kill |
| 1.3 | Bar30 on the bench | `ros2 topic echo /mavros/global_position/rel_alt` and `/banyu/vehicle/water_depth` | Steady value, noise < 0.05 m; `calibrated: true` after `prepare_autonomy`; no `SILENT`/`STUCK` health fault |
| 1.4 | GPS bridge | `run_gps_bridge.sh`; `ros2 topic echo /mavros/global_position/global` outdoors | `status.status >= 0` (fix), RTK fixed if the base is up; position within 0.1 m of a known point (RTK) |
| 1.5 | Bottom camera | `ros2 topic hz /bottom_camera/image_raw`, view the image | >= 15 Hz (the flash decoder needs it), MJPG, image not green (USB 2.0 bandwidth, `horus_camera.md`) |
| 1.6 | Recording | `sub_bringup` default `record:=True`, `record_images:=True` | A 1 min bag in `~/banyu_bags/<stamp>_task2_*` with `/banyu/*`, `/mavros/*`, `/rosout` and compressed images |

## 2. Camera calibration (dry, then in water)

| # | Check | How | Pass |
|---|---|---|---|
| 2.1 | Pipe diameter | Measure the competition pipe (or the handbook value) | `vehicle_execution_node.camera_heave.pipe_diameter_m` and the ring tracker's camera model set to it (sim: 0.0889 m, 3" Sch 40 PVC) |
| 2.2 | HORUS T10 fisheye calibration (D6) | Checkerboard, `camera_calibration` with the fisheye (equidistant) model, at the resolution used in the run | `bottom_camera_info.yaml` has `distortion_model: equidistant` and the 4 coefficients (today it is a zero `plumb_bob` placeholder); reprojection error < 0.5 px; the detector logs that it undistorts |
| 2.3 | Field of view | From the calibration: hfov = 2 atan(width / (2 fx)) of the undistorted image | `camera_heave.hfov_rad` set to it (sim 1.047 rad) |
| 2.4 | Range from the pipe width | Hold the camera over a pipe piece at 1.0, 1.6 and 2.5 m (in water: the refraction changes the fov) | The logged range is within 10% of the tape measure at each height |

## 3. Pool, surface

| # | Check | How | Pass |
|---|---|---|---|
| 3.1 | Depth datum at the surface | Float the vehicle, `prepare_autonomy` | `water_depth` 0.0 +- 0.1 m, `calibrated: true`; the datum is only taken before the run starts |
| 3.2 | GPS at the surface and under water (surface proof) | At the pool edge or outdoors: watch `/mavros/global_position/global` while the vehicle dives to 0.5 m and comes back | Under water the fix goes away (`status.status` -1, or no messages for > 2 s); it comes back within 10-30 s of surfacing. **If the Pixhawk keeps publishing a valid fix under water, the GPS surface proof (`calibrated_depth_node gps_surface`, used after a Bar30 fault) is not valid: do not rely on it** |
| 3.3 | Kill switch | SB Down in the water | Thrusters stop at once, Pixhawk disarms |

## 4. Pool, under water (tether or short runs)

| # | Check | How | Pass |
|---|---|---|---|
| 4.1 | ALT_HOLD depth hold with RC override | Dive by the mission (`--tier core`), let it hold | Depth held +- 0.1 m for 60 s, heading +- 5 deg |
| 4.2 | Forward speed (dead reckoning gain) | `TimedMove` forward 10 s at the mission speed, measure the distance | Speed known; set `ring_tracker dead_reckoning.velocity_gain` and `underwater_odometry_node velocity_gain` (achieved / commanded) |
| 4.3 | Camera lock on a pipe piece | Lay a pipe piece on the pool floor; dive with `--sensors baro` | The dive stops by camera over the pipe (`DiveToDepth` "Pipe in view"), the standoff holds 1.6 m +- 0.2 m |
| 4.4 | Ring colour | A mock node (RED, GREEN, BLUE LEDs) under the camera | Detector reads each colour at confidence >= 0.85; record the hue per colour |
| 4.5 | Magnet repair on a mock node | Mock node with the reed switch; Core repair (`CenterOnRing`, `HoldProbeOnRing`) | Probe within 0.15 m of the switch, the light turns GREEN, `HoldProbeOnRing` SUCCESS, 3 of 3 tries |
| 4.6 | Flash code (Advanced/Disruptive) | Mock node flashing the 5 s code | Decoder reads tin and circle right, 3 of 3 |
| 4.7 | Surfacing | `Surface` from 2 m | Surfaces, depth < 0.2 m, GPS fix back, no second dive |
| 4.8 | Report | Run the GCS bridge and `mock_gcs` on the GCS side at the surface (`tools/sos_integration/README.md`) | `PipelineSurvey` (and `ResourceDelivery`) received, seqs increase |

## 5. Full rehearsal (open water if possible)

A complete run with the USV `PipelineBuoys` feed (or the mock USV), a pipe with four nodes, one RED. Pass: the order, the repair and
the report, with the bag reviewed after surfacing (`/banyu/pipeline/ring_state`, `/banyu/mission/*`, `/rosout`).

## 6. Things that must stay off

- `--hydrophone` (no hydrophone is fitted; the script warns on `--real`).
- `use_dvl` (no DVL).
- `--gps-gate`, `--light-seed`, `--spawn*`, `--pipe-*` and the other scenario options (simulation only; refused on `--real`).

## 7. Profile for the run

See the verdict (`task2_feasibility_verdict.md`) for which `--sensors` profile the real run uses and why.
