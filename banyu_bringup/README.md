# Banyu Bringup (`banyu_bringup`)

This package contains launch files and bringup configurations for running the **BANYU RobotX / BlueROV2** system in both Gazebo simulation and real hardware environments.

---

## 🚀 Launch Files

### `launch/main_launch.launch.py`

Full simulation bringup script that composes Gazebo Harmonic, ArduSub SITL (`sitl.launch.py`), and the mission stack.

> **No launch file starts MAVROS.** Run it on its own with `tools/scripts/run/launch_mavros.sh`
> (`--sim` for sim time) and keep it running while the stack is relaunched.

#### Usage Commands

- **Full Simulation — Proof of Readiness (POR)**:
  ```bash
  ros2 launch banyu_bringup main_launch.launch.py mode:=por
  ```

- **Full Simulation — Task 2 (Underwater Pipeline & Rings)**:
  ```bash
  ./tools/scripts/run/launch_task2.sh --sim     # operator wrapper (see --help)
  ./tools/scripts/run/launch_mavros.sh --sim    # separate terminal
  ```
  Scored sim batches (start MAVROS themselves): `tools/scripts/sim/task2_sim_batch.sh --help`.

- **Bare ArduSub SITL Mode**:
  ```bash
  ros2 launch banyu_bringup main_launch.launch.py mode:=por use_ardusub_sitl:=False
  ```

- **Headless Mission Stack Only (against existing SITL or MAVROS)**:
  ```bash
  ros2 launch banyu_bringup mission.launch.py mode:=por
  # or Task 2 fixture mode:
  ros2 launch banyu_bringup mission.launch.py mode:=task2 use_sim:=True
  ```

- **Real Hardware Bringup Mode (onboard BlueROV2 / Mini-PC)**:
  ```bash
  ./tools/scripts/run/launch_mavros.sh          # separate terminal, after run_mavproxy.sh
  ros2 launch banyu_bringup sub_bringup.launch.py mode:=por
  ./tools/scripts/run/launch_task2.sh --real    # Task 2 (checks devices, MAVROS and ROS_DOMAIN_ID=40 first)
  ./tools/scripts/run/launch_task2.sh --real --test-mission   # Task 2 test: midpoint, 1 m dive, forward/back, report
  ```
  *(Launches `banyu_teleop` safety switches, `usb_cam` cameras, and the mission stack)*
  Devices (udev names), kill switch protocol, RTK GPS bridge and debug scripts:
  [`docs/hardware_setup.md`](../docs/hardware_setup.md).


---

## ⚙️ Launch Options

| Argument | Options | Default | Description |
| :--- | :--- | :--- | :--- |
| `mode` | `por` / `task2` | `por` | Selects which competition mission and world to launch (`por` or `task2`). |
| `use_sim` | `True` / `False` | `True` | Runs Gazebo simulation and SITL when `True`; skips sim for real ROV hardware when `False`. |
| `use_sim_time` | `True` / `False` | `True` | Sets `use_sim_time` parameter for ROS 2 nodes. |
| `use_ardusub_sitl` | `True` / `False` | `True` | Uses `sim_vehicle.py` SITL wrapper (UDP 14550 for QGC) when `True`; uses bare `ardusub` when `False`. |
| `use_gps` | `True` / `False` | `True` (`main_launch`); `sub_bringup`: `False` for `por`, `True` for `task2` | `False` removes the SITL GPS from boot and starts `nogps_odom_beacon` (GPS-denied, like the real AUV in POR). `launch_por.sh --sim` and `launch_sim.sh por` pass `False`. |
| `sensors` | `full` / `baro` / `minimal` / `gamble` | `full` | Task 2 sensor profile (`config/task2_sensors_<s>.yaml`): `full` = Bar30 + ALT_HOLD, dive to the fixed inspection depth; `baro` = Bar30 + ALT_HOLD, the dive stops by camera at the pipe (Bar30 as the floor), pipe-width standoff, dive point 1.5 m inside the start buoy, ring order dead-reckoned along the buoy line; `minimal` = no depth sensor (STABILIZE); `gamble` = GPS to the dive point (`dive_point:=auto`, the default, is `midpoint` for gamble), timed dive, guessed order. `launch_task2.sh --sensors`. |
| `use_hydrophone` | `True` / `False` | `False` | Task 2 only. Opt in to the hydrophone: `pinger_localization`, the sim `HydrophoneArray` plugin and bridge, pinger homing in `SearchPipeline` and pinger corrections in `underwater_odometry_node` (`config/task2_hydrophone_on.yaml`). The competition UUV has no hydrophone; with `False` nothing publishes `/banyu/pinger/observation` and no decision reads pinger data. `launch_task2.sh --hydrophone`. |
| `test_mission` | `True` / `False` | `False` | Task 2 only. Fly `task2_test_mission.xml` instead of the survey: GPS to the midpoint between the USV buoys, face the end buoy, dive to 1 m, timed forward and backward legs, surface, Core report (`config/task2_test_mission.yaml`, `docs/task2_test_mission.md`). `launch_task2.sh --test-mission`. |
| `record` | `True` / `False` | `True` (`sub_bringup`), `False` (`main_launch`) | Records the run to a rosbag, see [Mission Recording](#-mission-recording). In sim the bag uses sim time and includes the ground truth `/sim/ground_truth/odom` (POR). |
| `record_images` | `True` / `False` | `True` (`sub_bringup`), `False` (`main_launch`) | Includes compressed camera images in the bag. Raw images are never recorded. |
| `por_params_file` | path | `config/por_params_timed_safe.yaml` | POR profile. Profiles: `por_params_timed_safe.yaml`, `por_params_timed_fastest.yaml`, `por_params_replay.yaml` (see `docs/por/README.md` §2). One file serves RC and no-RC runs. |
| `bypass_rc` | `True` / `False` | `False` | No transmitter: vehicle status (POR and Task 2) ignores RC freshness and takes the mode from the ESP32 trigger (`/banyu/trigger/command` service, or `/banyu/trigger/cmd`: `'2'` auto, `'1'` manual, `'0'` kill; Task 2 starts MANUAL). Use with `enable_teleop:=False enable_esp32_trigger:=True`; `launch_por.sh --no-rc` and `launch_task2.sh --no-rc` set all three. |
| `replay_track` | `latest`, a name or a path | `latest` | Teach track flown by profiles with `use_replay: true` (newest `.csv` in `$BANYU_REPLAY_DIR`, default `~/banyu_replays`). See `docs/por/README.md` §4.8. |

---

## 🎥 Mission Recording

The link drops as soon as the AUV dives, so `sub_bringup.launch.py` records every run to a
rosbag on the vehicle by default (`record:=False` to skip). The recorder is
`scripts/record_mission.sh`, which can also be run on its own:

```bash
ros2 run banyu_bringup record_mission.sh por               # standalone, label "por"
ros2 run banyu_bringup record_mission.sh task2 --no-images # much smaller bag
```

Each run goes to `~/banyu_bags/<YYYYmmdd_HHMMSS>_<mode>/` (override with `BANYU_BAG_DIR`):

- `bag/`: MCAP bag with zstd chunk compression. It records all topics, plus hidden action
  feedback/status topics, and skips raw images and `/test*`.
- `run_info.txt`: start/stop time, host, git commit and recorder settings.
- `params/`: a copy of the parameter YAML (profile) the run used.

`tools/scripts/por/launch_por.sh --real` records by default and names the folder after the
profile (e.g. `20261026_133006_por_timed_safe`). It shows the bag location and free disk
in its banner, and takes `--no-record` and `--no-record-images`.

Built to survive a dive that ends badly:

- The bag splits every 60 s and the write cache is kept small, so a power cut loses at most
  the last ~60 s.
- Recording stops by itself if free disk drops below 2 GB (`BANYU_BAG_MIN_FREE`, in MB).
- Stopping the launch (Ctrl-C) closes the bag cleanly.

### After the run

```bash
# copy runs off the vehicle (from the laptop)
rsync -avP <user>@<auv-ip>:banyu_bags/ ~/banyu_bags/

# only if the AUV lost power before the bag was closed (no metadata.yaml):
ros2 run banyu_bringup recover_bag.sh ~/banyu_bags/<run>/bag

ros2 bag info ~/banyu_bags/<run>/bag
ros2 bag play ~/banyu_bags/<run>/bag   # replay into RViz / rqt
```

`recover_bag.sh` moves the truncated last file into `bag/partial/` and rebuilds the metadata
from the complete files. The `.mcap` files also open directly in Foxglove or PlotJuggler. Useful
topics for reviewing performance: `/banyu/mission/run_lifecycle`, `/banyu/vehicle_status`,
`/mavros/local_position/pose`, `/mavros/setpoint_raw/local`, `/banyu/detection`,
`/banyu/actions/*/_action/feedback` and `/rosout` (every node's log output).

---

## 🛁 Navigation Without GPS

The AUV has no DVL and, for POR, no GPS, so EKF3 has no horizontal position source.
`use_gps:=False` (the POR default, also passed by `launch_por.sh --sim`) runs
`scripts/nogps_odom_beacon.py`, which:

- selects EKF3 source set 2 with `MAV_CMD_SET_EKF_SOURCE_SET` (resent every 5 s, since the
  selection does not survive an autopilot reset), and
- streams a 10 Hz zero-delta `VISION_POSITION_DELTA` on MAVProxy's 14552 output, which takes
  EKF3 to `AID_RELATIVE`.

`/mavros/local_position/pose` then carries Bar30 depth and compass heading; horizontal position
stays at the origin. In simulation `use_gps:=False` also removes the SITL GPS from boot
(`config/sim_no_gps.parm`), like the real vehicle. With `use_gps:=True` (Task 2) the beacon does
not run and EKF3 stays on source set 1 (ArduPilot's GPS defaults).

The source set 2 parameters live in `config/banyu_defaults.parm`:

| Parameter | Value | Why |
|---|---|---|
| `VISO_TYPE` | `1` (MAVLink) | Accept the beacon's `VISION_POSITION_DELTA`. |
| `VISO_POS_X/Y/Z` | `0 / 0 / 0` | No physical sensor; a lever arm would turn every yaw into a fake translation. |
| `EK3_SRC2_VELXY` | `6` (EXTNAV) | `readyToUseBodyOdm()` needs EXTNAV or WHEEL_ENCODER, or the beacon cannot aid the filter. |
| `EK3_SRC2_POSXY` | `0` (NONE) | No absolute position; GPS(3) would wait for a fix forever. |
| `EK3_SRC2_POSZ` / `VELZ` | `1` / `0` | Baro depth, baro-derived vertical velocity. |
| `EK3_SRC2_YAW` | `1` (COMPASS) | Not the SRC2 default of 0/NONE: without a yaw source EKF3 never aligns and aiding is blocked. |

Check it on a running system: the launch log shows
`[nogps_odom_beacon] selecting EKF3 source set 2`, and
`ros2 topic hz /mavros/local_position/pose` is non-zero. Self-check without ROS:
`ros2 run banyu_bringup nogps_odom_beacon.py --self-check`.

The beacon reports zero motion, so EKF velocity is always ~0. That is fine for POR (ALT_HOLD +
RC override, open-loop speeds) but **never** feed it to GUIDED velocity control
(`docs/task2_underwater_position.md`).

