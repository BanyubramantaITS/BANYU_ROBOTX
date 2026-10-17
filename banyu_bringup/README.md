# Banyu Bringup (`banyu_bringup`)

This package contains launch files and bringup configurations for running the **BANYU RobotX / BlueROV2** system in both Gazebo simulation and real hardware environments.

---

## 🚀 Launch Files

### `launch/main_launch.launch.py`

Full simulation bringup script that composes Gazebo Harmonic, ArduSub SITL, MAVROS, and the mission stack.

#### Usage Commands

- **Full Simulation — Proof of Readiness (POR)**:
  ```bash
  ros2 launch banyu_bringup main_launch.launch.py mode:=por
  ```

- **Full Simulation — Task 2 (Underwater Pipeline & Rings)**:
  ```bash
  ros2 launch banyu_bringup main_launch.launch.py mode:=task2
  ```

- **Bare ArduSub SITL Mode**:
  ```bash
  ros2 launch banyu_bringup main_launch.launch.py mode:=por use_ardusub_sitl:=False
  ```

- **Headless Mission Stack Only (No Gazebo GUI)**:
  ```bash
  ros2 launch banyu_bringup mission.launch.py mode:=por
  # or Task 2 fixture mode:
  ros2 launch banyu_bringup mission.launch.py mode:=task2 use_sim:=True
  ```

- **Real Hardware Mode (onboard BlueROV2)**:
  ```bash
  ros2 launch banyu_bringup mission.launch.py mode:=por use_sim:=False fcu_url:=udp://192.168.2.1:14550@
  ```

- **No-DVL Negative Control** (see [Navigation Sources](#-navigation-sources-gps-on-the-surface-dvl-underwater)):
  comment out the `banyu::DvlMavlink` plugin in `banyu_simulation/worlds/main_world.world`.

---

## ⚙️ Launch Options

| Argument | Options | Default | Description |
| :--- | :--- | :--- | :--- |
| `mode` | `por` / `task2` | `por` | Selects which competition mission and world to launch (`por` or `task2`). |
| `use_sim` | `True` / `False` | `True` | Runs Gazebo simulation and SITL when `True`; skips sim for real ROV hardware when `False`. |
| `use_sim_time` | `True` / `False` | `True` | Sets `use_sim_time` parameter for ROS 2 nodes. |
| `use_ardusub_sitl` | `True` / `False` | `True` | Uses `sim_vehicle.py` SITL wrapper (UDP 14550 for QGC) when `True`; uses bare `ardusub` when `False`. |
| `fcu_url` | string | `udp://127.0.0.1:14551@` | Target MAVLink FCU address (`udp://127.0.0.1:14551@` for SITL to reserve 14550 for QGC; `udp://192.168.2.1:14550@` for physical BlueROV2). |

---

## 🛁 Navigation Sources: GPS on the Surface, DVL Underwater

ArduSub's EKF3 carries three switchable source sets. Two are used:

| | Set | Horizontal position | Horizontal velocity | Depth | Yaw |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Surface** | `EK3_SRC1_*` | GPS | GPS | Baro | Compass |
| **Dive** | `EK3_SRC2_*` | *none* | DVL (ExternalNav) | Baro | Compass |

`scripts/ekf_source_switch.py` flips between them on depth. **Nothing in
ArduPilot switches source sets on depth by itself** — the mechanism exists (RC
aux `RCx_OPTION 90`, Lua `ahrs:set_posvelyaw_source_set()`, or
`MAV_CMD_SET_EKF_SOURCE_SET`), but the decision is ours.

### Why the ordering is the point

`setOrigin()` rejects a second origin — `validOrigin` is sticky
(`AP_NavEKF3_Control.cpp:698`). A surface GPS fix anchors the EKF origin **once**;
diving to the DVL then dead-reckons from a georeferenced point rather than from
nowhere. Position still drifts underwater — the DVL bounds the drift *rate*, it
does not eliminate drift — but the estimate stays tied to a real place, and
surfacing re-acquires absolute position.

That is the entire reason to prefer this over a DVL-only setup.

### The DVL chain

```
Gazebo DVL sensor  ──gz topic /dvl/velocity──▶  banyu::DvlMavlink (world system)
                                                          │
                                                 VISION_POSITION_DELTA
                                                  (UDP 127.0.0.1:14552)
                                                          ▼
                                                    ArduSub SITL
                                             writeBodyFrameOdom() → EKF3
```

Sim only. Real hardware runs the A50's own driver instead; everything downstream
of `VISION_POSITION_DELTA` is unchanged.

**Why MAVLink straight to SITL and not MAVROS:**

- **MAVROS cannot emit `VISION_POSITION_DELTA`.** None of its 40 plugins does,
  and that message is the *only* one that bootstraps EKF3 aiding from a DVL
  alone. `VISION_SPEED_ESTIMATE` and `ODOMETRY`-twist land in
  `writeExtNavVelData()`, which is in neither `AID_NONE` exit branch.
- **Direct MAVLink injection:** `banyu::DvlMavlink` converts `gz.msgs.DVLVelocityTracking` directly to `VISION_POSITION_DELTA` into SITL, avoiding extra ROS hop latency.

So a gz-sim world system (`banyu_simulation/src/DvlMavlink.cc`, loaded by
`main_world.world`) subscribes to the gz topic and speaks MAVLink straight to
SITL on its own endpoint (`--out=udp:127.0.0.1:14552`, added to the
`sim_vehicle.py` invocation — it cannot share MAVROS' 14551). It owns the entire
frame conversion: Gazebo z-up body frame → ArduPilot FRD. Being part of the
world, it runs whenever that world does; there is no separate process to launch.

### The switching node

`scripts/ekf_source_switch.py` — a real ROS node, so it *is* a launch `Node`.
Runs in **both** sim and hardware: this is vehicle behaviour, not sim plumbing.

| Parameter | Default | |
| :--- | :--- | :--- |
| `dive_depth` | `0.5` m | Deeper than this → SRC2 (DVL). |
| `surface_depth` | `0.2` m | Shallower than this → SRC1 (GPS). |

The gap between them is hysteresis — inside the band the active set is held, so
a vehicle bobbing at the waterline does not flap the EKF between sources.

Depth comes from `/mavros/global_position/rel_alt` (negative below the surface),
**not** `local_position/pose`. `local_position` z is relative to the EKF
*origin* — wherever the vehicle happened to be when the origin was set, which in
this sim is on the pool floor. Measured against it, a sub sitting 2.5 m down
reports a depth of 0.07 m and the switch never fires; that bug was caught by the
first end-to-end run. `rel_alt` is relative to home, and ArduSub pins home
altitude to the water surface (measured offset in SITL: 0.14 m, comfortably
inside the hysteresis band).

There is still no feedback loop: `EK3_SRC*_POSZ` is Baro in *both* sets, so this
number does not depend on the horizontal source being chosen.

> `/mavros/altitude` would be the more direct source, but the `altitude` plugin
> is in `mavros_config.yaml`'s denylist.

Switching goes out as `MAV_CMD_SET_EKF_SOURCE_SET` (42007, param1 1-based) over
MAVROS' `/mavros/cmd/command`. A rejected or failed call is retried on the next
depth sample.

Logic check, no ROS required:

```bash
ros2 run banyu_bringup ekf_source_switch.py --self-check
```

### Parameters

All in [`config/banyu_defaults.parm`](config/banyu_defaults.parm), which
`sim_vehicle.py` loads via `--add-param-file`.

| Parameter | Value | Why |
| :--- | :--- | :--- |
| `VISO_TYPE` | `1` | MAVLink visual odometry backend. |
| `VISO_POS_X/Y/Z` | `0 / 0 / 0.10` | Lever arm to the sensor in FRD (z **down**). Mirrors `dvl_z = -0.10` in `bluerov2_gz`'s `model.xacro`, which is z-up. |
| `VISO_ORIENT` | `0` | **Not `25` (Down).** ArduSub rotates the incoming delta by this, and the bridge already emits vehicle-frame FRD. `25` would rotate correct data into nonsense. |
| `EK3_SRC2_VELXY` | `6` (EXTNAV) | `readyToUseBodyOdm()` returns false unless this is EXTNAV or WHEEL_ENCODER. This line is what lets the DVL aid the filter at all. |
| `EK3_SRC2_POSXY` | `0` (NONE) | A DVL gives velocity, not absolute position. GPS(3) here would make EKF3 wait at depth for a fix it cannot have. |
| `EK3_SRC2_YAW` | `1` (COMPASS) | **Not the SRC2 default of `0`/NONE.** With no yaw source EKF3 never completes yaw alignment and aiding is blocked outright — the DVL looks dead at depth for a reason that has nothing to do with the DVL. |
| `EK3_SRC2_POSZ` / `VELZ` | `1` / `0` | Baro depth, baro-derived vertical velocity. Defaults, restated because a source set is read as a unit. |

**`EK3_SRC1_*` is deliberately absent from the file.** ArduPilot's shipped
defaults already *are* the surface configuration — `POSXY 3`, `VELXY 3`,
`VELZ 3`, `POSZ 1`, `YAW 1` (`AP_NavEKF_Source.cpp:32-60`). Do not re-add them
as no-ops.

### Verifying which source is really in use

```bash
ros2 topic echo /mavros/estimator_status
```

| Flag | Surface (SRC1) | Dive, DVL fusing (SRC2) | Dive, nothing aiding |
| :--- | :--- | :--- | :--- |
| `const_pos_mode_status_flag` | `false` | `false` | **`true`** |
| `pos_horiz_rel_status_flag` | `true` | **`true`** | `false` |
| `pos_horiz_abs_status_flag` | **`true`** | `false` | `false` |

> [!IMPORTANT]
> **SITL's synthetic GPS does not cut out when the vehicle submerges.** It is
> derived from Gazebo ground truth and keeps streaming at depth. So "EKF3 is
> healthy underwater" proves nothing by itself — what proves it is
> `pos_horiz_abs` going **clear** once SRC2 is selected, because SRC2's
> `POSXY` is NONE. Ground truth leaking in would keep that flag set.
>
> This is the same trap the earlier DVL-only configuration solved by setting
> `SIM_GPS1_ENABLE 0`. That is no longer the right setting — the surface half
> needs a GPS to exist — so the discriminator moved from "no GPS exists" to
> "the active source set cannot see it".

Measured, holding station at 0.85 m in the PoR pool, then surfacing, then
returning to depth — SITL's synthetic GPS enabled and streaming throughout:

```
[dive1  ] depth= 0.85  flags=0x12f  ABS=clear  REL=SET  CONST_POS=clear    (50 s)
[surface] depth= 0.00  flags=0x33f  ABS=SET    REL=SET  CONST_POS=clear    (40 s)
[dive2  ] depth= 0.97  flags=0x12f  ABS=clear  REL=SET  CONST_POS=clear    (50 s)
```

with the switch firing cleanly in both directions:

```
depth 0.12 m -> SRC1/GPS   EKF source set 1 accepted
depth 0.56 m -> SRC2/DVL   EKF source set 2 accepted
```

`POS_HORIZ_ABS` going clear at depth *while a healthy GPS is still streaming* is
the result that matters — it is only reachable by the active source set refusing
to look at it.

Second readout, autopilot-side: every parsed `VISION_POSITION_DELTA` is written
to the `VISO` dataflash message whether or not the EKF uses it
(`AP_VisualOdom_Backend.cpp:73`). `VISO` present *and* `const_pos_mode` still
true means arriving but rejected — a different fault from never arriving.

Third, the bridge's own `--verbose` output:

```
[dvl] sent=1321 skipped=0 dt=0.099s v_frd=(-0.001,+0.006,+0.132) conf=99
```

`skipped` counts gz reports without bottom lock; those are dropped rather than
sent, since water-track velocity is relative to a moving medium.

### Traps

> [!IMPORTANT]
> **The vehicle must be driven off the pool floor before any result means
> anything.** An idle BlueROV2 Heavy free-falls at ~0.13 m/s and beds into the
> floor at 2.5 m — and it does this *during launch startup*, before a test
> script can even connect, so arming quickly is not sufficient on its own. A DVL
> sitting on the bottom has no bottom lock: measured `skipped=428` on one run and
> `skipped=1536` on another, versus **zero** skips once holding station at
> 0.85 m. The symptom is `stopped aiding` roughly 20–30 s in, followed by EKF3
> falling back to whatever else it can find, which looks exactly like a fusion
> failure and is not one.
>
> Arm into `ALT_HOLD` and climb to ~1 m depth first. `ALT_HOLD` then holds
> station there indefinitely (measured drift over 50 s: 0.85 → 0.85 m).

- **Reap stale processes before every run.** `sim_vehicle.py`'s `ardusub` and
  `mavproxy` children outlive the launch, and so do `banyu_mission`,
  `por_detection`, `robot_state_publisher` and `image_bridge`. A surviving
  `ardusub` cannot bind port 5501 and the gz plugin logs `ArduPilot controller
  has reset`; a surviving `banyu_mission` holds Groot2's ZMQ port and the next
  run dies with `Address already in use`, meaning no mission ran at all.
- **The switcher overrides manual source selection.** If you flip sources by
  hand (QGC, RC aux 90) it will be undone at the next threshold crossing. Stop
  the node if you want manual control.
- **Underwater, anything needing a global fix is dead** while SRC2 is active:
  geofence, RTL-to-home. `/mavros/local_position/pose` keeps working, because
  `AID_RELATIVE` maintains a local origin.

### Not yet exercised

Stated plainly, because the failure mode of this whole exercise is a config that
reads green while proving nothing:

- **The `proof_of_readiness` mission never surfaces.** The vehicle spawns at
  `z = -2` in a 2.5 m pool, i.e. already past `dive_depth`, so a mission run
  selects SRC2 at the first depth sample and stays there. The handover has been
  verified by driving the vehicle by hand, **not** by any automated mission run.
- **The mission has not been re-run on this configuration.** The evidence above
  is a hold-surface-hold cycle, not `NavToGate`→`MarkerOrbit`. The earlier full
  mission run was made with the DVL wired into SRC1 and `SIM_GPS1_ENABLE 0`.
- **Real DVL hardware, and a real GPS.** A BlueROV2 has no GPS antenna as
  standard — surface float, mast, or USBL is an open hardware question. Until
  that is settled, SRC1 on real hardware points at a source the vehicle does
  not have.
- **Accuracy.** The run shows EKF3 *accepts and sustains* the DVL velocity and
  that the source selection takes effect; no drift measurement has been made.

### Dependencies

`banyu::DvlMavlink` builds against gz-transport13, gz-msgs10 and the header-only
ROS `mavlink` package.
`ekf_source_switch.py` needs `rclpy`, `geometry_msgs` and `mavros_msgs`. It is
installed as a program by `CMakeLists.txt`.
