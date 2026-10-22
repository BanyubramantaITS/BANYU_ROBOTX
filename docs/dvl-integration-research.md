# DVL integration research — BANYU_ROBOTX

> **Note:** simulation only. The team has no physical DVL, so none of this is used in the real world.

Research date: 2026-08-15. Every claim below carries an inline citation to a primary source
(upstream source file + line, official protocol doc, or a locally installed upstream artifact).
Anything not confirmed from a primary source is marked **UNVERIFIED**.

Local toolchain this was checked against (from `dpkg -l`): `gz-harmonic 1.0.0-1~jammy`,
`libgz-sim8 8.15.0`, `libgz-sensors8 8.2.2`, `libgz-msgs10 10.4.0`,
`ros-humble-ros-gzharmonic{,-bridge} 0.244.12-3jammy`, `ros-humble-mavros{,-extras} 2.14.0`.
ArduPilot / MAVROS / gz upstream sources were fetched from their canonical repos on the date above.

---

## Bottom line

1. **"Adding DVL support" is two unrelated projects.** In simulation the sub already has a
   perfect position estimate because SITL synthesises a GPS from Gazebo ground truth
   ([`libraries/SITL/SITL.cpp` L730-737](https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SITL.cpp#L730-L737), `SIM_GPS1_ENABLE`). On real hardware there is no GPS at all, so
   `/mavros/local_position/pose` — which every `NavToGate` / `ReturnToStart` P-controller in
   `banyu_mission` reads — has no absolute source. **The DVL's real job is to make the hardware
   behave like the sim, not to improve the sim.**
2. **ArduSub does consume DVL data**, via `AP_VisualOdom` with `VISO_TYPE=1` (MAVLink). Two
   distinct EKF3 paths exist and they are *not* interchangeable (§1.3).
3. **Only `VISION_POSITION_DELTA` (body-frame odometry) can start EKF3 aiding from a DVL alone.**
   `VISION_SPEED_ESTIMATE` / `ODOMETRY`-twist feed `writeExtNavVelData`, which is *not* one of the
   conditions that moves EKF3 out of `AID_NONE`
   ([`AP_NavEKF3_Control.cpp` L283-299](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp#L283-L299)). This is why BlueRobotics' own driver defaults to `POSITION_DELTA`.
4. **Gazebo Harmonic ships a real DVL sensor** (`gz-sensors8-dvl`, `gz-sim8-dvl-system`, both
   installed here), but **`ros_gz_bridge` on Humble cannot bridge it** — the mapping exists only
   from `jazzy` onward (§3.2). Backporting is ~1 file.
5. **`ardupilot_gazebo` has no DVL or visual-odometry channel** — its FDM JSON carries only
   `imu/gyro/accel_body/position/quaternion/velocity/rng_1..6/windvane` (§3.3). Any DVL must go
   ROS → MAVROS → MAVLink → ArduSub.
6. **Two live landmines in this repo**: `mavros_config.yaml` denies `vision_speed_estimate`, but
   the plugin in MAVROS 2.14.0 is named `vision_speed` — the deny entry is a no-op (§4.3). And the
   `odometry` plugin, if used, needs `map_ned` / `base_link_frd` TF frames that **MAVROS does not
   publish** (§4.2).
7. **A DVL does not change the "velocity-loop, not position-setpoint" finding in AGENTS.md.**
   That finding is about AC_WPNav's behaviour on `0xDF8` position targets, not about estimate
   quality. A DVL makes the *feedback* (`ctx_->pose`) trustworthy on hardware; the *actuation*
   stays velocity-only (§5.5).

**The real choice** is: (a) sim-fidelity work (gz DVL sensor + custom bridge) which buys almost
nothing today, versus (c) hardware enablement (Water Linked A50 + BlueOS/first-party driver) which
is the only route that unblocks GPS-denied autonomy — versus (d) keeping the DVL out of the EKF
entirely and dead-reckoning in `banyu_mission`, which is cheap but abandons ArduSub's depth/attitude
fusion. Recommendation is in §5.6.

---

## 1. ArduSub / ArduPilot — how a DVL actually gets into the EKF

### 1.1 Which MAVLink messages ArduSub handles

All four candidate messages are dispatched in the **common** GCS layer, which `GCS_MAVLINK_Sub`
falls through to — `GCS_MAVLINK_Sub::handle_message()` handles a short vehicle-specific list and
then calls `GCS_MAVLINK::handle_message(msg)`
([`ArduSub/GCS_MAVLink_Sub.cpp` L570, L779](https://github.com/ArduPilot/ardupilot/blob/master/ArduSub/GCS_MAVLink_Sub.cpp#L570)).
So ArduSub inherits all of these:

| MAVLink msg | ID | Dispatch | Handler | Ends up in EKF3 as |
|---|---|---|---|---|
| `VISION_POSITION_DELTA` | 11011 | [`GCS_Common.cpp` L4548-4550](https://github.com/ArduPilot/ardupilot/blob/master/libraries/GCS_MAVLink/GCS_Common.cpp#L4548-L4550) | `handle_vision_position_delta` (L4020-4027) | `writeBodyFrameOdom` — **body-frame velocity fusion** |
| `VISION_SPEED_ESTIMATE` | 103 | `GCS_Common.cpp` L4572-4573 | `handle_vision_speed_estimate` (L4151-4166) | `writeExtNavVelData` — **NED velocity fusion** |
| `ODOMETRY` | 331 | `GCS_Common.cpp` L4564-4565 | `handle_odometry` (L4061-4093) | **both** `writeExtNavData` (pose) *and* `writeExtNavVelData` (velocity) |
| `GLOBAL_VISION_POSITION_ESTIMATE` | 101 | `GCS_Common.cpp` L4556-4557 | `handle_global_vision_position_estimate` (L4038-4045) | `writeExtNavData` — pose only |

Everything above is gated on `HAL_VISUALODOM_ENABLED`, which defaults to
`HAL_PROGRAM_SIZE_LIMIT_KB > 1024`
([`AP_VisualOdom_config.h`](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_VisualOdom/AP_VisualOdom_config.h)) — true for SITL and for Linux boards (Navigator/Raspberry Pi). On a
1 MB flash target it would be compiled out.

Important frame details, straight from the handlers:

- **`VISION_SPEED_ESTIMATE` is interpreted as earth-frame NED, not body-frame.** The handler passes
  `{m.x, m.y, m.z}` straight through
  ([`GCS_Common.cpp` L4151-4166](https://github.com/ArduPilot/ardupilot/blob/master/libraries/GCS_MAVLink/GCS_Common.cpp#L4151-L4166)), and the backend comment reads
  *"consume vision velocity estimate data and send to EKF, velocity in NED meters per second"*
  ([`AP_VisualOdom_MAV.cpp` L69-71](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp#L69-L71)). The MAVLink XML agrees: fields are
  *"Global X speed / Global Y speed / Global Z speed"*
  ([`common.xml` msg 103](https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml)).
  **A raw DVL body-frame velocity put into this message is only correct when the vehicle heading is
  zero and it is level.**
- **`ODOMETRY` is accepted only in `LOCAL_FRD` / `BODY_FRD`.** Any other frame pair is silently
  dropped (`GCS_Common.cpp` L4071-4075). The handler then rotates the FRD body velocity into NED
  with the message's own quaternion before calling `handle_vision_speed_estimate`
  (`GCS_Common.cpp` L4088-4092). This is the only ArduPilot path that does the body→earth rotation
  for you.
- **`VISION_POSITION_DELTA` is genuinely body-frame.** `angle_delta` and `position_delta` are
  defined relative to `MAV_FRAME_BODY_FRD`
  ([`ardupilotmega.xml` msg 11011](https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/ardupilotmega.xml)), and the backend rotates them by `VISO_ORIENT` before
  calling `AP::ahrs().writeBodyFrameOdom(...)`
  ([`AP_VisualOdom_Backend.cpp` L41-78](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_VisualOdom/AP_VisualOdom_Backend.cpp#L41-L78)).

Entry points into the estimator go `AP::ahrs().writeXxx()` → `AP_AHRS::writeExtNavVelData`
([`AP_AHRS.cpp` L1675-1683](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_AHRS/AP_AHRS.cpp#L1675-L1683)) → `NavEKF3::writeExtNavVelData`. `AP_DAL` mirrors the same three
calls for replay/log determinism
([`AP_DAL.h` L240-245](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_DAL/AP_DAL.h#L240-L245)).

### 1.2 Parameters — real names and values

The `VISO_` prefix is not registered by ArduSub; it is a common vehicle subgroup:
`AP_SUBGROUPINFO(visual_odom, "VISO", 3, AP_Vehicle, AP_VisualOdom)`
([`AP_Vehicle.cpp` L48-50](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Vehicle/AP_Vehicle.cpp#L48-L50)), so ArduSub gets it automatically. Full table verbatim from
[`libraries/AP_VisualOdom/AP_VisualOdom.cpp`](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_VisualOdom/AP_VisualOdom.cpp) L31-115:

| Param | Line | Values / Range | Default | Meaning |
|---|---|---|---|---|
| `VISO_TYPE` | L31-37 | `0:None, 1:MAVLink, 2:IntelT265, 3:VOXL(ModalAI)` | `0` | **Use `1` for a DVL.** `1` instantiates `AP_VisualOdom_MAV` (`AP_VisualOdom.cpp` L140-143). |
| `VISO_POS_X` / `_Y` / `_Z` | L39-62 | `-5 .. 5`, metres | `0.0` | Sensor position offset in body frame. Passed to `writeBodyFrameOdom` as `posOffset` (`AP_VisualOdom_Backend.cpp` L68). |
| `VISO_ORIENT` | L64-69 | `0:Forward, 2:Right, 4:Back, 6:Left, 24:Up, 25:Down` | `ROTATION_NONE` (0) | Rotation applied to `VISION_POSITION_DELTA` deltas only (`AP_VisualOdom_Backend.cpp` L48-52). **Not** applied to `VISION_SPEED_ESTIMATE`. |
| `VISO_SCALE` | L71-75 | float | `1.0` | Scale on *position* estimates (`AP_VisualOdom_MAV.cpp` L30-31). |
| `VISO_DELAY_MS` | L77-83 | `0 .. 250`, ms | `10` | Sensor delay vs inertial data; subtracted from the timestamp inside EKF3 (`AP_NavEKF3_Measurements.cpp` L128, L1152). |
| `VISO_VEL_M_NSE` | L85-91 | `0.05 .. 5.0` m/s | `0.1` | Floor on velocity measurement noise (`AP_VisualOdom_MAV.cpp` L79). |
| `VISO_POS_M_NSE` | L93-99 | `0.1 .. 10.0` m | `0.2` | Floor on position noise. |
| `VISO_YAW_M_NSE` | L101-107 | `0.05 .. 1.0` rad | `0.2` | Floor on yaw noise. |
| `VISO_QUAL_MIN` | L109-115 | `-1 .. 100` % | `0` | Data below this quality is dropped (`AP_VisualOdom_MAV.cpp` L49, L77). |

There is **no `VISO_ENABLE`** — `VISO_TYPE` carries `AP_PARAM_FLAG_ENABLE`.

EKF source selection, verbatim from
[`libraries/AP_NavEKF/AP_NavEKF_Source.cpp`](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF/AP_NavEKF_Source.cpp) L27-60:

| Param | Line | Values | Default |
|---|---|---|---|
| `EK3_SRC1_POSXY` | L27-32 | `0:None, 3:GPS, 4:Beacon, 6:ExternalNav` | `3` (GPS) |
| `EK3_SRC1_VELXY` | L34-39 | `0:None, 3:GPS, 4:Beacon, 5:OpticalFlow, 6:ExternalNav, 7:WheelEncoder` | `3` (GPS) |
| `EK3_SRC1_POSZ` | L41-46 | `0:None, 1:Baro, 2:RangeFinder, 3:GPS, 4:Beacon, 6:ExternalNav` | `1` (Baro) |
| `EK3_SRC1_VELZ` | L48-53 | `0:None, 3:GPS, 4:Beacon, 6:ExternalNav` | `3` (GPS) |
| `EK3_SRC1_YAW` | L55-60 | `0:None, 1:Compass, 2:GPS, 3:GPS with Compass Fallback, 6:ExternalNav, 8:GSF` | `1` (Compass) |
| `EK3_SRC_OPTIONS` | L136-141 | bit0: fuse *all* velocity sources; bit1: align extnav pos when using optflow; bit3: SRC per core | `0` |

Enable / selection:

- `EK3_ENABLE` — `0:Disabled, 1:Enabled`, default `1`, **reboot required**
  ([`AP_NavEKF3.cpp` L165-171](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3.cpp#L165-L171)). Its description explicitly says enabling EKF3 does not
  mean it is *used*; you must also set `AHRS_EKF_TYPE=3`.
- `AHRS_EKF_TYPE` — `0:Disabled, 2:Enable EKF2, 3:Enable EKF3, 10:Sim, 11:ExternalAHRS`
  ([`AP_AHRS.cpp` L171-176](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_AHRS/AP_AHRS.cpp#L171-L176)). ArduSub registers `AHRS_` and `EK3_` normally
  ([`ArduSub/Parameters.cpp` L578, L656-659](https://github.com/ArduPilot/ardupilot/blob/master/ArduSub/Parameters.cpp#L578)).
- **No ArduSub-specific DVL parameters exist.** Grepping `ArduSub/Parameters.cpp` for `VISO`
  returns nothing; everything is common-library. There is also **no ArduSub documentation page for
  DVL** — `ardupilot.org/sub/docs/{dvl,common-dvl,common-visual-odometry}.html` all return 404
  (probed 2026-08-15); only the Copter VIO page exists
  (`https://ardupilot.org/copter/docs/common-vio-tracking-camera.html`, HTTP 200).

**The de-facto reference parameter set** is the one BlueRobotics' own extension applies
([`BlueOS-Water-Linked-DVL/dvl-a50/dvl.py` L283-324](https://github.com/bluerobotics/BlueOS-Water-Linked-DVL/blob/master/dvl-a50/dvl.py#L283-L324)):

```
AHRS_EKF_TYPE   3
EK2_ENABLE      0
EK3_ENABLE      1
VISO_TYPE       1        # MAVLink
# "dvl" profile:
EK3_GPS_TYPE    3        # disable GPS in EKF3
EK3_SRC1_POSXY  6        # EXTNAV
EK3_SRC1_VELXY  6        # EXTNAV
EK3_SRC1_POSZ   1        # BARO   <-- depth stays on the pressure sensor
# optional:
RNGFND1_TYPE    10       # MAVLINK, DVL altitude as a rangefinder
```

Note it leaves `EK3_SRC1_VELZ` and `EK3_SRC1_YAW` alone, and keeps depth on the barometer. That
matches AGENTS.md's empirical finding that velocity-only setpoints hold depth precisely — depth is
the one axis that is already good.

**UNVERIFIED:** `EK3_GPS_TYPE` is set by the BlueOS driver but I did not locate its definition in
current ArduPilot master (it may be a legacy/renamed parameter). Verify against the running
firmware's parameter list before copying it.

### 1.3 Rate, latency, frames, and the EKF3 gating that actually matters

**Rate limits inside EKF3:**

- Body-frame odom (`writeBodyFrameOdom`) is dropped if it arrives faster than
  `sensorIntervalMin_ms = 50` → **max 20 Hz**
  ([`AP_NavEKF3_Measurements.cpp` L123](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp#L123), [`AP_NavEKF3.h` L517](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3.h#L517)).
- External-nav velocity (`writeExtNavVelData`) is dropped faster than
  `extNavIntervalMin_ms = 20` → **max 50 Hz** (`AP_NavEKF3_Measurements.cpp` L1145,
  `AP_NavEKF3.h` L519).

A Water Linked A50 runs 2–15 Hz depending on altitude (§2.1), comfortably inside both.

**Latency:** `VISO_DELAY_MS` maxes at 250 ms. Water Linked measured TCP JSON latency at ~4 ms mean,
2 ms σ, 13 ms observed max ([Water Linked integration docs](https://docs.waterlinked.com/dvl/integration/)) — but that is transport only; you must add
ping time-of-flight (the protocol exposes `time_of_validity` vs `time_of_transmission` precisely so
you can measure it, see §2.1) plus your own ROS/MAVROS hop. **The correct thing to do is stamp from
`time_of_validity` and tune `VISO_DELAY_MS` empirically, not to guess.**

**Frames, end to end:**

| Stage | Frame | Source |
|---|---|---|
| Water Linked DVL output | body FRD: x fwd, y starboard, z down | [WL axes doc](https://docs.waterlinked.com/dvl/axes/) |
| gz `DopplerVelocityLog` default | x-fwd, y-**left**, z-**up** (FLU) unless `<reference_frame>` set | `DopplerVelocityLog.hh` header docs, §3.1 |
| ROS convention (REP-103) | ENU / FLU | — |
| MAVROS `vision_speed` output | converts ENU→NED for you | `vision_speed_estimate.cpp` L106-115, §4.1 |
| ArduPilot `VISION_SPEED_ESTIMATE` | **NED earth frame** | §1.1 |
| ArduPilot `VISION_POSITION_DELTA` | **BODY_FRD**, rotated by `VISO_ORIENT` | §1.1 |

**The gating constraint for a GPS-denied sub — this is the single most important finding:**

`NavEKF3_core::setAidingMode()` only leaves `AID_NONE` under these conditions
([`AP_NavEKF3_Control.cpp` L283-299](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp#L283-L299)):

```cpp
if (readyToUseGPS() || readyToUseRangeBeacon() || readyToUseExtNav()) {
    PV_AidingMode = AID_ABSOLUTE;
} else if (readyToUseOptFlow() || readyToUseBodyOdm()) {
    PV_AidingMode = AID_RELATIVE;
}
```

- `readyToUseExtNav()` requires `EK3_SRC1_POSXY == EXTNAV` **and** `extNavDataToFuse`, i.e. an
  external **position** measurement (`AP_NavEKF3_Control.cpp` L611-622). Velocity alone does not
  satisfy it.
- `readyToUseBodyOdm()` requires `EK3_SRC*_VELXY ∈ {EXTNAV(6), WHEEL_ENCODER(7)}`, fresh body-odom
  data within 200 ms with `velErr < 1.0`, plus `tiltAlignComplete && delAngBiasLearned`
  (`AP_NavEKF3_Control.cpp` L561-584).

**Consequence:** a DVL that only publishes `VISION_SPEED_ESTIMATE` will have its data buffered by
`writeExtNavVelData` and consumed at `AP_NavEKF3_PosVelFusion.cpp` L597/L602 **once aiding is
already running**, but it will not by itself bring the filter out of `AID_NONE`. To bootstrap
aiding from a DVL alone you need `VISION_POSITION_DELTA`. That is exactly why
`BlueOS-Water-Linked-DVL` defaults `should_send = MessageType.POSITION_DELTA`
([`dvl.py` L68](https://github.com/bluerobotics/BlueOS-Water-Linked-DVL/blob/master/dvl-a50/dvl.py#L68)).

Also relevant: `AID_RELATIVE` gives relative position only — no EKF origin, no absolute lat/lon.
Body-odom fusion also feeds `AP_NavEKF3_PosVelFusion.cpp` L2050
(`bodyOdomDataToFuse && useVelXYSource(EXTNAV)`).

Feature flags are on for any realistic target: `EK3_FEATURE_BODY_ODOM` and
`EK3_FEATURE_EXTERNAL_NAV` both default to `HAL_PROGRAM_SIZE_LIMIT_KB > 1024`
([`AP_NavEKF3_feature.h`](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_feature.h)).

### 1.4 The SITL wrinkle nobody should skip

SITL simulates a GPS from the FDM ground truth; `SIM_GPS1_ENABLE` defaults to... see
[`SIM_GPS.cpp` L37-42](https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_GPS.cpp#L37-L42) (`ENABLE`, `0:Disable, 1:Enable`) registered as the `GPS1_`
subgroup of `SIM_` ([`SITL.cpp` L730-737](https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SITL.cpp#L730-L737)) → the parameter is **`SIM_GPS1_ENABLE`**.
`sim_vehicle.py` enables it in practice (the sub currently holds position in GUIDED, which requires
`AID_ABSOLUTE`).

**Any honest DVL test in this repo must set `SIM_GPS1_ENABLE 0` in
`banyu_bringup/config/banyu_defaults.parm`.** Without that, you will "successfully integrate a DVL"
and prove nothing, because the GPS is silently doing all the work.

---

## 2. Real hardware — the dominant DVLs

### 2.1 Water Linked DVL A50 / A125 (the BlueROV2-native choice)

Protocol: **JSON over TCP, port 16171**, newline-delimited, protocol version `json_v3.3`
([Water Linked DVL A50/A125 TCP JSON API](https://docs.waterlinked.com/dvl/dvl-json-protocol/)).

Velocity-and-transducer report — exact field list from that page:

| Field | Meaning |
|---|---|
| `time` | Milliseconds since last velocity report (ms) |
| `vx`, `vy`, `vz` | Velocity in x/y/z (m/s) |
| `fom` | Figure of merit — accuracy of the velocities (m/s) |
| `covariance` | 3×3 velocity covariance, entries in (m/s)² |
| `altitude` | Distance to reflecting surface along Z (m) |
| `transducers` | list of `{id, velocity, distance, rssi, nsd, beam_valid}`; `id` is 0-based |
| `velocity_valid` | true ⇒ bottom lock; altitude and velocities are valid |
| `status` | 8-bit mask; bit 0 = high temperature / imminent thermal shutdown |
| `time_of_validity` | Unix µs timestamp of the surface reflection ("center of ping") |
| `time_of_transmission` | Unix µs timestamp immediately before TCP send |
| `format` | `"json_v3.3"` |
| `type` | `"velocity"`, or `"velocity_water"` for water tracking |

Dead-reckoning report (`type: "position_local"`, ~5 Hz): `ts, x, y, z, std, roll, pitch, yaw,
type, status, format`. Commands: `reset_dead_reckoning`, `calibrate_gyro`, `trigger_ping`,
`get_version_info` (all same page).

**Coordinate frame** — right-handed body frame, **x forward, y right (starboard), z down toward the
transducers** ([WL Axes](https://docs.waterlinked.com/dvl/axes/)). That is FRD. A "mounting rotation offset" (yaw θ) maps body frame → vehicle
frame; if mounted back-to-front, set 180°. Frame origin is where the beam axes intersect, 28 mm
above the A50 back plate (A125: 70 mm) — this is the number that belongs in `VISO_POS_*`.

**Update rate** is altitude-dependent, 2–15 Hz (protocol page), governed by range mode
([WL Range mode](https://docs.waterlinked.com/dvl/range-mode/)):

| Range mode | Lower alt (m) | Upper alt (m) | Rate (Hz) |
|---|---|---|---|
| 0 | 0.05 | 0.6 | 15 |
| 1 | 0.3 | 3.0 | 10 |
| 2 | 1.5 | 14 | 5–6 |
| 3 | 7.7 | 36 | 7–8 |
| 4 | 15 | max | 2–4 |
| `wt` (water tracking) | 1.5 | 4.5 | 2 |

**For a pool / RobotX-style course this matters a lot**: at the sub's typical 2 m depth you are in
range mode 0–1, i.e. 10–15 Hz — good — but the A50's minimum altitude is 5 cm and mode 1 needs
≥ 0.3 m clearance. Sitting on the bottom loses lock.

Networking: DHCP client with mDNS at `http://waterlinked-dvl`, fallback static IP
**192.168.194.95** ([WL Network setup](https://docs.waterlinked.com/dvl/networking/)).

Also supported on all models over TCP and serial: **PD4 and PD6** Teledyne-compatible formats, and
a native Water Linked serial protocol ([WL Integration](https://docs.waterlinked.com/dvl/integration/)). Newer models A100 / A250 use RS232 and have
their own protocol pages.

### 2.2 Alternates — Nortek and Teledyne (integration-cost sketch)

- **Nortek DVL1000 / DVL500** — two 8-pin connectors, one serial one Ethernet; the command set and
  communication modes are in the first-party
  [DVL Integrator's Guide (N3015-008)](https://assets.nortekgroup.com/software/N3015-008-Nortek-DVL-Integrators-Guide1018.pdf) and
  [DVL Operations (N3015-006)](https://assets.nortekgroup.com/software/N3015-006-DVLOperations.pdf),
  indexed from [Nortek Support: Integrators Guide – DVL](https://support.nortekgroup.com/hc/en-us/articles/360029839191-Integrators-Guide-DVL).
  Binary Nortek-proprietary framing plus configurable triggering (`TTLRISE`, `SERIAL`, `RS485*`, …).
  **Cost:** you write the parser. No ArduPilot-native support, no BlueOS extension.
- **Teledyne RDI Pathfinder** — speaks the RDI PDx family (PD0/PD4/PD5/PD6/PD13) over serial or
  TCP/UDP. Specs:
  [Pathfinder Operation Manual](https://www.teledynemarine.com/en-us/resources/Documents/Brand%20Support/RD%20INSTRUMENTS/Technical%20Resources/Manuals%20and%20Guides/Pathfinder/PathFinder%20DVL%20Guide_Apr22.pdf),
  [Pathfinder ROV Integration Guide](https://www.teledynemarine.com/en-us/support/SiteAssets/RDI/Manuals%20and%20Guides/Pathfinder/Pathfinder_ROV_Integration_Guide.pdf),
  [FST012 "Decode PDx Binary Format Data"](https://www.teledynemarine.com/en-us/support/SiteAssets/RDI/Technical%20Notes/FST%20Documents/FST012.pdf).
  **Cost:** PD6 is ASCII and trivially parseable; PD0 is a rich binary ensemble (header `7F7Fh`).
  Water Linked units can emit PD4/PD6 too, so a PD6 parser is portable across both vendors.
  I did **not** read the full PDF specs — treat the per-field details as **UNVERIFIED** here.

### 2.3 First-party driver paths

| Path | What it is | Publishes | ROS 2 Humble? |
|---|---|---|---|
| [`waterlinked/waterlinked_dvl`](https://github.com/waterlinked/waterlinked_dvl) | **First-party C++ lib + ROS 2 driver** for A50/A125, MIT, actively pushed (2026-08-07). Donated by the Robotic Decision Making Lab. | `~/velocity_report`, `~/velocity_water_report` (`marine_acoustic_msgs/Dvl`), **`~/odom` (`nav_msgs/Odometry`)**, `~/dead_reckoning` (`PoseWithCovarianceStamped`); services for acoustic enable, dark mode, periodic cycling, `calibrate_gyro`, `reset_dead_reckoning`, `trigger_ping` ([`waterlinked_dvl_driver.cpp` L136-296](https://github.com/waterlinked/waterlinked_dvl/blob/main/waterlinked_dvl_driver/src/waterlinked_dvl_driver.cpp)) | **UNVERIFIED.** Its CI matrix tests `ROS_DISTRO: rolling` only ([`.github/workflows/ci.yml`](https://github.com/waterlinked/waterlinked_dvl/blob/main/.github/workflows/ci.yml)). Its deps (`marine_acoustic_msgs`, `generate_parameter_library`, `nav_msgs`, `tf2_geometry_msgs`, `std_srvs`, `rclcpp_components`) are all released for Humble — `marine_acoustic_msgs` at 2.1.0-1 ([rosdistro humble](https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml)) — so a Humble build is plausible but must be tried. |
| [`bluerobotics/BlueOS-Water-Linked-DVL`](https://github.com/bluerobotics/BlueOS-Water-Linked-DVL) | BlueOS Docker extension. **Not ROS at all** — it talks TCP:16171 to the DVL and posts MAVLink via mavlink2rest directly to the autopilot. | Nothing on ROS. Sends `VISION_POSITION_DELTA` (default), `VISION_SPEED_ESTIMATE`, or `GLOBAL_VISION_POSITION_ESTIMATE`, plus `DISTANCE_SENSOR` for altitude ([`dvl.py` L356-412](https://github.com/bluerobotics/BlueOS-Water-Linked-DVL/blob/master/dvl-a50/dvl.py#L356-L412), [`mavlink2resthelper.py` L313-358](https://github.com/bluerobotics/BlueOS-Water-Linked-DVL/blob/master/dvl-a50/mavlink2resthelper.py#L313-L358)) | N/A |
| [`waterlinked/dvl-a50-ros-driver`](https://github.com/waterlinked/dvl-a50-ros-driver) | **ROS 1 only, archived** (confirmed via GitHub API: `archived: true`, last push 2024-02-26). Water Linked's own docs list it as unmaintained legacy ([WL Software resources](https://docs.waterlinked.com/dvl/software-resources/)). | ROS 1 `DVL.msg` | No |

**One frame caveat worth flagging before anyone copies BlueOS:** in `SPEED_ESTIMATE` mode the
extension sends the raw DVL body-frame `[vx, vy, vz]` in `VISION_SPEED_ESTIMATE`
([`dvl.py` L398-400](https://github.com/bluerobotics/BlueOS-Water-Linked-DVL/blob/master/dvl-a50/dvl.py#L398-L400) → `mavlink2resthelper.py` L328-337, no rotation applied), while
ArduPilot interprets that message as NED (§1.1). Those two only agree at yaw = 0. The
`POSITION_DELTA` default does not have this problem because that message is body-frame by
definition. I am reporting what both sources say; I have **not** tested the runtime behaviour.

---

## 3. Simulation — a DVL in Gazebo Harmonic

### 3.1 The sensor exists and is already installed here

Confirmed present on this machine: `libgz-sensors8-dvl 8.2.2`, `libgz-sensors8-dvl.so.8.2.2`,
`libgz-sim8-dvl-system.so.8.15.0`, headers at
`/usr/include/gz/sensors8/gz/sensors/DopplerVelocityLog.hh`, protos at
`/usr/share/gz/gz-msgs10/protos/gz/msgs/dvl_*.proto`.

**SDF form** — it is a *custom* sensor, not a first-class SDF type. From the header docs
(`DopplerVelocityLog.hh`, the `\verbatim` block) and the shipped example
`/usr/share/gz/gz-sim8/worlds/dvl_world.sdf` L143-192:

```xml
<sensor name="teledyne_pathfinder_dvl" type="custom" gz:type="dvl">
  <pose degrees="true">-0.60 0 -0.16 0 0 180</pose>
  <always_on>1</always_on>
  <update_rate>1</update_rate>
  <topic>/dvl/velocity</topic>
  <gz:dvl>
    <type>phased_array</type>               <!-- or 'piston' -->
    <arrangement degrees="true">
      <beam id="1"><aperture>2</aperture><rotation>45</rotation><tilt>30</tilt></beam>
      <beam><aperture>2</aperture><rotation>135</rotation><tilt>30</tilt></beam>
      <beam><aperture>2</aperture><rotation>-45</rotation><tilt>30</tilt></beam>
      <beam><aperture>2</aperture><rotation>-135</rotation><tilt>30</tilt></beam>
    </arrangement>
    <tracking>
      <bottom_mode>
        <when>best</when>                    <!-- never | always | best -->
        <noise type="gaussian"><stddev>0.002</stddev></noise>
        <visualize>false</visualize>
      </bottom_mode>
      <!-- optional <water_mass_mode> with <water_velocity>, <boundaries><near|far>, <bins> -->
    </tracking>
    <resolution>0.01</resolution>
    <maximum_range>100.</maximum_range>
    <minimum_range>0.1</minimum_range>
    <reference_frame>0 0 0 0 0 -1.570796</reference_frame>  <!-- "ENU to SFM" -->
  </gz:dvl>
</sensor>
```

Every element name above is verbatim from the header's documented schema. Defaults documented
there: `aperture` 90°, `rotation` 0°, `tilt` 0°, `resolution` 1 cm, `minimum_range` 1 cm,
`maximum_range` 100 m, `reference_frame` identity, `<when>` `always`.

**Required system plugin.** The header states plainly: *"Note the
`tethys::DopplerVelocityLogSystem` plugin must be loaded for these custom sensors to be picked up
and setup."* In gz-sim 8 that is:

```xml
<plugin filename="gz-sim-dvl-system" name="gz::sim::systems::DopplerVelocityLogSystem"/>
```

(`/usr/share/gz/gz-sim8/worlds/dvl_world.sdf` L56-59). It requires
`gz-sim-sensors-system` alongside it (same file, L51-54). The sensor derives from
`gz::sensors::RenderingSensor` and is documented as *"built as a custom rendering sensor to
leverage GPU shaders for speed"* — **it needs a rendering context.**

**⚠ This repo's world does not currently load either system.** `grep filename=` on
`banyu_simulation/worlds/proof_of_readiness.world` yields only `gz-sim-physics-system`,
`gz-sim-user-commands-system`, `gz-sim-scene-broadcaster-system`, `libgz-sim-imu-system.so`,
`gz-sim-buoyancy-system`. `gz-sim-sensors-system` is absent. (Separately, this working tree's
`bluerov2_gz/models/bluerov2_heavy/` contains `model.sdf` / `model.sdf.in` and **no `model.xacro`
and no camera block**, contradicting AGENTS.md §Architecture — the pinned `bluerov2_gz` checkout
appears to be behind. Worth reconciling before planning sim work.)

**Message type and topic.** The sensor publishes `gz.msgs.DVLVelocityTracking`
(`DopplerVelocityLog.cc` L674: `node.Advertise<DVLVelocityTracking>(this->Topic())`). If `<topic>`
is empty, the system defaults it to `<scoped_entity_name>/dvl/velocity`
([`gz-sim8 src/systems/dvl/DopplerVelocityLogSystem.cc` L318-321](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/dvl/DopplerVelocityLogSystem.cc#L318-L321)).

**Message schema** (verbatim from `/usr/share/gz/gz-msgs10/protos/gz/msgs/`):

```protobuf
message DVLVelocityTracking {
  Header header = 1;
  enum DVLType { DVL_TYPE_UNSPECIFIED=0; DVL_TYPE_PISTON=1; DVL_TYPE_PHASED_ARRAY=2; }
  DVLType type = 2;
  DVLTrackingTarget target = 3;     // { TargetType type; DVLRangeEstimate range; DVLKinematicEstimate position }
  DVLKinematicEstimate velocity = 4;// { ReferenceType reference; Vector3d mean; repeated double covariance }
  repeated DVLBeamState beams = 5;  // { int32 id; DVLKinematicEstimate velocity; DVLRangeEstimate range;
                                    //   double rssi; double nsd; bool locked }
  int32 status = 6;
}
// DVLRangeEstimate { double mean; double variance }
// DVLKinematicEstimate.ReferenceType { DVL_REFERENCE_UNSPECIFIED=0, _EARTH=1, _SHIP=2 }
// DVLTrackingTarget.TargetType   { DVL_TARGET_UNSPECIFIED=0, _BOTTOM=1, _WATER_MASS=2 }
```

The simulated velocity is always tagged `DVL_REFERENCE_SHIP` — i.e. vehicle/sensor frame, never
earth frame (`DopplerVelocityLog.cc` L1546, L1594). Its default axes are **x-forward, y-left, z-up**
(header docs) unless you supply `<reference_frame>`; the `dvl_world.sdf` example rotates -90° about
z and calls that "ENU to SFM".

### 3.2 `ros_gz_bridge` on Humble **cannot** bridge it

Verified two ways.

**Locally (the installed binary):** dumping the gz type strings out of
`/opt/ros/humble/lib/libros_gz_bridge_lib.so` gives 49 supported `gz.msgs.*` types
(`Actuators … Wrench`). **`DVLVelocityTracking` is not among them**, and `strings | grep -i dvl`
returns nothing.

**Upstream:** [`ros_gz` `humble` branch `ros_gz_bridge/ros_gz_bridge/mappings.py`](https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_bridge/ros_gz_bridge/mappings.py) contains
zero occurrences of "dvl", and `ros_gz_bridge/src/convert/` on that branch has no
`marine_acoustic_msgs.cpp`.

**But it exists one distro up.** [`ros_gz` `jazzy` branch `mappings.py` L54-56](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/ros_gz_bridge/mappings.py#L54-L56):

```python
'marine_acoustic_msgs': [
    Mapping('Dvl', 'DVLVelocityTracking'),
],
```

with the converter at
[`jazzy ros_gz_bridge/src/convert/marine_acoustic_msgs.cpp`](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/src/convert/marine_acoustic_msgs.cpp) (199 lines, bidirectional).

**Implication, stated plainly:** with stock Humble packages you cannot bridge the gz DVL sensor.
Three ways out, in ascending cost:

1. **Backport** the jazzy `marine_acoustic_msgs.cpp` + the `mappings.py` entry into a locally-built
   `ros_gz_bridge`. `marine_acoustic_msgs` **is** released for Humble
   ([rosdistro humble, `marine_msgs` 2.1.0-1](https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml)), so
   `apt install ros-humble-marine-acoustic-msgs` works. Cost: source-build `ros_gz_bridge`, which
   this repo currently gets from apt.
2. **Write a tiny standalone bridge node.** Project DAVE already did exactly this — see
   [`IOES-Lab/dave` `gazebo/dave_ros_gz_plugins/src/DVLBridge.cc`](https://github.com/IOES-Lab/dave/blob/ros2/gazebo/dave_ros_gz_plugins/src/DVLBridge.cc): a `gz::transport::Node::Subscribe`
   on the DVL topic plus a `rclcpp` publisher of `dave_interfaces/msg/DVL`. ~200 lines, no
   `ros_gz_bridge` fork. This is the cheapest honest path.
3. Skip the gz sensor entirely (route (b), §5.2).

For reference, the target ROS message `marine_acoustic_msgs/msg/Dvl` carries
`velocity_mode, dvl_type, geometry_msgs/Vector3 velocity, float64[9] velocity_covar, altitude,
course_gnd, speed_gnd, num_good_beams, sound_speed, beam_ranges_valid, beam_velocities_valid, …`
with the documented frame convention *"z-axis in direction DVL is pointing, x-axis aligned with
vehicle forward"*
([`apl-ocean-engineering/marine_msgs` `marine_acoustic_msgs/msg/Dvl.msg`](https://github.com/apl-ocean-engineering/marine_msgs/blob/ros2/marine_acoustic_msgs/msg/Dvl.msg)).

### 3.3 `ardupilot_gazebo` has no DVL / visual-odometry path

Checked the exact pin in `src/import.repos` (`ArduPilot/ardupilot_gazebo`, branch `ros2`).

- Repo tree has no DVL-related file at all (GitHub trees API, `ros2` branch).
- `src/ArduPilotPlugin.cc` (2043 lines) has zero matches for `dvl`, `vision`, or visual odometry.
- The FDM JSON it sends to SITL contains exactly these keys: `timestamp`, `imu{gyro, accel_body}`,
  `position`, `quaternion`, `velocity`, `rng_1..rng_6`, `windvane{direction, speed}`,
  `no_time_sync`, `no_lockstep`
  ([`ArduPilotPlugin.cc`, JSON writer ~L1923-2020](https://github.com/ArduPilot/ardupilot_gazebo/blob/ros2/src/ArduPilotPlugin.cc)).

**So there is no back-door.** Simulated DVL data has to travel
`gz → ROS 2 → MAVROS → MAVLink → ArduSub`, exactly like real hardware. That is actually a virtue:
whatever you build in sim is the same code path you ship.

(Note in passing: the FDM already hands SITL a perfect `velocity` and `position`. Adding a
simulated DVL does **not** make SITL's estimate better; it only exercises the ingest path. See §5.)

### 3.4 `dave` / `uuv_simulator`

- **`uuvsimulator/uuv_simulator` is ARCHIVED** (GitHub API: `archived: true`, last push
  2023-08-08). Gazebo Classic only. Do not use.
- **Project DAVE moved.** `Field-Robotics-Lab/dave` still exists (default branch `master`, last
  push 2024-08-19); active development is at
  [`IOES-Lab/dave`](https://github.com/IOES-Lab/dave), default branch **`ros2`**, last push
  2026-05-20. That branch contains `dave_interfaces/msg/{DVL,DVLBeam,DVLTarget}.msg`, the
  `DVLBridge` gz↔ROS 2 plugin above, and Nortek DVL500/DVL1000 sensor models whose SDF uses the
  **same `type="custom" gz:type="dvl"` + `<gz:dvl>` schema** as gz-sim 8 (e.g.
  [`models/dave_sensor_models/description/nortek_dvl1000_300/model.sdf` L24-84](https://github.com/IOES-Lab/dave/blob/ros2/models/dave_sensor_models/description/nortek_dvl1000_300/model.sdf), `<update_rate>8</update_rate>`, `/dvl/velocity`).
  So DAVE is *compatible* with the modern gz DVL sensor rather than shipping its own physics.
- **UNVERIFIED:** which exact ROS 2 distro + gz version DAVE's `ros2` branch targets. Their docs
  live at `http://dave-ros2.notion.site` (per their README) and I did not confirm Humble+Harmonic
  support. Treat DAVE as *a reference implementation to copy from*, not as a dependency to add.

---

## 4. MAVROS — the ROS 2 → MAVLink hop

All references are to **mavros 2.14.0**, the version installed here
(`/opt/ros/humble/share/mavros/package.xml`), tag
[`mavlink/mavros@2.14.0`](https://github.com/mavlink/mavros/tree/2.14.0).

### 4.1 `vision_speed` — the natural fit for a velocity-only DVL

Plugin id **`vision_speed`** (not `vision_speed_estimate`), class
`VisionSpeedEstimatePlugin`, registered in
`/opt/ros/humble/share/mavros_extras/mavros_plugins.xml` L245-250.

Subscribed topics ([`vision_speed_estimate.cpp` L46-53](https://github.com/mavlink/mavros/blob/2.14.0/mavros_extras/src/plugins/vision_speed_estimate.cpp#L46-L53)) — all under `/mavros/vision_speed/`:

| ROS topic | Type |
|---|---|
| `/mavros/vision_speed/speed_twist_cov` | `geometry_msgs/TwistWithCovarianceStamped` |
| `/mavros/vision_speed/speed_twist` | `geometry_msgs/TwistStamped` |
| `/mavros/vision_speed/speed_vector` | `geometry_msgs/Vector3Stamped` |

**What it converts:** ENU → NED on the linear velocity vector and its 3×3 covariance
(`convert_vision_speed`, L106-115: `ftf::transform_frame_enu_ned(vel_enu)`). The ENU↔NED transform
is implemented as a reflection — swap x/y, negate z
([`ftf_frame_conversions.cpp` L68-69](https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/lib/ftf_frame_conversions.cpp#L68-L69)).

**What it does NOT convert:** anything body-frame. It does no baselink→aircraft (FLU→FRD) rotation
and no yaw rotation. `header.frame_id` is ignored entirely. **It assumes you hand it an
already-earth-frame ENU velocity.** A raw DVL body-frame velocity fed here is wrong unless the
vehicle heading is zero. You must rotate body→ENU yourself (using the FCU attitude) before
publishing.

`get_subscriptions()` returns empty (`/* Rx disabled */`, L56-59) — it is TX-only.

### 4.2 `odometry` — the only plugin that will do the body→earth rotation, with a catch

Plugin id **`odometry`**, class `OdometryPlugin`
(`mavros_plugins.xml` L154-157). Topics ([`odom.cpp` L82-88](https://github.com/mavlink/mavros/blob/2.14.0/mavros_extras/src/plugins/odom.cpp#L82-L88)):

| ROS topic | Type | Direction |
|---|---|---|
| `/mavros/odometry/out` | `nav_msgs/Odometry` | ROS → FCU (`ODOMETRY` msg) |
| `/mavros/odometry/in` | `nav_msgs/Odometry` | FCU → ROS |

It always emits `frame_id = MAV_FRAME::LOCAL_FRD`, `child_frame_id = MAV_FRAME::BODY_FRD`,
`estimator_type = MAV_ESTIMATOR_TYPE::VISION` (`odom.cpp` L276-278) — which is exactly what
ArduPilot's `handle_odometry` demands (§1.1).

**The catch — and it is a big one.** `odom_cb` does *not* use the hardcoded ENU→NED helpers. It
looks up **TF transforms** (`odom.cpp` L250-255):

```cpp
lookup_static_transform(odom->header.frame_id + "_ned", odom->header.frame_id, tf_parent2parent_des);
lookup_static_transform(odom->child_frame_id + "_frd", odom->child_frame_id, tf_child2child_des);
```

So publishing `nav_msgs/Odometry` with `frame_id: map`, `child_frame_id: base_link` requires
**`map_ned` and `base_link_frd` frames to exist in TF**. Grepping the whole mavros 2.14.0 source
tree for `base_link_frd` / `map_ned` / `odom_ned` finds them only in `odom.cpp` comments and
`gimbal_control.cpp` — **MAVROS does not publish these static transforms.** You must publish them
yourself (e.g. two `tf2_ros static_transform_publisher` nodes in
`proof_of_readiness.launch.py`).

Worse: `lookup_static_transform` catches the `tf2::TransformException`, logs an error, and
`return`s — leaving the caller's `Eigen::Affine3d` **uninitialised** (`odom.cpp` L115-128).
`odom_cb` then proceeds to use it. Missing TF frames therefore produce garbage odometry, not a
clean failure. Verify the frames exist before trusting any output.

### 4.3 `vision_pose` — pose only

Plugin id **`vision_pose`**, class `VisionPoseEstimatePlugin` (`mavros_plugins.xml` L238-243).
Topics `/mavros/vision_pose/pose` (`PoseStamped`) and `/mavros/vision_pose/pose_cov`
(`PoseWithCovarianceStamped`), or TF-listen mode via the `tf/listen` parameter
([`vision_pose_estimate.cpp` L59-90](https://github.com/mavlink/mavros/blob/2.14.0/mavros_extras/src/plugins/vision_pose_estimate.cpp#L59-L90)). It applies `ftf::transform_frame_enu_ned` to
position and covariance (L127-135) and sends **`VISION_POSITION_ESTIMATE`** (L135) — *not*
`GLOBAL_VISION_POSITION_ESTIMATE`. Both land in the same `handle_common_vision_position_estimate_data`
in ArduPilot, so functionally equivalent.

Relevant only if you want to push the DVL's **dead-reckoned** pose (`type: position_local`) at
ArduSub. Note that dead-reckoned position drifts unboundedly, which is a poor thing to feed
`EK3_SRC1_POSXY=6`.

### 4.4 Allowlist / denylist implications for `mavros_config.yaml`

`banyu_bringup/config/mavros_config.yaml` sets:

```yaml
plugin_denylist:
  - actuator_control
  - ftp
  - hil
  - altitude
  - debug_value
  - image_pub
  - px4flow
  - vibration
  - vision_speed_estimate   # <-- see below
  - wheel_odometry
```

(mirrored from `/opt/ros/humble/share/mavros/launch/apm_pluginlists.yaml`).

MAVROS iterates `plugin_factory_loader.getDeclaredClasses()` and calls `add_plugin(name)` for each
([`mavros_uas.cpp` L155-156](https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/lib/mavros_uas.cpp#L155-L156)); `name` is the pluginlib **class name**, i.e. the `name=`
attribute in `mavros_plugins.xml`. Matching is `fnmatch(pattern, pl_name, FNM_CASEFOLD)`
(`mavros_uas.cpp` L194-202), and the denylist is evaluated in `is_plugin_allowed` (L216-232).

Therefore:

- **`vision_speed_estimate` does not match the registered plugin name `vision_speed`.** The entry
  is a **no-op** and the `vision_speed` plugin is loaded. This is presumably a MAVROS 1 → MAVROS 2
  plugin rename that upstream's `apm_pluginlists.yaml` never caught up with. *Verified by source
  inspection, not by runtime test* — confirm with
  `ros2 topic list | grep vision_speed` against a running stack. If you want it genuinely disabled,
  the entry must read `vision_speed`.
- **`odometry` is not denied** → `/mavros/odometry/out` and `/in` already exist today.
- `wheel_odometry` **does** match and is genuinely disabled (irrelevant here).
- If you go with route (a) or (b) below, either delete the `vision_speed_estimate` line (it is
  misleading) or replace it with the correct name and then remove it when you actually want the
  plugin. Right now the config *says* one thing and *does* another — worth fixing regardless of
  whether a DVL ever ships.

---

## 5. Candidate integration routes

### 5.1 (a) gz `DopplerVelocityLog` → custom bridge → MAVROS → ArduSub EKF3

**Shape:** add `gz-sim-sensors-system` + `gz-sim-dvl-system` to
`banyu_simulation/worlds/proof_of_readiness.world`; add a `<sensor type="custom" gz:type="dvl">`
block to the bluerov2_heavy model; write (or backport) a bridge from
`gz.msgs.DVLVelocityTracking`; add a node that converts sensor-frame velocity → the message ArduSub
needs; set `VISO_TYPE=1`, `EK3_SRC1_VELXY=6` in `banyu_defaults.parm`; set `SIM_GPS1_ENABLE=0`.

**Cost:** highest. Bridge (~200 LOC, DAVE has a template) + a frame/rotation node + world and model
edits + parameter work + a rendering-capable sim (the DVL is a `RenderingSensor`, and the current
world does not even load `gz-sim-sensors-system`).

**What breaks / collides:**
- The bluerov2_gz pin in this working tree has no `model.xacro` and no camera block, contradicting
  AGENTS.md. Sorting that out is a prerequisite, not a side quest.
- AGENTS.md forbids touching the `front_camera` sensor block; adding a *new* sibling sensor does not
  violate that, but the model file situation must be resolved first.
- Adding a rendering sensor costs sim frame rate. AGENTS.md already documents SITL/gz timing
  fragility ("`ArduPilot controller has reset`", stale `ardusub`/`mavproxy` processes). Extra GPU
  load is a real regression risk to a stack that is already timing-sensitive.

**What it buys:** an honest end-to-end rehearsal of the hardware data path, including beam dropouts
when the bottom is out of range — which is the failure mode that will actually bite in a pool.

### 5.2 (b) Gazebo ground-truth odometry → MAVROS `odometry/out` → ArduSub (fake DVL)

**Shape:** bridge `gz.msgs.Odometry` → `nav_msgs/Odometry` (already supported on Humble:
`mappings.py` has `Mapping('Odometry','Odometry')` and `Mapping('Odometry','OdometryWithCovariance')`),
add noise if you want realism, publish `map_ned` / `base_link_frd` static TFs, publish to
`/mavros/odometry/out`, set `SIM_GPS1_ENABLE=0` + `VISO_TYPE=1` + `EK3_SRC1_{POSXY,VELXY}=6`.

**Cost:** lowest by a wide margin. No custom bridge, no gz sensor, no rendering cost. Mostly launch
+ parameter work.

**What breaks / collides:**
- The `odometry` plugin's silent-garbage-on-missing-TF behaviour (§4.2). Must publish the two
  static transforms and assert they exist.
- It is a *lie about the sensor*: no beam geometry, no altitude limit, no loss-of-bottom-lock, no
  2–15 Hz altitude-dependent rate. It will make the mission look more robust than it is.
- Feeding position through `ODOMETRY` gives `AID_ABSOLUTE` immediately, which papers over the
  `AID_NONE`/`AID_RELATIVE` distinction of §1.3 — the exact thing a real DVL forces you to confront.

**Verdict:** good as a *scaffold* to prove the MAVROS→ArduSub half of the pipe works, bad as an
end state. Cheap enough to do first regardless of which route wins.

### 5.3 (c) Real Water Linked A50 → BlueOS extension → MAVLink direct; ROS only observes

**Shape:** install the DVL, plug it into the BlueROV2's onboard network, install
`BlueOS-Water-Linked-DVL` from the BlueOS Extension Manager, pick `POSITION_DELTA` mode (the
default), let it set `VISO_TYPE=1` / `EK3_SRC1_*` itself. ROS 2 keeps consuming
`/mavros/local_position/pose` unchanged.

**Cost:** lowest on hardware. Zero ROS code. It is BlueRobotics' supported path.

**What breaks / collides:**
- **Nothing in `banyu_mission` changes at all.** `ctx_->pose` is still
  `/mavros/local_position/pose`, still ENU, still `frame_id: map`. Every P-controller keeps working.
  That is the whole appeal.
- It bypasses ROS entirely, so you get no DVL topic to log, no `banyu_msgs`-side visibility, and no
  way to reproduce a hardware failure in sim.
- Requires BlueOS on the companion computer. **UNVERIFIED** whether this repo's target vehicle runs
  BlueOS or a plain ROS 2 companion.
- The frame caveat in §2.3 if anyone flips it to `SPEED_ESTIMATE`.

**Variant (c′):** run [`waterlinked/waterlinked_dvl`](https://github.com/waterlinked/waterlinked_dvl) on ROS 2 instead, take its `~/odom`
(`nav_msgs/Odometry`, twist populated from `vx/vy/vz` with the 3×3 covariance —
[`waterlinked_dvl_driver.cpp` L187-201](https://github.com/waterlinked/waterlinked_dvl/blob/main/waterlinked_dvl_driver/src/waterlinked_dvl_driver.cpp#L187-L201)), remap to `/mavros/odometry/out`. This gets you ROS
visibility *and* the correct body→earth rotation (done by ArduPilot's `handle_odometry`, §1.1) for
essentially the cost of a launch file — **provided the driver builds on Humble**, which is
untested (§2.3). This is the most attractive hardware option if the build works.

### 5.4 (d) DVL used only by ROS-side mission logic, never fused into ArduSub

**Shape:** run a DVL driver in ROS 2, integrate velocity in `banyu_mission` (or a new node) to
maintain a dead-reckoned pose in `MissionContext`, and drive `NavToGate` / `ReturnToStart` off that
instead of `/mavros/local_position/pose`.

**Cost:** medium. No autopilot parameter risk, no MAVLink work — but you now own an estimator.

**What breaks / collides:**
- You throw away ArduSub's EKF, which is the only thing fusing the **barometer** (depth) and the
  **compass** (heading). AGENTS.md's core empirical finding is that *velocity-only setpoints hold
  depth precisely* — that is EKF3 + baro doing the work. Dead-reckoning in ROS does not replace it.
- Yaw drift is unbounded without a compass fusion, and every visual-servoing node
  (`GateAlign`, `MarkerAlign`, `MarkerOrbit`, `TurnAround`'s yaw-capture) already depends on
  heading-consistent behaviour.
- ArduSub's own GUIDED mode still needs *some* position estimate to accept
  `setpoint_raw/local`; running the autopilot blind while ROS thinks it knows where it is, is the
  worst of both worlds.

**Verdict:** only defensible as a *diagnostic overlay* (log DVL-derived odometry alongside the EKF's
to quantify drift), not as the navigation source.

### 5.5 The collision with "navigation is velocity-loop, not position-setpoint"

Worth stating precisely, because it is easy to get wrong:

AGENTS.md's finding is that ArduSub GUIDED **position** targets (`type_mask 0xDF8`) route through
`AC_WPNav`, which crawls and lets depth drift in this SITL setup; `NavToGate` / `ReturnToStart`
therefore run their own P-controller on pose error and stream **velocity-only** setpoints
(`0xDC7`). Confirmed in the code: `bt_nodes.cpp` L130 (`velocitySetpoint`), L467/L537/L541
(`bodyVelocityYawRateSetpoint`), L737, and `mission_context.hpp` L94-127.

**A DVL does not touch the actuation side.** It changes the *feedback* term. Concretely:

- The P-controllers read `ctx_->pose`, fed from `/mavros/local_position/pose`
  (`banyu_mission.cpp` L105). That topic's content comes from EKF3.
- In sim, EKF3 has a synthetic GPS, so `ctx_->pose` is essentially ground truth and the P-controller
  converges. **On hardware with no GPS and no DVL, EKF3 sits in `AID_NONE` (§1.3), so
  `local_position/pose` is at best dead-reckoned from IMU and unusable for `NavToGate`.**
- Adding a DVL correctly ⇒ EKF3 reaches `AID_RELATIVE` (body-odom) or `AID_ABSOLUTE` (extnav pose)
  ⇒ `ctx_->pose` becomes meaningful ⇒ the *existing* velocity-loop code works unchanged.

**So a DVL is what makes the existing velocity-loop design viable on hardware. It is not a reason
to revisit `0xDF8`.** If anything it argues *harder* for the velocity loop: with `AID_RELATIVE`
there is no EKF origin at all, so absolute position targets are meaningless, whereas streaming
velocity setpoints is perfectly well-defined. Do not let "we have a DVL now, let's try position
setpoints again" happen — the `AC_WPNav` problem is orthogonal and unchanged.

Two second-order effects to watch:

1. `FS_EKF_ACTION 0` in `banyu_defaults.parm` ("never failsafe") currently masks EKF health.
   Once a DVL is the position source, EKF variance spikes on bottom-lock loss become the thing you
   most want to see. Keep the failsafe *action* off if you like, but the mission layer should start
   watching `/mavros/state` / EKF status flags.
2. Every setpoint must still carry an explicit zero yaw rate (AGENTS.md). A DVL changes nothing
   there.

### 5.6 Recommendation

Sequence, cheapest-first, each step independently useful:

1. **Fix the `mavros_config.yaml` denylist name** (`vision_speed_estimate` → `vision_speed`, or
   delete). One line, removes a lie from the config. (§4.4)
2. **Set `SIM_GPS1_ENABLE 0` behind a launch argument.** Until you do, no DVL claim about this sim
   is testable. (§1.4)
3. **Route (b) as a scaffold** — ground-truth `nav_msgs/Odometry` → `/mavros/odometry/out`, with
   the two static TF frames. Proves the MAVROS→ArduSub half. (§5.2)
4. **Route (c′) for hardware** — try `waterlinked/waterlinked_dvl` on Humble; if it builds, remap
   `~/odom` → `/mavros/odometry/out` and you are done, with ROS-side visibility. Fall back to
   route (c) BlueOS extension if it does not. (§5.3)
5. **Route (a) only if sim fidelity of bottom-lock loss becomes a real mission risk.** (§5.1)

---

## 6. Open questions for the repo owner

Crisp enough to become decision tickets.

1. **Sim-only, hardware-only, or both?** These are different projects (§Bottom line #1). Which one
   is actually being funded this cycle?
2. **Which DVL model, and is it already purchased?** A50 vs A125 vs A100/A250 changes the protocol
   page, the serial electrical interface, and the minimum altitude. Nortek/Teledyne changes
   everything (§2.2).
3. **Does the target vehicle run BlueOS, or a plain ROS 2 companion?** This single answer decides
   between route (c) (zero code) and route (c′) (a launch file) — see §5.3.
4. **Should the DVL be fused into ArduSub's EKF3, or consumed only by `banyu_mission`?** If EKF3:
   accept that you are changing `EK3_SRC1_*` and therefore the behaviour of *every* existing mission
   node that reads `local_position/pose`. If mission-only: accept losing baro/compass fusion (§5.4).
5. **`VISION_POSITION_DELTA` or `ODOMETRY`?** Delta is the only one that bootstraps aiding from
   scratch (§1.3) and is what BlueRobotics ships; `ODOMETRY` is the only one MAVROS produces with a
   correct body→earth rotation (§4.2). Picking Delta means writing the delta computation yourself
   (BlueOS does it in ~15 lines, `dvl.py` L386-397). Picking `ODOMETRY` means also feeding a
   position, which for a DVL means dead-reckoning, which drifts.
6. **Where does depth come from?** BlueRobotics keeps `EK3_SRC1_POSZ = 1` (Baro). Is there any
   reason to deviate? (Default answer: no.)
7. **Is `SIM_GPS1_ENABLE 0` acceptable as the default for the proof-of-readiness sim?** It will make
   the sim strictly harder and may expose latent issues in `NavToGate` / `ReturnToStart`. Behind a
   launch arg, or on by default?
8. **Who owns the bluerov2_gz pin?** This working tree's `bluerov2_gz` lacks the `model.xacro` and
   camera block AGENTS.md describes (§3.1). Any model-level sim work (adding a DVL sensor) is
   blocked on reconciling that.
9. **Are we willing to source-build `ros_gz_bridge`?** That is the difference between backporting
   the jazzy DVL mapping and writing a standalone bridge node (§3.2).
10. **What is the acceptance test?** Suggested, and it should be written before any code: with
    `SIM_GPS1_ENABLE 0`, the sub completes `NavToGate → GateAlign → GatePass` with position error
    bounded, and the EKF reports a non-`AID_NONE` aiding mode for the whole run.

---

## Appendix — source inventory

**ArduPilot** (`github.com/ArduPilot/ardupilot`, `master`, fetched 2026-08-15)
`ArduSub/Parameters.cpp`, `ArduSub/GCS_MAVLink_Sub.cpp`,
`libraries/GCS_MAVLink/GCS_Common.cpp`,
`libraries/AP_VisualOdom/{AP_VisualOdom.cpp,AP_VisualOdom.h,AP_VisualOdom_Backend.cpp,AP_VisualOdom_MAV.cpp,AP_VisualOdom_config.h}`,
`libraries/AP_Vehicle/AP_Vehicle.cpp`,
`libraries/AP_AHRS/AP_AHRS.cpp`, `libraries/AP_DAL/AP_DAL.h`,
`libraries/AP_NavEKF/AP_NavEKF_Source.cpp`,
`libraries/AP_NavEKF3/{AP_NavEKF3.cpp,AP_NavEKF3.h,AP_NavEKF3_Control.cpp,AP_NavEKF3_Measurements.cpp,AP_NavEKF3_PosVelFusion.cpp,AP_NavEKF3_feature.h}`,
`libraries/SITL/{SITL.cpp,SIM_GPS.cpp}`

**MAVLink** `mavlink/mavlink` `common.xml` (VISION_SPEED_ESTIMATE id 103);
`ArduPilot/mavlink` `ardupilotmega.xml` (VISION_POSITION_DELTA id 11011)

**MAVROS** `mavlink/mavros` tag `2.14.0`:
`mavros_extras/src/plugins/{vision_speed_estimate,vision_pose_estimate,odom}.cpp`,
`mavros/src/lib/{mavros_uas,ftf_frame_conversions}.cpp`,
`mavros/include/mavros/frame_tf.hpp`; local
`/opt/ros/humble/share/mavros_extras/mavros_plugins.xml`,
`/opt/ros/humble/share/mavros/launch/apm_pluginlists.yaml`

**Gazebo** local install: `/usr/include/gz/sensors8/gz/sensors/DopplerVelocityLog.hh`,
`/usr/share/gz/gz-msgs10/protos/gz/msgs/dvl_*.proto`,
`/usr/share/gz/gz-sim8/worlds/dvl_world.sdf`,
`/usr/lib/x86_64-linux-gnu/{libgz-sensors8-dvl.so.8.2.2,libgz-sim8-dvl-system.so.8.15.0}`;
upstream `gazebosim/gz-sensors@gz-sensors8 src/DopplerVelocityLog.cc`,
`gazebosim/gz-sim@gz-sim8 src/systems/dvl/DopplerVelocityLogSystem.cc`

**ros_gz** `gazebosim/ros_gz` branches `humble` and `jazzy`:
`ros_gz_bridge/ros_gz_bridge/mappings.py`, `ros_gz_bridge/src/convert/marine_acoustic_msgs.cpp`;
local `/opt/ros/humble/lib/libros_gz_bridge_lib.so`

**Water Linked** <https://docs.waterlinked.com/dvl/> — `dvl-json-protocol`, `axes`, `range-mode`,
`networking`, `integration`, `software-resources`, `dvl-a50`

**Drivers** `bluerobotics/BlueOS-Water-Linked-DVL` (`dvl-a50/dvl.py`,
`dvl-a50/mavlink2resthelper.py`, `README.md`); `waterlinked/waterlinked_dvl`;
`waterlinked/dvl-a50-ros-driver` (archived); `apl-ocean-engineering/marine_msgs`;
`ros/rosdistro` `humble/distribution.yaml`

**Simulators** `ArduPilot/ardupilot_gazebo@ros2 src/ArduPilotPlugin.cc`;
`IOES-Lab/dave@ros2`; `uuvsimulator/uuv_simulator` (archived)

**Vendor docs (alternates)** Nortek N3015-008 / N3015-006; Teledyne Pathfinder Operation Manual,
ROV Integration Guide, FST012
