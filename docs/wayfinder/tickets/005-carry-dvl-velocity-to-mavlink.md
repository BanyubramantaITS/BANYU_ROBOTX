# 005 — Carry DVL velocity from gz to MAVLink

`wayfinder:task` · parent: [Map: DVL in sim, fused by ArduSub EKF3](../map-dvl-sim-ekf3.md)
· status: **CLOSED** · resolved by build + experiment
· blocked by: ~~002~~, ~~003~~ (both closed) · blocks: 008

## Question

Build the route [Choose the transport route from gz sensor to ArduSub](./002-choose-transport-route.md)
selected: a ROS node that reads the gz DVL sensor and speaks
`VISION_POSITION_DELTA` directly to SITL on its own MAVLink endpoint. MAVROS is
not in this path.

Two pieces:

**1. The gz → ROS bridge.** `ros_gz_bridge` on Humble cannot carry
`gz.msgs.DVLVelocityTracking` — the mapping exists only from `jazzy`
(`Mapping('Dvl','DVLVelocityTracking')`). Either backport that mapping (~1 file)
or lift DAVE's standalone `DVLBridge.cc` (~200 lines). Pick on which is less to
maintain against a pinned Humble.

**2. The MAVLink sender.** Emits `VISION_POSITION_DELTA` at the DVL's ping rate.

### The parts that will actually go wrong

- **`velErr` is load-bearing.** `readyToUseBodyOdm()` requires
  `bodyOdmDataNew.velErr < 1.0f` and data fresher than 200 ms
  (`AP_NavEKF3_Control.cpp:559-573`). Populate it honestly from the sensor's
  figure-of-merit. A hardcoded zero will appear to work and is a lie the EKF
  will believe.
- **The message is a *delta*, not a velocity.** `VISION_POSITION_DELTA` carries
  a position delta and an **angle delta** over a stated time interval. The gz
  sensor gives velocity, so the node integrates per-ping. Decide what supplies
  the angle-delta term and whether ArduSub tolerates it being zero — check
  `AP_VisualOdom_Backend.cpp` for how the field is consumed before assuming.
- **Frames and signs.** The DVL reports body-frame. Nothing in this path goes
  through MAVROS, so **none** of MAVROS's ENU↔NED conversion applies — this node
  owns the entire conversion. Verify each axis empirically by driving the sub one
  direction at a time; do not reason it out from conventions.
- **The endpoint.** MAVROS holds `udp://127.0.0.1:14551`, SITL exports `14550`
  for QGC. This node needs a third output, added via `sim_vehicle.py --out`.
  Confirm ArduSub accepts vision messages from a non-primary GCS link.
- Consider publishing a ROS topic alongside, purely for observability — the
  chosen route otherwise makes the DVL invisible to `ros2 topic echo`.

Done when the message is confirmed arriving at ArduSub — arrival, not fusion.
Fusion is 006 and 008.

## Resolution

Built as **one Python process**, `banyu_bringup/scripts/dvl_mavlink_bridge.py`,
wired into `proof_of_readiness.launch.py`.

**The gz→ROS bridge turned out to be unnecessary.** `gz-transport13` and
`gz-msgs10` both ship working Python bindings on this box, so the script
subscribes to the gz topic *directly*. No `ros_gz_bridge` backport, no lifted
DAVE `DVLBridge.cc`, no new apt dependency, no C++ package. It is not even a ROS
node — it needs no `rclpy` — just an `ExecuteProcess`-style entry in the launch.

Chain: gz `/dvl/velocity` → integrate per ping → `VISION_POSITION_DELTA` over
UDP to SITL on a third endpoint (`--out=udp:127.0.0.1:14552`, added to the
`sim_vehicle.py` command; 14550 is QGC, 14551 is MAVROS).

### Frame mapping — measured, not reasoned

Driving an unrotated probe along each world axis with `VelocityControl` and
reading the raw sensor gave: world +X → `dvl.y`, world +Y → `−dvl.x`,
world +Z → `dvl.z`. That is a +90° yaw between sensor and body frames, which is
the `<reference_frame>0 0 0 0 0 -1.570796</reference_frame>` in the DVL model.
Composing with Gazebo-z-up → ArduPilot-FRD gives:

```
vx_frd =  dvl.y      vy_frd =  dvl.x      vz_frd = -dvl.z
```

Verified end to end through the bridge against a stub autopilot:

| commanded | emitted FRD |
|---|---|
| world +X 0.5 | (+0.500, −0.000, +0.010) |
| world +Y 0.5 | (−0.000, **−0.500**, +0.010) |
| world +Z 0.5 (up) | (−0.000, +0.000, **−0.490**) |

Signs are right in all three axes, `dt` is a clean 0.100 s at the 10 Hz sensor
rate, and `position_delta = v × dt` as `AP_NavEKF3_Measurements.cpp:131` expects.

### Four things that each silently broke it

1. **`VISION_POSITION_DELTA` is not in the `common` dialect.** pymavlink needs
   `MAVLINK_DIALECT=ardupilotmega`.
2. **…and not in ardupilotmega v1.0 either.** Without `MAVLINK20=1` pymavlink
   resolves to `dialects.v10.ardupilotmega`, where the send function still does
   not exist. Both env vars must be set **before** importing pymavlink — the
   dialect binds at import, so `mavutil.set_dialect()` afterwards is too late.
3. **The gz target-type enum hangs off `DVLTrackingTarget`**, not off the parent
   `DVLVelocityTracking` message.
4. `VISO_ORIENT` had to be corrected **25 → 0** — see below.

### Correction to 006

`handle_vision_position_delta_msg()` rotates **both** `angle_delta` and
`position_delta` by `VISO_ORIENT` (`AP_VisualOdom_Backend.cpp:48-52`). Since this
bridge already emits vehicle-frame FRD, `VISO_ORIENT 25` (Down) would have
rotated correct data into nonsense. Changed to **0** in `banyu_defaults.parm`.
`VISO_ORIENT` describes an *un-converted* sensor's mounting; ours is
pre-converted.

### Correction to 002

002 called `velErr` "load-bearing — a hardcoded zero is a lie the EKF will
believe". Half right. `velErr = VERR_MIN + (VERR_MAX−VERR_MIN)×(1−quality/100)`
(`AP_NavEKF3_Measurements.cpp:138`) with defaults `EK3_VIS_VERR_MIN 0.1` /
`MAX 0.9`, so velErr always lands in 0.1–0.9 — **always under the `< 1.0` gate**.
Confidence therefore cannot *block* fusion with default params; it only scales
how far the EKF trusts the data. The bridge still derives it honestly from the
message covariance.

### Known limitations, deliberately shipped

- **`angle_delta` is always zero.** The bridge has no attitude source, so the
  EKF assumes no body rotation over each interval. That costs a lever-arm error
  while yawing (`VISO_POS_Z` is 0.10 m) — most likely to show up in
  `MarkerOrbit`. Revisit if orbit-phase drift appears.
- **Vertical velocity reads ~2% low** (0.490 against a commanded 0.500). Present
  in the raw sensor too, so it is a sensor-model artifact, not a bridge bug.
- **No ROS topic is published**, so the DVL is invisible to `ros2 topic echo`.
  Skipped as YAGNI; the gz topic is directly observable with `gz topic -e`.
- Water-track velocity is **dropped**, not forwarded — it is relative to a moving
  medium and would poison a dead-reckoned delta. A dropout also resets the delta
  accumulator so no interval ever spans a gap.
