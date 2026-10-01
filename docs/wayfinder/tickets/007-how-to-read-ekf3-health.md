# 007 — How do we read EKF3 velocity-source health from this stack?

`wayfinder:grilling` · parent: [Map: DVL in sim, fused by ArduSub EKF3](../map-dvl-sim-ekf3.md)
· status: **CLOSED** · resolved AFK
· blocked by: ~~001~~ (closed) · blocks: 008

## Question

The destination's proof is autopilot-side: EKF3 confirms it is fusing the DVL.
Pin down the concrete mechanism for observing that, before 008 tries to.

Settle:

- **Which readout.** `EKF_STATUS_REPORT` velocity variance, the visual-odometry
  message accept/reject counters, ArduSub's onboard log, or the MAVROS topic
  that surfaces EKF status — whichever actually exposes source selection rather
  than merely "the EKF is happy".
- **How to reach it from here.** MAVROS topic, MAVLink inspector, QGC on port
  14550 (AGENTS.md notes it works fine as a second GCS), or pulling the SITL
  log.
- **What distinguishes fusing from ignoring.** A healthy EKF that silently never
  accepted a single DVL sample must be distinguishable from one that is fusing.
  This is the crux — name the specific signal that separates them.

Numeric thresholds for "healthy" are deliberately not settled here; they need
data from a first run and are parked in the map's Not-yet-specified.

Resolution records the exact command/topic and the signal to look for.

## Resolution

**Two readouts, because neither alone separates the failure modes.**

### 1. Live, ROS-side — `/mavros/estimator_status`

`mavros_msgs/EstimatorStatus`
(`/opt/ros/humble/share/mavros_msgs/msg/EstimatorStatus.msg`), carrying
ArduPilot's `EKF_STATUS_REPORT` flags. The `sys_status` plugin is **not** in
`mavros_config.yaml`'s denylist, so it is already available — nothing to enable.

```bash
bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && \
  ros2 topic echo /mavros/estimator_status'
```

The four flags that matter:

| Flag | Meaning |
|---|---|
| `const_pos_mode_status_flag` | **true = EKF has no aiding at all.** The clearest negative. |
| `velocity_horiz_status_flag` | horizontal velocity estimate healthy |
| `pos_horiz_rel_status_flag` | *relative* horizontal position — what `AID_RELATIVE` gives |
| `pos_horiz_abs_status_flag` | *absolute* position — GPS |

With synthetic GPS off, a DVL that is genuinely fusing reads
`const_pos_mode=false`, `velocity_horiz=true`, `pos_horiz_rel=true`,
`pos_horiz_abs=false`. That combination is reachable *only* through
`AID_RELATIVE`, which is exactly the body-odom path.

### 2. Autopilot-side — the `VISO` dataflash message

`AP_VisualOdom_Backend::Write_VisualOdom()` is called at
`AP_VisualOdom_Backend.cpp:73`, immediately beside the `writeBodyFrameOdom()`
call at `:62`. So **every `VISION_POSITION_DELTA` that arrives and parses gets
logged**, whether or not the EKF then uses it.

Log message `VISO`, fields `TimeUS,dt,AngDX,AngDY,AngDZ,PosDX,PosDY,PosDZ,conf`
(`AP_VisualOdom/LogStructure.h:97-98`).

### The discriminator — this is the crux the ticket asked for

| `VISO` in log | `const_pos_mode` | Verdict |
|---|---|---|
| present | false | **Fusing.** The destination. |
| present | true | Arriving but rejected — check `velErr` and `EK3_SRC1_VELXY 6` |
| absent | — | Never arrived. Transport problem, i.e. [005](./005-carry-dvl-velocity-to-mavlink.md) |

A healthy EKF that silently never accepted a DVL sample is exactly the middle
row, and the flags alone cannot distinguish it from the top row — which is why
the log is not optional here.

### Deliberately unresolved

Numeric thresholds for "healthy" stay in the map's **Not yet specified**. The
flags are boolean and sufficient for a pass/fail; variance thresholds need
010's baseline before they mean anything.
