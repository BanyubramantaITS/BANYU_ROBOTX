# 010 — Turn off SITL's synthetic GPS so the DVL has a job

`wayfinder:task` · parent: [Map: DVL in sim, fused by ArduSub EKF3](../map-dvl-sim-ekf3.md)
· status: **CLOSED** · resolved AFK
· blocked by: ~~006~~ (closed) · blocks: 008

## Question

SITL synthesises a GPS from Gazebo ground truth (`SIM_GPS1_ENABLE`,
`libraries/SITL/SITL.cpp` L730-737). So ArduSub already has a perfect position
estimate, EKF3 is happily aided without any DVL, and **a DVL added on top would
show green while proving nothing**. This ticket is what makes the destination
honest rather than theatre.

Do:

- Disable synthetic GPS in the SITL run (via `banyu_defaults.parm`, the same
  curated file 006 edits) and record what breaks.
- **Establish the baseline first, before the DVL is in the loop.** With GPS off
  and no DVL, EKF3 should fail to aid — capture exactly how that failure
  presents, using 007's mechanism. Without that negative control there is no way
  to tell a fusing DVL from a placebo.
- Note what else in the stack depends on having an absolute position. Every
  `NavToGate`/`ReturnToStart` P-controller reads `/mavros/local_position/pose`;
  with `AID_RELATIVE` there is no EKF origin, so expect the existing mission to
  degrade or refuse. **That degradation is an expected finding, not a
  regression to fix here** — nav rework is out of scope for this map.
- Expect arming to be affected. Record it rather than disabling the check
  reflexively.

Resolution records the parameter change, the negative-control observation, and
which mission phases stop working without absolute position.

## Resolution

Synthetic GPS is off, the negative control exists, and it is unambiguous.

### The parameter change

Appended to `banyu_bringup/config/banyu_defaults.parm`:

```
SIM_GPS1_ENABLE 0     # no ground-truth GPS; the DVL now has a job
EK3_SRC1_POSXY  0     # NONE. A DVL gives velocity, not absolute position;
                      # leaving this at GPS(3) makes EKF3 wait for a GPS that
                      # no longer exists.
EK3_SRC1_POSZ   1     # BARO. Depth still comes from a sensor a sub really has.
```

`EK3_SRC1_VELXY 6` (EXTNAV) was already in place from
[006](./006-ardusub-params-for-ekf3-fusion.md).

### The negative control

Same params, DVL bridge not running (`use_dvl:=False`, the launch argument this
ticket added for exactly this purpose). `EKF_STATUS_REPORT` requested explicitly
via `MAV_CMD_SET_MESSAGE_INTERVAL` — MAVROS does not stream msg 193 at default
rates:

```
EKF flags=0xa7 [ATTITUDE, VELOCITY_HORIZ, VELOCITY_VERT, POS_VERT_ABS, CONST_POS_MODE]
vel_var=0.000  pos_horiz_var=0.008  TOTAL EKF_STATUS_REPORT=165
```

`CONST_POS_MODE` **set**, `POS_HORIZ_REL` **clear**, `POS_HORIZ_ABS` **clear**.
That is `AID_NONE`: EKF3 has pinned the vehicle at a constant position because
it has nothing to aid with. No `started relative aiding` message ever appears.

With the DVL bridge running and nothing else changed, the same probe reads
`0xa7` → `0x12f` — `POS_HORIZ_REL` set, `CONST_POS_MODE` clear. The two runs
differ in exactly one thing, so the DVL is doing the work. Evidence in
[008](./008-end-to-end-fusion-verified.md).

> Caution about `VELOCITY_HORIZ`: it is set in **both** runs. It reports the
> velocity variance being under threshold, and in `CONST_POS_MODE` the filter's
> velocity is fake-constrained to zero, which trivially passes. **`VELOCITY_HORIZ`
> alone proves nothing.** The discriminating pair is `CONST_POS_MODE` (must be
> clear) and `POS_HORIZ_REL` (must be set). This sharpens
> [007](./007-how-to-read-ekf3-health.md)'s mechanism.

### What breaks without absolute position — less than expected

The ticket predicted the mission would degrade or refuse. It does not, provided
the DVL is actually locked:

- **Arming** needs `ARMING_CHECK` relaxed — the GPS-related prearm checks veto
  otherwise. Recorded, **not** persisted: the test scripts set `ARMING_CHECK 0`
  at runtime over MAVLink. Nothing in the repo was changed. Deciding whether the
  committed defaults should carry an arming-check exception is a real question
  and is **not** answered here.
- **`/mavros/local_position/pose` still works.** `AID_RELATIVE` has no absolute
  origin but does maintain a local one, and the BT's P-controllers read relative
  position. The full mission — `NavToGate`, `GateAlign`, `GatePass`,
  `SearchMarker`, `MarkerAlign`, `MarkerOrbit` — completed with GPS off. This
  contradicts this ticket's own prediction and is worth stating plainly.
- **`POS_HORIZ_ABS` never sets and never will.** Anything needing a global fix
  (`/mavros/global_position/*`, geofence, RTL-to-home) is dead with GPS off.
  Nothing in the current mission uses those.

### The trap this exposed, which is not about GPS at all

With aiding removed, an unarmed BlueROV2 Heavy **free-falls**: ~0.13 m/s steady
descent, DVL slant range 1.79 m → 0.21 m → bottom lock lost, and once bedded
into the floor it never recovers. Five to ten seconds of idle after spawn is
enough to lose the lock before anything else starts.

This is a buoyancy/depth-keeping property of the existing sim, **not** a DVL or
fusion defect, but it invalidates any run that lets the vehicle idle. Every
result here therefore arms into `ALT_HOLD` first and hands over to the mission
afterwards. A run that skips this shows `stopped aiding` about 20 s in and looks
exactly like a fusion failure — the first two runs of this ticket did, and were
misread that way before the gz DVL topic was instrumented directly.

A related process trap: `banyu_mission`, `por_detection`, `robot_state_publisher`
and `image_bridge` all outlive the launch alongside the `ardusub`/`mavproxy`
pair AGENTS.md already warns about. A surviving `banyu_mission` holds Groot2's
ZMQ port and the next run dies with `Address already in use` — which silently
means no mission ran at all.
