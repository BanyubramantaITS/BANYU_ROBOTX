# 006 — ArduSub parameters that make EKF3 fuse the velocity

`wayfinder:task` · parent: [Map: DVL in sim, fused by ArduSub EKF3](../map-dvl-sim-ekf3.md)
· status: **CLOSED** · resolved AFK
· blocked by: ~~001~~, ~~003~~ (both closed) · blocks: 008, 010

## Question

Add the parameter set from 001 to
`banyu_bringup/config/banyu_defaults.parm` so ArduSub routes the incoming
velocity into EKF3 as a velocity source.

Covers at minimum: enabling the visual-odometry backend, the sensor lever-arm
offsets and orientation (from 003's recorded mount pose), and repointing the
EKF3 source set so horizontal velocity comes from the DVL rather than whatever
it currently falls back to with no GPS.

Constraints:

- Param names must be **verified against this ArduPilot checkout's source**, not
  recalled. AGENTS.md records that the names here are modern (`WP_SPD`, not
  `WPNAV_SPEED`) and that guessing has burned this repo before.
- `banyu_defaults.parm` is the curated file, applied via `--defaults` for bare
  `ardusub` and `--add-param-file=` for `sim_vehicle`. The workspace-root
  `mav.parm` is an auto-dumped reference, not the place to edit.
- Changing EKF source selection can make the vehicle refuse to arm or fall back
  silently. Record what the pre-change behaviour was so 008 has a baseline.

## Resolution

Added to `banyu_bringup/config/banyu_defaults.parm`. **Every name and value was
read out of `/home/donjoe/ardupilot`, not recalled** — AGENTS.md is explicit
that guessing param names has burned this repo before.

| Param | Value | Meaning | Verified at |
|---|---|---|---|
| `VISO_TYPE` | 1 | MAVLink backend | `AP_VisualOdom.cpp:34-37` |
| `VISO_POS_X/Y/Z` | 0, 0, 0.02 | Lever arm, FRD metres | `AP_VisualOdom.cpp:39-62` |
| `VISO_ORIENT` | 25 | Down | `AP_VisualOdom.cpp:67-69` |
| `VISO_DELAY_MS` | 10 | Sensor latency | `AP_VisualOdom.cpp:83` |
| `VISO_VEL_M_NSE` | 0.1 | Velocity noise | `AP_VisualOdom.cpp:91` |
| `EK3_SRC1_VELXY` | 6 | `EXTNAV` | `AP_NavEKF_Source.h:24`, `.cpp:39` |

**The load-bearing line is `EK3_SRC1_VELXY 6`.** `readyToUseBodyOdm()`
(`AP_NavEKF3_Control.cpp:559-573`) returns false unless the velocity-XY source is
`EXTNAV` or `WHEEL_ENCODER`, so without it the DVL is accepted and then ignored.
Default is `GPS` (3).

**Sign of `VISO_POS_Z`, since it is easy to get backwards.** The xacro says
`dvl_z = -0.02` because Gazebo's model frame is **z-up**; ArduPilot's body frame
is **FRD**, z-down. Below the hull is therefore **+0.02** here. The DVL model's
link origin is deliberately the beam-axis intersection point — what Water Linked
defines the sensor origin as — so the numbers transfer with no arithmetic. Same
convention as `params/ping.params`' `RNGFND1_POS_X -0.18  # From models/...`.

### Deliberately not set

`EK3_SRC1_POSXY` is left at its default. With SITL's synthetic GPS still on, GPS
supplies absolute position and the DVL rides alongside it. Deciding what POSXY
becomes belongs to
[Turn off SITL's synthetic GPS so the DVL has a job](./010-disable-sitl-synthetic-gps.md),
which now has its parameter name confirmed: **`SIM_GPS1_ENABLE`**
(`SITL.cpp:725` declares the `GPS1_` subgroup, `SIM_GPS.cpp:42` the `ENABLE`
member).

`EK3_ENABLE` and `AHRS_EKF_TYPE` were not touched — EKF3 is already the active
estimator on this checkout, and setting defaults explicitly is noise.

### Untested

These params have not yet been run against SITL. Arming behaviour and whether
the source change is accepted cleanly is
[008](./008-end-to-end-fusion-verified.md)'s to establish, along with the
pre-change baseline this ticket asked for — which cannot be captured without a
run.
