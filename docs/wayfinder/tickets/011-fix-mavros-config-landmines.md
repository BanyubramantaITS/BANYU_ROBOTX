# 011 — Fix the two MAVROS config landmines

`wayfinder:task` · parent: [Map: DVL in sim, fused by ArduSub EKF3](../map-dvl-sim-ekf3.md)
· status: **CLOSED — ruled out of scope**, not resolved
· never on the route after [Choose the transport route from gz sensor to ArduSub](./002-choose-transport-route.md)

> **Why this is closed without an answer.** 002 chose a direct-MAVLink route that
> bypasses MAVROS entirely for the DVL path, so neither defect below blocks
> anything on this map any more. Both are **real** and worth fixing — as a
> separate small piece of work, not as a step toward this destination. The
> findings are preserved here so the next person doesn't have to rediscover them.

## Question

001 surfaced two defects in the existing MAVROS setup. Neither was caused by DVL
work and both are worth fixing regardless; they land here because 005 would trip
over them.

**1. A denylist entry that does nothing.** `banyu_bringup/config/mavros_config.yaml`
L21 denies `vision_speed_estimate`. The actual plugin is named `vision_speed`
(**verified**: `/opt/ros/humble/share/mavros_extras/mavros_plugins.xml` L245,
MAVROS 2.14.0). The denylist is fnmatch-based, so the entry never matches and the
plugin loads. The config asserts one thing and does another.

Decide whether the intent was to deny it — in which case fix the name — or
whether it should now load deliberately, given the DVL work. The file comment
says it mirrors `mavros/apm_pluginlists.yaml`, so check whether upstream has the
same typo before "fixing" it in a way that diverges from the mirror.

**2. The `odometry` plugin fails dirty.** It requires `map_ned` / `base_link_frd`
TF frames that MAVROS does not itself publish. `lookup_static_transform` swallows
the exception and leaves an **uninitialised `Affine3d`**, which `odom_cb` then
uses — so missing frames produce garbage transforms rather than a clean error.

If 002 selects any route through the `odometry` plugin, those frames must be
published deliberately and their absence must be made loud. If 002 routes around
MAVROS entirely, record the hazard here anyway so it does not ambush a later
effort.

Both findings belong in AGENTS.md once resolved — they are exactly the class of
non-obvious wiring that file exists to record.
