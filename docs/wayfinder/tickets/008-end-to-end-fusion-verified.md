# 008 — End-to-end: EKF3 fusing the simulated DVL in a real run

`wayfinder:task` · parent: [Map: DVL in sim, fused by ArduSub EKF3](../map-dvl-sim-ekf3.md)
· status: **CLOSED** · resolved AFK
· blocked by: ~~002~~, ~~004~~, ~~005~~, ~~006~~, ~~007~~, ~~010~~ (all closed)

> **010 is what makes this ticket mean anything.** With SITL's synthetic GPS on,
> EKF3 is already aided from ground truth and a DVL bolted on top will read
> healthy while contributing nothing. Do not run this until 010's negative
> control exists.

## Question

The destination ticket. Run `proof_of_readiness.launch.py` with everything from
003–007 in place and confirm, by 007's mechanism, that EKF3 is fusing the DVL
velocity and staying healthy for the duration.

Before believing any result, clear stale processes — AGENTS.md is explicit that
`sim_vehicle.py`'s `ardusub` child and `mavproxy` outlive their launch, and a
new `ardusub` that can't bind port 5501 makes the gz plugin log
`ArduPilot controller has reset`. Kill all of them first or the run is garbage.

Done when:

- The DVL velocity source shows as active and accepted, not merely present.
- It stays healthy across the full existing mission, including the
  `MarkerOrbit` strafe (the phase most likely to break bottom-lock — see 004).
- Nothing in the existing mission regressed. This effort is not licensed to
  change navigation behaviour; AGENTS.md's velocity-loop finding is explicitly
  out of scope. If fusion changes how the sub flies, that is a finding to
  report, not a thing to tune here.

Resolution records the observed evidence verbatim.

## Resolution

**EKF3 fuses the simulated DVL, across the whole existing mission, with SITL's
synthetic GPS off.** The destination is reached.

### The run

`proof_of_readiness.launch.py`, defaults (`use_dvl:=True`), all stale processes
reaped first. Armed into `ALT_HOLD` at spawn so the vehicle holds depth instead
of free-falling out of bottom lock (see
[010](./010-disable-sitl-synthetic-gps.md) — this step is mandatory, not
cosmetic), then `ros2 service call /banyu/mission/start` hands over to the BT.

### Evidence: EKF3 aiding, sampled every ~5 s for 165 s

```
t= 15.4 armed=True flags=0x12f [ATT,VEL_H,VEL_V,POS_H_REL,POS_V_ABS,PRED_H_REL]
t= 45.0 armed=True flags=0x12f [ATT,VEL_H,VEL_V,POS_H_REL,POS_V_ABS,PRED_H_REL]
 ... identical at every sample ...
t=165.1 armed=True flags=0x12f [ATT,VEL_H,VEL_V,POS_H_REL,POS_V_ABS,PRED_H_REL]
```

Read against [007](./007-how-to-read-ekf3-health.md)'s mechanism and
[010](./010-disable-sitl-synthetic-gps.md)'s caveat:

- `POS_HORIZ_REL` **set** → `readyToUseBodyOdm()` returned true and EKF3 is in
  `AID_RELATIVE`. This is the bit that matters.
- `CONST_POS_MODE` **clear** → not the no-aiding fallback. In the negative
  control this bit is set.
- `POS_HORIZ_ABS` **clear** → no GPS is involved. Nothing here is ground truth
  leaking in.
- **Zero** `stopped aiding` messages for the whole run. The full statustext
  tally is one `fusing odometry` and one `started relative aiding` per IMU, and
  nothing else.

### Evidence: the source held up

```
[dvl] sent=1321 skipped=0 dt=0.099s v_frd=(-0.001,+0.006,+0.132) conf=99
```

1321 `VISION_POSITION_DELTA` messages, **`skipped=0`** — every single gz report
was a `DVL_TARGET_BOTTOM` lock, none dropped. Straight off the gz topic:

```
t=  5.0  msgs=  35  target=DVL_TARGET_BOTTOM  range=1.799  rtf=1.00
t= 45.0  msgs= 322  target=DVL_TARGET_BOTTOM  range=0.565  rtf=1.00
t=105.1  msgs= 755  target=DVL_TARGET_BOTTOM  range=0.648  rtf=1.01
t=165.2  msgs=1190  target=DVL_TARGET_BOTTOM  range=0.654  rtf=1.01
```

### Evidence: the mission, unregressed

```
NavToGate: origin=(-0.03, -0.48, -0.95), target=(-0.03, -0.48, -2.85)
Mode set to GUIDED / Vehicle armed
NavToGate: arrived at gate
GateAlign: object centered
GatePass: through the gate (1.50 m)
SearchMarker: approach complete at 2.47 m
MarkerAlign: object centered
MarkerOrbit: full revolution (360.7 deg)
```

Every phase completed, **including `MarkerOrbit`** — [004](./004-bottom-lock-in-por-pool.md)
flagged the strafe as the most likely place to break bottom-lock, and it did
not: slant range sat at 0.65 m throughout, well inside the sensor's 0.05 m
minimum. Aiding never lapsed during it.

Nothing in the mission was modified for this. No navigation behaviour was
touched; AGENTS.md's velocity-loop finding stands exactly as it was.

### One finding worth reporting rather than fixing

The mission completing at all with GPS off contradicts
[010](./010-disable-sitl-synthetic-gps.md)'s prediction that relative-only
aiding would starve the BT's P-controllers. It does not: `AID_RELATIVE`
maintains a local origin and `/mavros/local_position/pose` keeps working. That
is a real result, but it is *not* licence to conclude the nav architecture is
fine — it was tested once, in a pool, with a perfectly-behaved simulated sensor.

### What this does not prove

- Real DVL hardware. Out of scope for this map, and the gz sensor is
  well-behaved in ways an A50 in a real pool will not be.
- Anything at all if the vehicle is left unarmed. The free-fall trap in
  [010](./010-disable-sitl-synthetic-gps.md) will destroy the result silently.
- Accuracy. This shows EKF3 *accepts and sustains* the velocity, not that the
  resulting state estimate is good. No drift measurement was made.
