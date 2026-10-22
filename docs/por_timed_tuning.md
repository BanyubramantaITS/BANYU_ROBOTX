# Timed (camera-free) POR: Sim Results and Real-Pool Tuning

Course: start → 3 m → gate (2 m wide, bar 1 m deep) → 10 m → marker. In sim
the start is at x = -7, the gate at x = -4 and the marker at x = +6. The pool
is 19 × 10 × 2.5 m.

## ALT_HOLD update (October 2026)

Everything below this section was measured when POR flew `GUIDED` with velocity
setpoints, from an underwater spawn (z = -2). POR now flies `ALT_HOLD` with RC
override (`control.output_mode: rc_override`) and the sim spawns at the surface,
so the motion model below no longer applies:

- Speed is open loop. In sim, actual speed ~= 1.34 * sqrt(speed_x) forward/back and
  ~= 1.39 * sqrt(speed_y) sideways (gain_xy 300); e.g. 0.9 -> 1.28 m/s.
- The vehicle coasts ~0.7-1 m after each leg (no active braking as in GUIDED).
- The dive happens in place from the surface and ends on `check_depth_reached`.
- Measured in sim time; Gazebo runs at ~0.7x real time on the dev laptop.

| Profile | ALT_HOLD sim result | Gate back (sim time) |
|---|---|---|
| `por_params_timed_fast.yaml` | ✅ re-timed | ~31 s |
| `por_params_timed_safe.yaml` | ✅ re-timed | ~36 s |
| `por_params_timed_fastest.yaml`, `..._fastest_turn.yaml` | ❌ diamond does not close; not re-timed yet | - |
| `por_params.yaml` (turn box) | ❌ as before | - |
| hybrid, vision_assisted, slow, dead-thruster | not run yet | - |

## Results (Gazebo + ArduSub SITL, ground-truth scored)

_GUIDED-era results (historical):_

A run passes when it crosses the gate going out (|y| ≤ 0.66 m, below the bar,
above the floor), makes a full loop around the marker with ≥ 0.5 m clearance,
crosses the gate going back, and stays ≥ 0.3 m from every wall.

| Profile | File | Time | Result |
|---|---|---|---|
| Original defaults | `por_params.yaml` | 89 s | ❌ BT reports SUCCESS but the robot only reaches x≈0 (14 s of forward motion is too short). The extra `return_turn` points it at the side wall. |
| Turn box, fixed (A3) | values below | 102 s | ✅ but heading error builds up turn by turn (ends about 6° off) |
| **SAFE** strafe box | `por_params_timed_safe.yaml` | 87 s | ✅ speed ±5%, heading ±5°, 0.5 m start offset; pure-timer ✅ |
| **FAST** strafe box | `por_params_timed_fast.yaml` | 64 s | ✅ speed −8%..+5%, heading ±6°, pure-timer ✅ (fails at +8% speed) |
| **FASTEST** diamond (D3) | `por_params_timed_fastest.yaml` | 43.6 s (gate back 33.3 s) | ✅ speed ±6%, heading +6° (fails at +8% speed) |
| Camera return (hybrid) | `por_params_hybrid.yaml` | 70.8 s | ✅ FAST outbound, then turns 180° and uses the camera to pass the gate |

Launch:

```bash
ros2 launch banyu_bringup main_launch.launch.py mode:=por \
  por_params_file:=$(ros2 pkg prefix banyu_bringup)/share/banyu_bringup/config/por_params_timed_fast.yaml
```

### Why the strafe box beats a turn box

- An IMU-capped 90° turn at 0.6 rad/s **overshoots by about 14°**, because the
  cap stops the command but the vehicle keeps rotating. Four turns stack the
  error, and over the 10 m return leg 12° becomes **2.7 m** of lateral miss.
- The BlueROV2 Heavy can strafe, so the box keeps yaw fixed. The `circle_turn_*`
  slots become strafes (`speed_y`, `yaw_rate 0`, `target_angle 0`). The robot
  then **backs up** through the gate along the same line it drove out on, so a
  start-heading error moves the outbound and return paths together and the
  gate stays hit.
- With no turns there is no compass dependency, which suits a pure-timer run.

### Measured motion model (sim)

`distance ≈ speed × (duration − dead_time)`, with a gain of about 1.0:

| Motion | Dead time |
|---|---|
| Forward | 0.55 s |
| Backward | 0.85 s |
| Strafe | 0.6–0.75 s |

Depth: baro mode holds `gate_depth_m`. In pure-timer mode, ArduSub holds the
depth reached at the end of the dive (0.4 m/s × 4.4 s → about 1.85 m).

### What limits tolerance

- **Speed error** moves the whole loop along x (10% × 13 m ≈ 1.3 m). The pool
  only has 3.5 m behind the marker, which caps this at about ±5–8%.
- **Start heading error** moves the loop sideways (tan θ × 13 m). Lateral
  half-size 2 m gives about ±6°. Widening the strafes buys more tolerance at
  the cost of time.

## Real-pool tuning procedure (stopwatch + tape)

1. **Calibrate speed, the most important step.** Run one straight segment at
   the speed you'll use (0.9 for FAST, 0.6 for SAFE) for 10 s. Measure the
   distance and compute `v_real = d / (10 − 0.55)`. Do the same backward and
   for strafes. You need these to within **~4%**. Repeat on low and full
   battery.
2. **Scale durations.** For each segment:
   `new_duration = (planned_distance / v_real) + dead_time`. Planned distances
   (FAST):

   | Segment | Planned distance |
   |---|---|
   | Dive (while moving at 0.5 m/s) + forward | to 2 m before the marker (≈ 11 m from start) |
   | Strafe right | 2 m |
   | Forward | 3.5 m |
   | Strafe left | 4 m |
   | Backward | 3.5 m |
   | Strafe right | 2 m |
   | Back up | through the gate and 2 m past it |

3. **Set depth.** `gate_depth_m` should sit midway between the bottom of the
   bar and the floor (1.8 m in a 2.5 m pool). For pure timer, set
   `dive_duration_s ≈ depth / 0.42` (sim: 4.4 s → 1.85 m) and check it with a
   depth reading.
4. **Align the start heading** to the gate–marker line within 3–5°, using a
   line on the pool floor or a sighting pole. This is the second biggest error
   source.
5. **Tune in order:** dive → straight to the marker → box → return. Fix one
   segment at a time and film from the deck.
6. If the robot hits the marker or the far wall, shorten `approach_pole`. If
   the loop misses the marker, lengthen it. If it misses the gate on return,
   check heading alignment first, then the strafe balance (left = right + right).

## Turn-based alternative (A3, passes but slower and less robust)

Use this only if strafing is weak on the real vehicle:
`approach_pole 13.5 s @0.6`, turns at `yaw_rate ±0.4` with
`target_angle 1.45` (cap set short to absorb overshoot), `duration 6`. Legs are
4.0 / 7.5 / 7.8 / 7.5 s @0.5. `return_turn` strafes back to the centre line
(`speed_y 0.5`, 4.2 s, `target 0`). `return_to_gate` is 20.5 s @0.6.

## Fastest: combined-axis diamond (D3)

Fixed yaw. Every segment combines axes, so the robot never stops to change
direction.

| # | Segment | Duration | vx | vy | vz | Path |
|---|---|---|---|---|---|---|
| 1 | dive | 3.5 s | +1.1 | 0 | −0.6 | Dives while driving forward (baro hold 1.8 m) |
| 2 | pass_gate | 0 | | | | Skipped, merged into dive + approach |
| 3 | approach_pole | 7.8 s | +1.1 | 0 | 0 | Through the gate to about 3.3 m before the marker |
| 4 | circle_turn_1 | 2.9 s | +1.0 | −1.2 | 0 | Diagonal front-right |
| 5 | circle_forward_1 | 2.9 s | +1.0 | +1.2 | 0 | Diagonal front-left, past the marker |
| 6 | circle_turn_2 | 2.9 s | −1.0 | +1.2 | 0 | Diagonal back-left |
| 7 | circle_forward_2 | 3.4 s | −1.0 | −1.2 | 0 | Diagonal back-right. Longer, so the diamond closes. |
| 8 | return_to_gate | 11.8 s | −1.1 | 0 | 0 | Backs up through the gate |
| 9 | surface | 5 s | 0 | 0 | +0.4 | |

Slots that aren't listed have duration 0. The loop takes about 12 s, against 26 s for the strafe box.

Development history (sim):

| Variant | Gate back | Notes |
|---|---|---|
| FAST box | 55 s | baseline |
| D1/D2 diamond (0.7 / 0.85 m/s per axis) | 44.8 / 41.8 s | Same outbound as FAST |
| F1c (0.9 m/s outbound, combined dive) | 40.3 s | Fails at −6° heading (diamond didn't close) |
| F2c (1.1 m/s outbound, 1.0 m/s diamond) | 33.7 s | Fails at −6° heading |
| **D3** (F2c + vy 1.2, closing leg 3.4 s) | **33.3 s** | ✅ ±6% speed, +6° heading |
| Hexagon (H1) | 46.5 s | Slower; coasting stretches it in x, so no extra margin |
| Orbit (vy −0.8 + yaw +0.4) | 44.7 s | Passes, but heading drift moves the return 0.3 m off-centre |

Hard limit: the marker is only 3.5 m from the far wall. At +8% speed error the
whole loop shifts about 1.3 m toward that wall, so no loop around the marker
fits. Calibrate the real speed to within ±5% (see the stopwatch procedure).

Sim testing note: keep the sim off the robot's LAN DDS domain
(`ROS_DOMAIN_ID=<unused>`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`).
Traffic from the robot trips the execution safety watchdog ("Vehicle safety or
run authorization fault").
