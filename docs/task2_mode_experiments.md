# Task 2: Pixhawk mode behaviour with the GPS lost under water (simulation)

Run with `sim_gps_gate` (worst case: GPS off as soon as the body is 0.1 m below the surface, back on 10 s after
surfacing) and `tools/scripts/gps_loss_mode_experiment.py`, one fresh sim per mode. The sim batches are in the
scratchpad (not committed); numbers below are copied from their JSONL logs. Plan: `task2_core_disruptive_plan.md`
section 6.

Method per mode: arm, dive to 1.5 m, request the mode, hold 40 s with neutral input, then a 20 s forward leg at a
commanded 0.3 m/s, then surface and time how long GUIDED takes to be accepted again. Ground truth is Gazebo's pose
(`gz topic /world/ocean/dynamic_pose/info`). Depth drift is positive downward: a negative value means the vehicle
rose.

| # | Mode (input) | Accepted | Depth drift in 40 s | Forward leg, 20 s at 0.3 m/s | Notes |
|---|---|---|---|---|---|
| M1 | GUIDED, already active when the GPS is cut (velocity setpoints) | yes | +0.04 m | 5.46 m (91% of the commanded 6.0 m), no sideways drift | EKF3 logs "stopped aiding" and lane switches, mode never cancelled |
| M1b | GUIDED requested after the GPS has been gone 20 s (dive in ALT_HOLD first) | **yes** | **-1.1 to -1.2 m (rose)** | **-0.08 m: the setpoints are ignored** | The mode switch succeeds but the vehicle stops tracking and loses depth hold. A trap: nothing reports the failure |
| M2 | GUIDED + zero-displacement ExternalNav beacon | not run | | | Known unsafe (`task2_underwater_position.md`): the beacon says the vehicle is still, so velocity control keeps adding thrust |
| M3 | ALT_HOLD + `/mavros/rc/override` | yes | +0.03 to +0.04 m | 10.27 m with +90 PWM on forward (0.51 m/s) | Holds depth and heading, returns to GUIDED 2.0 s after the GPS is back |
| M4 | STABILIZE + RC override | yes | -1.6 m (rose) | 3.3 m | No depth hold: the vehicle is positively buoyant and rises |
| M5 | MANUAL + RC override | yes | -1.46 m (rose) | 3.6 m | Same, no stabilisation |
| M6 | POSHOLD + RC override | yes | +0.06 m | 3.94 m (0.2 m/s) | Accepted and held depth in the sim, but it needs a position estimate; on the robot this is not trusted |

Return to GUIDED after surfacing was accepted 2.0 s after the GPS came back in every trial (the first attempt
after the 10 s reacquire delay).

## Limits of these numbers

- **The sim IMU is perfect.** M1 works because the EKF dead-reckons a velocity that has no error. On the robot the
  accelerometer error is not observed once the GPS is gone, so the EKF velocity drifts and GUIDED would track a
  wrong measurement. I tried to inject that error: `SIM_ACC1_BIAS_*` has no effect with the JSON (Gazebo) backend,
  and a wrong `INS_ACCOFFS_*` calibration offset also changed nothing (forward leg 5.45 m, the same as without it).
  The robot-side drift of M1 is therefore **unverified in the sim**, not shown to be safe.
- The forward-speed scale of the RC override differs between the sim (0.51 m/s for +90 PWM, so about 175 PWM per
  m/s) and the POR gain assumed in `por_params.yaml` (300 PWM per m/s). The real value must come from `TimedMove`
  on the robot (plan section 9).

## Decision

Underwater (after `DiveToDepth`): **ALT_HOLD with RC override**, the same path as POR. Reasons:

1. It holds depth and heading without any position estimate (M3), so it does not depend on the GPS or an IMU that
   the sim cannot make imperfect.
2. M1b shows that GUIDED has a silent failure mode right after the GPS is lost, and the plan cannot tolerate a mode
   switch that succeeds while the vehicle stops following commands.
3. The surface legs (`NavigateSurfaceGps`, `AlignHeading`) stay in GUIDED with the GPS, and the switch back is clean.

Keep GUIDED for the surface only. Do not use STABILIZE/MANUAL (they rise) or POSHOLD (needs a position).
Implementation work item W2 changes the mode in the BT and in `vehicle_execution` accordingly.
