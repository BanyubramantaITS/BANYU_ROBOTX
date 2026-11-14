# Task 2 Core, Advanced, Disruptive: feasibility verdict (simulation)

Branch `feat/task2-core-disruptive-sim`. Updated 2026-11-02 after the worst-case run (goal: minimal sensors, bad conditions). Every result
is judged from Gazebo ground truth (`/robotx/pipeline/status`, `ring_recorder.py`) and scored by `tools/scripts/check/task2_score_run.py`.
Simulation only; nothing here is a hardware result. Hypotheses H1-H38 and every run: `task2_run_log.md`. Earlier verdict (H1-H23): git history.

## Verdict per sensor set

| Sensor set | Verdict | Evidence |
|---|---|---|
| Bottom camera + Pixhawk (IMU, compass) + RTK GPS at the surface + **Bar30** (ALT_HOLD = depth hold) | **YES in the sim, all three tiers** | Random spawn on the final code: Core 4/4 (450), Advanced 4/4 (475), Disruptive 4/4 (500); Disruptive colour pairs 7/8, Advanced circle GREEN 2/2. |
| Bottom camera + Pixhawk + RTK GPS, **no depth sensor** (STABILIZE) | **FAILED in the sim** (repair and the traverse) | The dive, the GPS-fix surfacing, the order without a pipe map and the always-report path work; holding a height over the pipe and the repair do not (details below). |

**Sensor limit that decides it:** with no depth sensor ArduSub cannot use ALT_HOLD; in STABILIZE the heave stick is raw thrust. In this sim
a few PWM past the deadband sink the vehicle at 0.2-1.2 m/s and below it the vehicle floats up at about 0.04 m/s, too coarse to sink
slowly onto the magnetic switch with the lid centred, and too coarse to hold a standoff once the pipe leaves the view (four runs:
floated to the surface or sank to the seabed). The repair (250 points) needs a depth hold. **Keep the Bar30 on the real vehicle.** The
pipe depth is still unknown with the Bar30, so its dive must also stop by camera (planned H39).

**Fallback:** `task2_gamble_strategy.md` (GPS midway between the buoys, a Bar30 dive with timed surge/sway/yaw legs, a guessed light order; at most 200 points, about 87.5 on average; sim only).

## Success rates (Gazebo ground truth, Bar30, final code)

| Case | Result |
|---|---|
| Core, random spawn, nodes 1-4 (seeds 2, 1, 5, 9) | 4/4 PASS 450 |
| Advanced, random spawn, nodes 1-4 | 4/4 PASS 475 (seed 5 needed H24 and H25) |
| Disruptive, random spawn, nodes 1-4 | 4/4 PASS 500 |
| Disruptive colour pairs (seeds 12 18 35 3 43 6 24 34) | 7/8: six PASS 500; seed 18 450 (order unverified, best-effort Core report; H38 now keeps the code); seed 24 FAIL 50 (duplicate records, the RED node never recorded) |
| Advanced circle GREEN (seeds 12, 6) | 2/2 PASS 475 |
| Minimal sensors, Core (middle spawn) | best 200 (order right, repair failed); most runs 25-50 (see the run log) |

Each node has 2-3 passing runs on the final code, not the 5/5 the plan asked for; none of this is a statistical rate.

## What works with the minimal set (built and seen in the sim)

- **D1 camera descent:** sinks until the pipe (with a range from its width) is in view; the dive point is 1.5 m inside the start buoy
  (the pipe is not below the buoy); a light alone does not end the dive (the start buoy's indicator once did).
- **D4 GPS-fix surfacing:** with no depth, rise until a fresh GPS fix proves the surface (seen: z -11.9 to the surface, Surface SUCCESS).
- **D7 order without a pipe map:** ring positions dead-reckoned from the last fix and the commanded velocity, progress along the
  buoy-to-buoy line: the order was right in every run that saw the four nodes.
- **D3 second dive and always report:** a failed dive or traverse surfaces, moves 3 m along the buoy line and dives again; an incomplete
  survey still sends the best-effort Core survey (saved 425 points in Disruptive seed 18).
- **D5 Bar30 health** (silent, stuck, step; the drift check is off after a false fault in ALT_HOLD) and **D2 pipe-width range**: unit tests;
  in the sim D2 alone could not hold the height in STABILIZE.

## Failures fixed in this run

H24 tracker camera height from the pipe width (Advanced seed 5 order); H25 false Bar30 fault and the sim GPS gate; H26 buoyancy integral;
H38 the best-effort report keeps a verified flash code; harness: run start only after `autonomous_ready`, a YAML insertion that disabled
the MAVROS output, processes stopped by PID. Details and the failed attempts (H27-H37b): `task2_run_log.md`.

## Not done

1. Step 4 scenarios (pipe depth 3/8/13 m, visibility, current, compass, GPS float and loss, buoy error, fisheye, IR LED, node faults, Marina
   Bay, pipe moved, the worst-case combo): tools exist in `launch_task2.sh`, only the first run started (it found the fixed-depth problem above).
2. 5/5 per tier and node, 10/10 in the worst-case combo.
3. The duplicate-record failure (Disruptive seed 24).
4. The real camera (HORUS T10 calibration, D6) and every hardware test.
