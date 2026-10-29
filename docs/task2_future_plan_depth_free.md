# Task 2: plan for a depth-free pipe search and more test scenarios (Marina Bay, limited sensors)

Status: plan only, nothing here is built yet. Written 2026-10-29 after the Core, Advanced and Disruptive sim work
(`task2_feasibility_verdict.md`). Default decisions are taken where the real values are unknown; each is marked **Assume**.

## 1. Problem

Today the dive goes to a fixed `inspection_depth_m` 12.5 m read from the Bar30, and the pipe is followed at that depth. In Marina Bay
the depth of the water, the depth of the pipe and its exact place are unknown, and the handbook says the pipe may span variable depths,
about 1-2 m above the seabed, and may be non-coplanar. The Bar30 can also fail. The robot has: IMU and compass, a bottom camera (HORUS T10,
fisheye, IR LED), an optional hydrophone, RTK GPS only at the surface, and the two buoy lat/long. No altimeter, DVL or sonar.

Goal: the dive depth, the following depth and the repair depth all come from the **camera**; the Bar30 becomes a helper that can be lost.

## 2. Design

### 2.1 Depth-free descent (replaces `DiveToDepth` to a fixed depth)
- Dive by RC heave in ALT_HOLD at a slow fixed rate (start 0.1-0.15 m/s) over the start buoy (GPS position held at the surface first).
- Stop the descent when the camera has locked something: a light blob (GREEN or RED, bright, small, round) or the pipe (a long pale band
  with two parallel edges). Lights are the better first cue: they are seen from about 1.3 m in the sim and flash.
- Hard stops that do not need the depth sensor: a maximum descent time (rate x time = maximum depth, **Assume** 15 m, configurable),
  an abort on the first leak, tilt or battery fault, and a rise to the surface (GPS-fix rule below) on timeout.
- If the Bar30 is alive it is only logged and used to cap the dive; the decision to stop does not depend on it.

### 2.2 Standoff and vertical tracking from the pipe's apparent width
- The pipe is a PVC pipe of a roughly known diameter (**Assume** 0.06-0.12 m, measure at the real site). Range to the pipe is
  `range = fx x diameter / width_px` (fx from the fisheye calibration, undistorted pixels). The same works with the lid of a light node (lid size already
  gives range in the sim: `lid_diameter_m`, `focal_ratio`).
- Hold a standoff by heave: command heave from the error between the measured pipe width and the width for the wanted range (**Assume** 1.0-1.5 m above the pipe).
  Following the pipe then also follows it up and down (the "vertical tracking" the handbook's variable depth asks for), with no depth sensor.
- The surge, sway and yaw control stays as it is (pipe offset and heading in the image). Heave is the new, fourth axis.

### 2.3 Search when the pipe is not in view
The pipe lies between the two buoys, near their ends; the start buoy and the end buoy give the heading and the length. Use a 3D search around the
start buoy, in this order:
1. **Descend in steps** (0.5 m) with a short lateral sweep (+-3 m, along the buoy-to-buoy line first, then across it) at each step. The footprint of the camera at 1.5 m is about 1.7 m wide,
   so the sweep spacing is about 1.2 m.
2. **Use the hydrophone** when it is available: the bearing to the green buoy's pinger (hung below the buoy) says where the start end is, even when the GPS is a few metres off.
3. **Turn the IR LED on and off** at each step and keep the better contrast (backscatter in turbid water can wash the lights out with the LED on, which the sim does not model).
4. After a full sweep without a find, rise 1 m, move 3 m along the line and restart (a shifted pipe, a GPS offset). After N tries, surface and report the Core survey with what was seen
   (an empty survey is still a valid report; the points for the report order are lost, the heartbeat and the start buoy points are kept).

### 2.4 Surface without a depth sensor
The Surface action needs a calibrated water depth today. Add a second rule: **rise by RC heave until a valid, fresh GPS fix appears** (it does not work under water), then hold at the surface.
Time limit: the maximum descent time of section 2.1 plus 50%. This is also the safe response to a Bar30 failure (no depth means rise now).

### 2.5 Bar30 health
Flag it as failed when: the value is stuck for more than 3 s while the heave command is not zero, it steps by more than 0.3 m in one sample, it disagrees with the integrated heave command by more than 1 m,
or its datum drops (the sim barometer already jumps by about 0.5 m at arming). On a failure: go to depth-free mode (2.1-2.4) and log it; never loosen the rule that the datum is only taken at the surface before the run.

### 2.6 Camera model
The sim camera is a pinhole at 320x240; the real one is fisheye 1920x1080. Calibrate the HORUS T10 (OpenCV fisheye), undistort the points that are used (light centres, pipe edges), and keep every
threshold in `task2_params.yaml`. Resample the video to the processing size and keep 25 Hz. The contact lid size 0.14, `lid_diameter_m` 0.074 and `focal_ratio` 0.866 are sim values.

### 2.7 The pipe is not found at the start buoy (today this ends the mission with no report)
Now: the UUV dives at the start buoy, `AcquirePipeline` waits 15 s, then `ReacquirePipeline` does one turning sweep; this is tried 3 times. If all three fail, `DiveAndAcquirePipeline` fails, the mission
ends in fallback SURFACE and **no report is sent** (seen as "Mission failed: fallback mode -> SURFACE"). In the sim the pipe starts about 1.5 m from the buoy, so it is always found; in Marina Bay the GPS error, the buoy drift
on its anchor line and the unknown pipe offset can make the first dive miss it. Plan, in this order:
1. **Widen the search at the start buoy** (section 2.3): steps in depth and lateral sweeps along and across the buoy-to-buoy line, not only a turn on the spot.
2. **Dive in the middle as a second try.** The pipe lies between the two buoys, so a dive halfway along the line meets it more often. This is allowed because the ring order is already sorted by progress **from the active buoy end,
   whichever end or direction the pipe is met from** (H18). From the middle: follow to the end nearer the start buoy, come back, follow to the far end, so that all four nodes are seen, repair the red one when it is met, then surface and report.
   Make it a parameter (`dive_point`: start_buoy, midpoint, or fallback after N failures) and test both orders.
3. **Always report.** After the search budget (parameter, **Assume** 4 minutes) surface by the GPS-fix rule and send the survey with what was seen (possibly empty). The heartbeat and start-buoy points are kept; the order and repair points are lost only for that run.
4. Scenarios (section 4): pipe offset from the buoy line by 0-4 m, heading off by +-25 degrees, GPS offset 1-3 m, a buoy drifted 2 m: acceptance is "found within the budget in 10 of 10 runs, or a report is sent".

## 3. Work items and acceptance

| # | Item | Done when (sim first, then pool, then the site) |
|---|---|---|
| D1 | Light/pipe lock during a slow RC descent, with the depth sensor hidden from the mission | The pipe is found with the Bar30 topic removed, at pipe depths 3, 8, 13 m, 10 of 10 |
| D2 | Pipe-width standoff and heave | The standoff stays within +-0.3 m while the pipe in the world rises and falls by 3 m |
| D3 | 3D search (steps, sweeps, hydrophone prior, LED toggle), the midpoint dive and the always-report fallback (2.7) | Finds a pipe offset by up to 4 m and rotated by up to 25 degrees from the buoy line |
| D4 | GPS-fix surfacing | Surfaces and reports with the Bar30 stuck, offset by 1 m, or silent |
| D5 | Bar30 health monitor | Each fault type in section 2.5 is flagged within 3 s in a replay |
| D6 | Camera calibration and detector retune on the HORUS | Ring and pipe detection recall and false rate on real recordings from a pool and the site |

Order: D4 and D5 first (they make a depth sensor failure safe), then D1, D2, D3, then D6 in parallel as soon as there is real video.

## 4. More test scenarios (sim now, real later)

Make each one a launch option with a seed, as the random spawn is, and log the Gazebo ground truth.

**Pipe and course**
1. Pipe depth from 3 to 13 m in steps; a pipe whose segments are at different depths (a ramp, a step of 1.5 m); a pipe tilted by 10 degrees.
2. The buoys' GPS positions off by 1-3 m, the pipe offset from the buoy line by 0-4 m, the pipe heading off the buoy line by +-25 degrees.
3. Two RED nodes (the handbook says "at least one"); the red node first, last and in the middle; a node on a vertical support (needs a side view or a pitched camera).
4. The pipe continues past the end buoy, or ends before it.

**Water and light** (Marina Bay: a reservoir, green, turbid, boat wakes, no tide; the depth and visibility are unknown)
5. Visibility 0.5, 1, 2 m (the turbidity plugin exists); sun glare and surface reflections near the surface; near dark at depth.
6. IR LED on and off with backscatter; the camera blinded by a reflection of the lights on the pipe.
7. Floating or sunk objects, fish and bubbles that look like lights; a boat's lights and reflections on the water that look like the start buoy's indicator.
8. A current or a wake of 0.1-0.3 m/s (a constant push, then a gust): the hold on the node and the hold at the surface must keep working (the vehicle already drifts 0.06-0.15 m/s with no command).

**Sensors and the vehicle**
9. Bar30: stuck, offset, noisy, silent mid-dive; the datum jump at arming (already seen).
10. GPS: RTK float instead of fixed (decimetres), multipath near the dock, a lost fix at the surface in the middle of the transit.
11. Compass off by 10-30 degrees or disturbed by the pipe's metal; the camera lens dirty or half covered.
12. A thruster out (the sim has the `degraded_thrusters_2_3.parm` set), a low battery, a leak flag.

**Light node faults** (from the build guide, `UNDERWATER_MAGNETIC_ACTIVATED_LIGHT.md`)
13. `BATTERY_THERMISTOR_ERROR` blinks RED at 500 ms, `MAG_READ_ERROR` blinks RED at 250 ms, a hot node switches the LEDs off: none of these may be taken as the damaged node, as the repair, or as the flash code.
14. The repair needs 0.5-3 s of magnetic hold and the trigger can be 23-39 mm away (the magnet size is not decided): sweep the hold time and the trigger distance in the sim.
15. The Advanced flash can be GREEN: it must not be taken for the steady GREEN after the repair or for the dark half.

**Comms and the run**
16. The start-point message late or twice; the run start before the course; a heartbeat gap; the WiFi link lost at the surface (the report is resent until acknowledged, up to 60 s).
17. The UUV starts at the dock, not inside the course; the transit crosses the pipe line; another vehicle's wake.
18. Time limit per attempt (not known yet): log the time of each stage so the budget can be set when the rule is known.

## 5. Data to collect at the site before tuning (cheap and very useful)

- A video from the HORUS (LED on and off) over any pipe or edge at 1, 1.5 and 2 m, in the real water: for the visibility, the colours and the lens model.
- The water depth at the dock and near the buoys (a weighted line is enough), and the surface GPS error at the dock (RTK fixed or float).
- The pipe diameter and the light node dimensions when the organisers publish them, and the magnet that is used for the probe.
- A bag of the Bar30, IMU and compass while the vehicle is lowered by hand to known depths (the real datum jump and drift).

## 6. Defaults taken while the real values are unknown

Maximum depth 15 m, descent rate 0.12 m/s, standoff 1.2 m, pipe diameter 0.1 m, search sweep +-3 m, sweep spacing 1.2 m, give up and report after 3 shifted sweeps.
All are parameters; replace them with the site values.
