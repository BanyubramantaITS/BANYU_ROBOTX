# Task 2: next plan (after the worst-case run of 2026-11-02)

Decision from the sim: the real Task 2 uses the bottom camera, the Pixhawk (IMU, compass), RTK GPS at the surface **and the Bar30**
(ALT_HOLD depth hold). With no depth sensor the repair and the traverse failed in the sim (`task2_feasibility_verdict.md`). The pipe depth
and place are still unknown, so the camera-based parts built for the minimal set are kept and run on top of the depth hold.

## Work items, in order

1. **H39 `sensors:=baro` profile.** ALT_HOLD with the Bar30, plus the camera descent (stop when the pipe is in view, not at a fixed depth),
   the pipe-width standoff (in ALT_HOLD the heave stick is a climb rate, so the camera loop is fine), the dive point 1.5 m inside the
   start buoy and the dead-reckoned order along the buoy line (no pipe map). D5 health and D4 GPS-fix surfacing stay as the fallback when
   the Bar30 fails. `launch_task2.sh --sensors baro`. Done when Core passes at pipe depth 3, 8 and 13 m (one run each).
2. **Duplicate records (Disruptive seed 24).** The tracker made four records before the RED node (one node seen from far or from 0.9 m
   beside the line) and the cap of four kept the RED one out. Keep more candidates and merge them by spacing at the end of the traverse,
   or merge records closer than the node spacing. Done when seed 24 and a 0.9 m lateral offset case pass.
3. **Re-run step 1 and 2 in the baro profile** (random spawn, Core, Advanced, Disruptive nodes 1-4, the colour seeds).
4. **Scenarios (baro profile)** with the existing options: pipe depth 3/8/13, visibility 1 and 2 m, current 0.2 m/s with gusts,
   compass 20 deg, GPS float and a lost fix in the transit, buoy report error 2 m, fisheye, IR LED, a node fault (thermistor, mag, hot),
   Marina Bay site, the pipe moved and rotated; then the worst-case combo.
5. **5/5 per tier and node** (random spawn), 10/10 light order and repair in the worst-case combo.
6. **Update** `task2_feasibility_verdict.md` and `task2_sim_results.md`.

## Before the water

- Measure the real pipe diameter and calibrate the HORUS T10 (fisheye, equidistant): the range from the pipe width needs both.
- Check the Bar30 on the bench and in the pool (the datum is taken at the surface before the run).
- Pool test: dive, camera lock, repair on a mock node with the magnet, surfacing, report.

## Known sim limits

The sim EKF keeps a near-perfect position under water (the real one has none: the order uses dead reckoning in the minimal and baro
profiles); the sim camera is 320x240 pinhole; turbidity has no backscatter from the vehicle's light; the STABILIZE thrust response is
specific to the sim model.
