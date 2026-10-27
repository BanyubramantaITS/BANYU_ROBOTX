# Core, five repeats, same conditions (tier core, GPS gate on, sim fixture of main_world.world, damaged node 2)

Expected report from the active (green) buoy end: `GREEN RED GREEN GREEN`. Driver: `tools/scripts/task2_sim_e2e.sh`.

| Run | Survey reported | Order | First accept of each ring: (node id, colour, Gazebo x of the vehicle) |
|---|---|---|---|
| 1 | GREEN RED GREEN GREEN | correct | (0 G 3.71) (1 R 2.46) (2 G 0.98) (3 G -0.45) |
| 2 | GREEN RED GREEN GREEN | correct | (0 G 3.74) (1 R 2.32) (2 G 0.93) (3 G -0.49) |
| 3 | GREEN GREEN RED GREEN | **wrong** | (0 G 3.70) (1 G 2.28) (2 R 2.28) (3 G 0.99) |
| 4 | none (no report in 300 s) | no report | (0 G 3.76) (1 R 2.41) (2 G 0.96) (3 G -0.48); mission log ends after `WaitSurveyComplete`, no `Surface` |
| 5 | GREEN GREEN RED GREEN | **wrong** | (0 G 3.72) (1 G 2.22) (2 R 2.22) (3 G 0.79) |

Correct 2 of 5, wrong order 2 of 5, no report 1 of 5. Plus the earlier repeat (`core_repro1_*`, correct) and run 9
(wrong): 3 correct of 7 comparable runs.

Failure pattern: in every wrong run the same physical node (node 2, damaged) is accepted twice at the SAME place
(Gazebo x 2.22-2.28): first as GREEN, then as RED. The extra green pushes the red from slot 2 to slot 3 and node 4 is
never recorded. In the correct runs the node is accepted once, as RED, at x 2.3-2.5. Hypothesis (not checked): the
light is still green, or the camera sees it green, when the node first comes into view, and the tracker does not merge
a green and a red observation of the same spot. Run 4 is a different failure (the BT never reaches the surface step).
