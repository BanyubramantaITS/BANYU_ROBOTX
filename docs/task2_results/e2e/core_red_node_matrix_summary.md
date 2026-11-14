# Core, red node matrix (active end = green buoy, tier core, GPS gate on, `--light-seed` 2/1/5/9)

| Damaged node (from the active end) | Seed | Expected | Reported | Result |
|---|---|---|---|---|
| 1 | 2 | RED GREEN GREEN GREEN | RED GREEN GREEN GREEN | pass |
| 2 | 1 | GREEN RED GREEN GREEN | none in 300 s | no report (the BT never reached the surface step) |
| 3 | 5 | GREEN GREEN RED GREEN | GREEN GREEN GREEN RED | wrong (red one slot late) |
| 4 | 9 | GREEN GREEN GREEN RED | GREEN GREEN GREEN GREEN | wrong (red never recorded) |

1 of 4 correct. With the 5-run repeat of node 2 (`core_batch1_summary.md`): 2 of 5 correct there. The other end of the
pipe as the active end (R5-R8) was not run: the sim fixture is fixed to the green buoy side.
