# Core, red node matrix after the ring tracker fix (a node first seen green, then red, is one red record)

Same conditions as `core_red_node_matrix_summary.md` (tier core, GPS gate on, active end = green buoy, `--light-seed`
2/1/5/9 = damaged node 1/2/3/4). Fix: the tracker upgrades the last record to RED when the same node turns red and
never downgrades RED to GREEN; sim association distance 0.8 m. Driver: `tools/scripts/task2_sim_e2e.sh`.

| Run | Damaged node | Expected | Reported | Result |
|---|---|---|---|---|
| n1 | 1 | RED GREEN GREEN GREEN | RED GREEN GREEN GREEN | pass |
| n2a | 2 | GREEN RED GREEN GREEN | GREEN RED GREEN GREEN | pass |
| n3 | 3 | GREEN GREEN RED GREEN | GREEN GREEN GREEN RED | **wrong** |
| n4 | 4 | GREEN GREEN GREEN RED | GREEN GREEN GREEN RED | pass |
| n2b | 2 | GREEN RED GREEN GREEN | GREEN RED GREEN GREEN | pass |
| n2c | 2 | GREEN RED GREEN GREEN | GREEN RED GREEN GREEN | pass |

5 of 6 correct, and every run sent a report (before the fix: 4 of 11, 2 without a report). The loop was stopped by the
operator before `n3b` and `n4b`. Node 3 was wrong in this batch and in the earlier matrix (red one slot late); not
investigated. Before the fix, node 2 was right in 3 of 7 runs; with the fix 3 of 3.
