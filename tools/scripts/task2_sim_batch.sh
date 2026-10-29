#!/bin/bash
# SIMULATION ONLY: run task2_sim_e2e.sh once per light seed (seeds 2, 1, 5, 9 damage node 1, 2, 3, 4) and print, from
# Gazebo ground truth, the damaged node's final state and the reported order.
#   tools/scripts/task2_sim_batch.sh --out <dir> [--tier core|advanced|disruptive] [--seeds "2 1 5 9"] [--headless]
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT=""; TIER="core"; SEEDS="2 1 5 9"; MODE="--gui"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --out) OUT="$2"; shift 2 ;;
    --tier) TIER="$2"; shift 2 ;;
    --seeds) SEEDS="$2"; shift 2 ;;
    --headless) MODE=""; shift ;;
    *) echo "unknown argument $1" >&2; exit 1 ;;
  esac
done
[ -n "$OUT" ] || { echo "--out is required" >&2; exit 1; }
for SEED in $SEEDS; do
  "${SCRIPT_DIR}/task2_sim_e2e.sh" $MODE --out "$OUT/seed$SEED" --tier "$TIER" --launch-arg --light-seed --launch-arg "$SEED" > "$OUT.seed$SEED.out" 2>&1
  python3 - "$OUT/seed$SEED" "$SEED" <<'PY'
import json, re, sys
d, seed = sys.argv[1], sys.argv[2]
node = {'2': 1, '1': 2, '5': 3, '9': 4}.get(seed)
try:
    status = open(f'{d}/pipeline_status.txt').read()
    m = re.search(r'light_node_%s[^}]*?\\"state\\": \\"([A-Z_]+)' % node, status)
    state = m.group(1) if m else 'unknown'
except OSError:
    state = 'no status'
seg = None
try:
    for line in open(f'{d}/mock_gcs.jsonl'):
        if 'uuv_pipeline_survey' in line:
            seg = json.loads(line)['segments']
except OSError:
    pass
print(f'seed {seed} node {node}: final state {state}, reported {seg}')
PY
done
