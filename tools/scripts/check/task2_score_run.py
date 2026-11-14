#!/usr/bin/env python3
"""SIMULATION ONLY: score one task2_sim_e2e.sh run from Gazebo ground truth.

  tools/scripts/check/task2_score_run.py <run dir> [--tier core|advanced|disruptive] [--label text]

Reads <dir>/pipeline_status.txt (the lights' final state from Gazebo) and <dir>/mock_gcs.jsonl (what the GCS received).
Prints one line: PASS/FAIL, the points the UUV earned (heartbeat 25, start buoy 25, light order 150, repair 250,
Advanced circle 25, Disruptive tin 25 + circle 25) and why. Self-check: --self-check.
"""
import json
import re
import sys


def parse_status(text):
    """The last gz message in the file (gz topic -e can print more than one)."""
    blocks = re.findall(r'^data: "(.*)"\s*$', text, re.M)
    if not blocks:
        return None
    return json.loads(blocks[-1].encode().decode('unicode_escape'))


def score(status, gcs_rows, tier):
    reasons, points = [], 0
    heartbeat = any(r.get('kind') == 'uuv_heartbeat' for r in gcs_rows)
    survey = next((r for r in reversed(gcs_rows) if r.get('kind') == 'uuv_pipeline_survey'), None)
    delivery = next((r for r in reversed(gcs_rows) if r.get('kind') == 'uuv_resource_delivery'), None)
    if heartbeat:
        points += 25
    if status is None:
        return False, points, ['no Gazebo status']
    damaged = status.get('damaged_node_id')
    nodes = status.get('nodes', {})
    dstate = nodes.get(f'light_node_{damaged}', {}).get('state', 'unknown')
    expected = ['RED' if i == damaged else 'GREEN' for i in range(1, 5)]
    seg = survey.get('segments') if survey else None
    if survey:
        points += 25  # start buoy reported with the survey
    else:
        reasons.append('no survey')
    order_ok = seg == expected
    if order_ok:
        points += 150
    elif survey:
        reasons.append(f'order {seg} != {expected}')
    repaired = dstate in ('TASK_COMPLETED_GREEN', 'ADVANCED_FLASH', 'DISRUPTIVE_CYCLE')
    if repaired:
        points += 250
    else:
        reasons.append(f'node {damaged} {dstate}')
    ok = order_ok and repaired
    seq = status.get('sequence_config', {})
    if tier in ('advanced', 'disruptive'):
        want_circle = seq.get('advanced_target_color') if tier == 'advanced' else seq.get('disruptive_second_color_circle')
        got = delivery or {}
        if got.get('circle') == want_circle:
            points += 25
        else:
            ok = False
            reasons.append(f'circle {got.get("circle")} != {want_circle}')
        if tier == 'disruptive':
            want_tin = seq.get('disruptive_first_color_tin')
            if got.get('tin') == want_tin:
                points += 25
            else:
                ok = False
                reasons.append(f'tin {got.get("tin")} != {want_tin}')
    return ok, points, reasons or [f'node {damaged} repaired, order {seg}']


def main(argv):
    if '-h' in argv or '--help' in argv:
        print(__doc__)
        return 0
    if '--self-check' in argv:
        st = {'damaged_node_id': 2, 'sequence_config': {'advanced_target_color': 'BLUE'},
              'nodes': {'light_node_2': {'state': 'TASK_COMPLETED_GREEN'}}}
        rows = [{'kind': 'uuv_heartbeat'}, {'kind': 'uuv_pipeline_survey', 'segments': ['GREEN', 'RED', 'GREEN', 'GREEN']},
                {'kind': 'uuv_resource_delivery', 'tin': 'ANY', 'circle': 'BLUE'}]
        assert score(st, rows, 'advanced')[:2] == (True, 475)
        assert score(st, rows[:2], 'core')[:2] == (True, 450)
        assert score(st, [], 'core')[0] is False
        txt = 'data: "{\\"damaged_node_id\\": 3, \\"nodes\\": {}}"'
        assert parse_status(txt)['damaged_node_id'] == 3
        print('task2_score_run self-check OK')
        return 0
    d = argv[1]
    tier = argv[argv.index('--tier') + 1] if '--tier' in argv else 'core'
    label = argv[argv.index('--label') + 1] if '--label' in argv else d
    try:
        status = parse_status(open(f'{d}/pipeline_status.txt').read())
    except (OSError, ValueError):
        status = None
    rows = []
    try:
        for line in open(f'{d}/mock_gcs.jsonl'):
            try:
                rows.append(json.loads(line))
            except ValueError:
                pass
    except OSError:
        pass
    ok, points, reasons = score(status, rows, tier)
    print(f'{label}: {"PASS" if ok else "FAIL"} {points} pts ({"; ".join(reasons)})')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
