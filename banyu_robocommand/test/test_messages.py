#!/usr/bin/env python3
"""Smallest thing that fails if the wire contract breaks.

Run standalone (no ROS needed, but protobuf==7.35 is):
    python3 src/banyu_robocommand/test/test_messages.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from banyu_robocommand import messages  # noqa: E402
from banyu_robocommand.rc_proto import (  # noqa: E402
    common_pb2,
    rx_common_pb2,
    rx_reports_pb2,
)


def test_heartbeat_roundtrip():
    report = messages.build_heartbeat(
        team_id='ITSN', vehicle_id='UUV1', seq=7,
        state=common_pb2.STATE_AUTO,
        lat=-7.2756, lon=112.7937,
        altitude_hae_m=1.5, depth_m=2.0,
        spd_mps=0.4, heading_deg=93.0, roll_deg=1.0, pitch_deg=-2.0,
        current_task=rx_common_pb2.TASK_INFRA_SURVEY_REPAIR,
    )

    decoded = rx_reports_pb2.RxReport()
    decoded.ParseFromString(report.SerializeToString())

    assert decoded.WhichOneof('body') == 'heartbeat'
    assert (decoded.team_id, decoded.vehicle_id, decoded.seq) == ('ITSN', 'UUV1', 7)
    assert decoded.sent_at.seconds > 0
    hb = decoded.heartbeat
    assert hb.state == common_pb2.STATE_AUTO
    assert hb.vehicle_type == rx_common_pb2.TYPE_UUV  # a UUV is TYPE_UUV even surfaced
    assert hb.flight_phase == rx_common_pb2.FLIGHT_PHASE_UNKNOWN  # UAV-only field
    assert hb.current_task == rx_common_pb2.TASK_INFRA_SURVEY_REPAIR
    assert abs(hb.position.latitude - (-7.2756)) < 1e-9
    assert abs(hb.depth_m - 2.0) < 1e-6


def test_run_declaration_roundtrip():
    request = messages.build_run_declaration(
        team_id='ITSN', seq=1, vehicle_ids=['USV1', 'UUV1'],
        tiers=[common_pb2.TIER_NONE, common_pb2.TIER_CORE,
               common_pb2.TIER_NONE, common_pb2.TIER_NONE],
    )
    decoded = messages.rx_requests_pb2.RxRequest()
    decoded.ParseFromString(request.SerializeToString())

    assert decoded.WhichOneof('body') == 'run_declaration'
    decl = decoded.run_declaration
    assert list(decl.vehicle_ids) == ['USV1', 'UUV1']       # PoR needs >= 2
    assert decl.task2_tier == common_pb2.TIER_CORE
    assert decl.task1_tier != common_pb2.TIER_UNKNOWN       # §3.4.10: never UNKNOWN


def test_topics():
    # Handbook example: robocommand/robotx/RNRX/USV1/report
    assert messages.report_topic('RNRX', 'USV1') == 'robocommand/robotx/RNRX/USV1/report'
    assert messages.request_topic('RNRX') == 'robocommand/robotx/RNRX/request'
    assert messages.command_topic('RNRX') == 'robocommand/robotx/RNRX/command'
    assert messages.COURSE_TOPIC == 'robocommand/robotx/course'


if __name__ == '__main__':
    test_heartbeat_roundtrip()
    test_run_declaration_roundtrip()
    test_topics()
    print('ok')
