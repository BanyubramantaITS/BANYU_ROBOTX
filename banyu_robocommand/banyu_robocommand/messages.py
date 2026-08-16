"""Protobuf/topic layer for RoboCommand RobotX 2026. No ROS in here on purpose,
so it can be unit-tested (and eyeballed) without a running graph.

Wire contract: RobotX 2026 Team Handbook §3.4 (live GitBook, not the PDF) and
the schema at github.com/robonation/robocommand @ f6457fa. See
src/docs/robocommand-integration-research.md.
"""

from .rc_proto import common_pb2, rx_common_pb2, rx_reports_pb2, rx_requests_pb2

TOPIC_ROOT = 'robocommand/robotx'

# §3.4.6 topic layout.
COURSE_TOPIC = f'{TOPIC_ROOT}/course'


def report_topic(team_id: str, vehicle_id: str) -> str:
    return f'{TOPIC_ROOT}/{team_id}/{vehicle_id}/report'


def request_topic(team_id: str) -> str:
    return f'{TOPIC_ROOT}/{team_id}/request'


def command_topic(team_id: str) -> str:
    return f'{TOPIC_ROOT}/{team_id}/command'


def build_heartbeat(
    *,
    team_id: str,
    vehicle_id: str,
    seq: int,
    state: int,
    lat: float,
    lon: float,
    altitude_hae_m: float,
    depth_m: float,
    spd_mps: float,
    heading_deg: float,
    roll_deg: float,
    pitch_deg: float,
    current_task: int,
    vehicle_type: int = rx_common_pb2.TYPE_UUV,
) -> rx_reports_pb2.RxReport:
    """One RxReport carrying a Heartbeat. seq is per-vehicle (§3.4.13)."""
    report = rx_reports_pb2.RxReport(team_id=team_id, vehicle_id=vehicle_id, seq=seq)
    report.sent_at.GetCurrentTime()
    hb = report.heartbeat  # touching the field selects the oneof
    hb.state = state
    hb.position.latitude = lat
    hb.position.longitude = lon
    hb.altitude_hae_m = altitude_hae_m
    hb.depth_m = depth_m
    hb.spd_mps = spd_mps
    hb.heading_deg = heading_deg
    hb.roll_deg = roll_deg
    hb.pitch_deg = pitch_deg
    hb.current_task = current_task
    hb.vehicle_type = vehicle_type
    # flight_phase left FLIGHT_PHASE_UNKNOWN — UAV only.
    return report


def build_run_declaration(
    *, team_id: str, seq: int, vehicle_ids: list, tiers: list
) -> rx_requests_pb2.RxRequest:
    """RunDeclaration for tasks 1-4. tiers is four common_pb2.TaskTier values.

    ponytail: no uav_geofence — this SoS fields no UAV. Add the repeated LatLng
    (first point == last point, §3.4) if a UAV is ever declared.
    """
    request = rx_requests_pb2.RxRequest(team_id=team_id, seq=seq)
    request.sent_at.GetCurrentTime()
    decl = request.run_declaration
    decl.vehicle_ids.extend(vehicle_ids)
    decl.task1_tier, decl.task2_tier, decl.task3_tier, decl.task4_tier = tiers
    return request


TIER_BY_NAME = {
    'unknown': common_pb2.TIER_UNKNOWN,
    'none': common_pb2.TIER_NONE,
    'core': common_pb2.TIER_CORE,
    'advanced': common_pb2.TIER_ADVANCED,
    'disruptive': common_pb2.TIER_DISRUPTIVE,
}
