#!/usr/bin/env python3
"""Reference GCS-side bridge: RobotX SoS ROS topics (domain 10) <-> RoboCommand MQTT.

Wraps the unmodified GcsReporter from robocommand-rx-ui-itsn/report.py and adds what an
integration test against BANYU's UUV showed it lacks:

  1. Command forwarding: RxCommand (RunStart, Task 4) -> rx_msgs/Command on /system/mission/command.
  2. Stale telemetry: heartbeats for a vehicle stop after --stale-timeout-s without fresh ROS
     heartbeats instead of replaying the last position (a dived UUV is not reachable).
  3. Restart safety: report/request sequences, declaration and run_id persist in --state-file, so a
     restarted GCS never restarts report seqs at 1 or re-declares the run.
  4. One-shot reports: PipelineSurvey / ResourceDelivery are de-duplicated per declared run by
     payload hash, because latched ROS topics re-deliver them to a restarted subscriber.
  5. Next run: /gcs_ros_bridge/new_run (std_srvs/Trigger) sends a new RunDeclaration, like the
     nala2 OCS's /nala2_report/new_run. RoboCommand may then reuse run_id 1 ("Reset run ID & seq").

It is a reference for the GCS team, not part of the vehicle stack. Run from the GCS repo venv with
ROS sourced and ROS_DOMAIN_ID=10:
  python gcs_ros_bridge.py --host 127.0.0.1 --team ITSN --state-file gcs_state.json
"""
import argparse
import hashlib
import json
import logging
import os
import sys
import threading
import time

# Default: robocommand-rx-ui-itsn checked out next to the colcon workspace.
_DEFAULT_GCS_REPO = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', '..',
    'robocommand-rx-ui-itsn'))
GCS_REPO = os.environ.get('GCS_REPO', _DEFAULT_GCS_REPO)
sys.path.insert(0, GCS_REPO)
# rclpy message packages need numpy/yaml from the system; appended so the venv protobuf wins.
sys.path.append('/usr/lib/python3/dist-packages')

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rx_msgs.msg import (Command, Course, Heartbeat, IncidentAck, LatLng,  # noqa: E402
                         MissionStatus, PipelineSurvey, ReadinessReport, ResourceDelivery,
                         VehicleType)
from std_srvs.srv import Trigger  # noqa: E402

from report import GcsReporter  # noqa: E402
from robocommand_ui.proto_helper import common_pb2, rx_common_pb2, rx_reports_pb2  # noqa: E402

VOLATILE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
log = logging.getLogger('gcs_ros_bridge')


class BridgeReporter(GcsReporter):
    """GcsReporter with command callback, stale-heartbeat policy and persisted sequences."""

    def __init__(self, *args, on_command=None, state_file=None, stale_timeout_s=3.0, **kwargs):
        super().__init__(*args, **kwargs)
        self._on_command_cb = on_command
        self._state_file = state_file
        self._stale_timeout_s = stale_timeout_s
        self._telemetry_rx = {}
        self._stale_logged = set()
        self.sent_reports = set()
        self._load_state()

    # ---- persistence ----
    def _load_state(self):
        if not self._state_file or not os.path.exists(self._state_file):
            return
        with open(self._state_file) as handle:
            state = json.load(handle)
        self.seq._request_seq = int(state.get('request_seq', 0))
        self.seq._vehicle_report_seqs = {k: int(v) for k, v in state.get('report_seqs', {}).items()}
        self.declaration_seq = state.get('declaration_seq')
        self.run_id = state.get('run_id')
        self.sent_reports = set(state.get('sent_reports', []))
        log.info('Restored state: request_seq=%d report_seqs=%s declaration=%s run_id=%s',
                 self.seq.get_request_seq(), self.seq.get_all_vehicle_seqs(),
                 self.declaration_seq, self.run_id)

    def save_state(self):
        if not self._state_file:
            return
        state = {'request_seq': self.seq.get_request_seq(),
                 'report_seqs': self.seq.get_all_vehicle_seqs(),
                 'declaration_seq': self.declaration_seq, 'run_id': self.run_id,
                 'sent_reports': sorted(self.sent_reports)}
        temporary = self._state_file + '.tmp'
        with open(temporary, 'w') as handle:
            json.dump(state, handle)
        os.replace(temporary, self._state_file)

    def publish_report(self, vehicle_id, body_field, body):
        ok = super().publish_report(vehicle_id, body_field, body)
        if ok:
            self.save_state()
        return ok

    def _declare_locked(self):
        ok = super()._declare_locked()
        if ok:
            self.save_state()
        return ok

    # ---- telemetry freshness ----
    def mark_fresh(self, vehicle_id):
        self._telemetry_rx[vehicle_id] = time.monotonic()
        self._stale_logged.discard(vehicle_id)

    def heartbeat(self, vehicle_id):
        received = self._telemetry_rx.get(vehicle_id)
        if received is not None and time.monotonic() - received > self._stale_timeout_s:
            if vehicle_id not in self._stale_logged:
                log.warning('%s telemetry stale (> %.1f s); heartbeats paused until it returns',
                            vehicle_id, self._stale_timeout_s)
                self._stale_logged.add(vehicle_id)
            return False
        return super().heartbeat(vehicle_id)

    # ---- commands ----
    def _handle_command(self, command):
        super()._handle_command(command)
        self.save_state()
        if command.team_id and command.team_id != self.team_id:
            return
        if command.WhichOneof('body') == 'run_start' and self.run_id != command.run_start.run_id:
            return      # declaration_seq mismatch: GcsReporter rejected it
        if self._on_command_cb:
            self._on_command_cb(command)


class Bridge(Node):
    def __init__(self, reporter_factory, vehicle_id, event_log):
        super().__init__('gcs_ros_bridge')
        self.vehicle_id = vehicle_id
        self.events = open(event_log, 'a', buffering=1) if event_log else None
        self.course_pub = self.create_publisher(Course, '/system/mission/course', LATCHED)
        self.command_pub = self.create_publisher(Command, '/system/mission/command', VOLATILE)
        self.reporter = reporter_factory(self.on_course, self.on_command)
        self._lock = threading.Lock()
        self.create_subscription(Heartbeat, '/system/vehicle/uuv/heartbeat', self.on_heartbeat,
                                 VOLATILE)
        self.create_subscription(MissionStatus, '/system/mission/status', self.on_status, VOLATILE)
        self.create_subscription(PipelineSurvey, '/system/vehicle/uuv/task2/pipeline_survey',
                                 self.on_survey, LATCHED)
        self.create_subscription(ResourceDelivery, '/system/vehicle/uuv/task2/delivery',
                                 self.on_delivery, LATCHED)
        self.create_subscription(IncidentAck, '/system/vehicle/uuv/task4/incident_ack',
                                 self.on_incident_ack, VOLATILE)
        self.create_subscription(ReadinessReport, '/system/vehicle/uuv/task4/readiness',
                                 self.on_readiness, VOLATILE)
        self.create_service(Trigger, '/gcs_ros_bridge/new_run', self.on_new_run)

    def on_new_run(self, _request, response):
        response.success = self.reporter.declare()
        response.message = (f'RunDeclaration seq={self.reporter.declaration_seq}'
                            if response.success else 'not connected to the broker')
        self.event('new_run', ok=response.success, declaration_seq=self.reporter.declaration_seq)
        log.info('new_run: %s', response.message)
        return response

    def event(self, kind, **data):
        if self.events:
            data.update(kind=kind, t=time.time())
            self.events.write(json.dumps(data) + '\n')

    # ---- RoboCommand -> ROS ----
    def on_course(self, course):
        msg = Course(course_id=course.course_id, pinger_freq_hz=course.pinger_freq_hz)
        msg.boundary = [LatLng(latitude=c.latitude, longitude=c.longitude) for c in course.corners]
        msg.uav_geofence = [LatLng(latitude=a, longitude=b) for a, b in self.reporter.geofence]
        self.course_pub.publish(msg)
        self.event('ros_course_out', course_id=msg.course_id, pinger=msg.pinger_freq_hz)
        log.info('Course %s -> /system/mission/course', msg.course_id)

    def on_command(self, command):
        body = command.WhichOneof('body')
        out = []

        def lat_lng(point):
            return LatLng(latitude=point.latitude, longitude=point.longitude)
        if body == 'run_start':
            out.append(Command(command_type=Command.CMD_RUN_START, run_id=command.run_start.run_id,
                               target_vehicle=VehicleType.TYPE_UNKNOWN))
        elif body == 'assistance_request':
            b = command.assistance_request
            out.append(Command(command_type=Command.CMD_TASK4_ASSISTANCE,
                               target_vehicle=b.vehicle_type, position=lat_lng(b.position)))
        elif body == 'keep_out_zone':
            b = command.keep_out_zone
            out.append(Command(command_type=Command.CMD_TASK4_KEEP_OUT_ZONE,
                               target_vehicle=b.vehicle_type, radius_m=b.radius_m,
                               center=lat_lng(b.center)))
        elif body == 'all_clear':
            out.append(Command(command_type=Command.CMD_TASK4_ALL_CLEAR,
                               target_vehicle=command.all_clear.vehicle_type))
        elif body == 'moving_object_alert':
            b = command.moving_object_alert
            for vehicle_type in b.affected_vehicle_types:
                out.append(Command(command_type=Command.CMD_TASK4_MOVING_OBJECT,
                                   target_vehicle=vehicle_type, heading_deg=b.heading_deg,
                                   speed_mps=b.speed_mps, position=lat_lng(b.position)))
        elif body == 'readiness_confirm':
            b = command.readiness_confirm
            target = VehicleType.TYPE_UUV if b.vehicle_id.upper().startswith('UUV') \
                else VehicleType.TYPE_UNKNOWN
            out.append(Command(command_type=Command.CMD_TASK4_READINESS_CONFIRM,
                               target_vehicle=target, report_seq=b.report_seq,
                               detail=b.vehicle_id))
        for msg in out:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.command_seq = command.seq
            if msg.run_id == 0 and self.reporter.run_id:
                msg.run_id = self.reporter.run_id
            self.command_pub.publish(msg)
            self.event('ros_command_out', body=body, seq=msg.command_seq, run_id=msg.run_id,
                       type=msg.command_type, target=msg.target_vehicle)
            log.info('RxCommand seq=%d %s -> /system/mission/command type=%d target=%d run=%d',
                     command.seq, body, msg.command_type, msg.target_vehicle, msg.run_id)

    # ---- ROS -> RoboCommand ----
    def on_heartbeat(self, m):
        self.reporter.mark_fresh(self.vehicle_id)
        self.reporter.update(self.vehicle_id, lat=m.position.latitude, lng=m.position.longitude,
                             spd_mps=m.spd_mps, heading_deg=m.heading_deg, roll_deg=m.roll_deg,
                             pitch_deg=m.pitch_deg, altitude_hae_m=m.altitude_hae_m,
                             depth_m=m.depth_m, state=m.state, current_task=m.current_task)

    def on_status(self, m):
        self.event('ros_status_in', run_id=m.run_id, state=m.state, task=m.current_task,
                   message=m.message)

    def report_once(self, field, body, **info):
        # The declaration seq separates runs even when RoboCommand reuses run_id after a reset.
        key = hashlib.sha256(
            f'{self.reporter.declaration_seq}:{self.reporter.run_id}:{field}:'.encode() +
            body.SerializeToString()).hexdigest()
        with self._lock:
            if key in self.reporter.sent_reports:
                log.info('%s %s already reported for run %s; skipping duplicate',
                         self.vehicle_id, field, self.reporter.run_id)
                self.event('mqtt_report_duplicate_skipped', field=field, **info)
                return
            ok = self.reporter.publish_report(self.vehicle_id, field, body)
            if ok:
                self.reporter.sent_reports.add(key)
                self.reporter.save_state()
        self.event('mqtt_report_out', field=field, ok=ok, **info)
        log.info('%s -> RxReport.%s published=%s', self.vehicle_id, field, ok)

    def report(self, field, body, **info):
        ok = self.reporter.publish_report(self.vehicle_id, field, body)
        self.event('mqtt_report_out', field=field, ok=ok, **info)
        log.info('%s -> RxReport.%s published=%s', self.vehicle_id, field, ok)

    def on_survey(self, m):
        body = rx_reports_pb2.PipelineSurveyReport(
            active_buoy_position=common_pb2.LatLng(latitude=m.active_buoy_position.latitude,
                                                   longitude=m.active_buoy_position.longitude),
            segments=list(m.segments))
        self.report_once('pipeline_survey', body, lat=m.active_buoy_position.latitude,
                         lon=m.active_buoy_position.longitude, segments=list(m.segments))

    def on_delivery(self, m):
        body = rx_reports_pb2.ResourceDeliveryRequest(
            task=m.task, resource_color=m.resource_color,
            delivery_circle_color=m.delivery_circle_color)
        self.report_once('resource_delivery', body, task=m.task)

    def on_incident_ack(self, m):
        self.report('incident_ack', rx_reports_pb2.IncidentAck(command_seq=m.command_seq),
                    command_seq=m.command_seq)

    def on_readiness(self, m):
        self.report('readiness', rx_reports_pb2.ReadinessReport(command_seq=m.command_seq),
                    command_seq=m.command_seq)


TIERS = {'NONE': common_pb2.TIER_NONE, 'CORE': common_pb2.TIER_CORE,
         'ADVANCED': common_pb2.TIER_ADVANCED, 'DISRUPTIVE': common_pb2.TIER_DISRUPTIVE}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--team', default='ITSN')
    parser.add_argument('--vehicle', default='UUV1')
    parser.add_argument('--tiers', default='NONE,DISRUPTIVE,NONE,NONE',
                        help='task1..task4 tiers: NONE, CORE, ADVANCED or DISRUPTIVE')
    parser.add_argument('--state-file', default='gcs_bridge_state.json')
    parser.add_argument('--stale-timeout-s', type=float, default=3.0)
    parser.add_argument('--events', default=None, help='optional JSONL event log')
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(name)s %(levelname)s %(message)s')
    tiers = tuple(TIERS[t.strip().upper()] for t in args.tiers.split(','))
    if len(tiers) != 4:
        parser.error('--tiers needs four values')

    def factory(on_course, on_command):
        return BridgeReporter(args.host, args.team, {args.vehicle: rx_common_pb2.TYPE_UUV}, tiers,
                              [], on_course=on_course, on_command=on_command,
                              state_file=args.state_file, stale_timeout_s=args.stale_timeout_s)

    rclpy.init()
    node = Bridge(factory, args.vehicle, args.events)
    node.reporter.start()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.reporter.stop()


if __name__ == '__main__':
    main()
