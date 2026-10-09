#!/usr/bin/env python3
"""
Verify the UUV bridge with a mock Team GCS and vehicle on ROS domain 42.

Source the ROS and workspace setup files, then run this script with Python.
Use --no-spawn to test an existing bridge configured with the same parameters.
The current rx_msgs schema uses numeric run IDs, current_task/STATE_AUTO, and
command_seq. PipelineSurvey has no run ID; ReadinessReport itself signals ready.
"""

import argparse
from collections import deque
import math
import os
import shutil
import signal
import subprocess
import sys
import time
import zlib

from banyu_msgs.msg import (
    RunLifecycle,
    Task2RingRecord,
    Task2Submission,
    Task2SurveyEvidence,
    VehicleStatus,
    WaterDepth,
)
from banyu_msgs.srv import SealTask2Report, TriggerPipelineSurvey
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rx_msgs.msg import (
    Command,
    Course,
    Heartbeat,
    IncidentAck,
    MissionStatus,
    PipelineSegmentStatus,
    PipelineSurvey,
    ReadinessReport,
    RobotState,
    RxTask,
    VehicleType,
)
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_srvs.srv import Trigger


RUN_ID = 'run-sos-e2e'
OUTBOX_DIR = '/tmp/banyu_mock_test_outbox'
MISSION_START_SERVICE = '/banyu/mission/start'
MISSION_ABORT_SERVICE = '/banyu/mission/abort'
SEAL_SERVICE = '/banyu/mission/seal_task2_report'
SURVEY_SERVICE = '/banyu/communication/trigger_pipeline_survey'
BRIDGE_COMMAND = [
    'ros2', 'run', 'banyu_communication', 'uuv_comm_node', '--ros-args',
    '-p', 'heartbeat_rate_hz:=2.0',
    '-p', 'survey.publish_legacy_copies:=true',
    '-p', 'survey.surface_hold_s:=0.2',
    '-p', f'survey.outbox_path:={OUTBOX_DIR}/reports.outbox',
    '-p', 'anchor_latitude:=33.0',
    '-p', 'anchor_longitude:=-118.0',
    '-p', 'anchor_altitude_hae_m:=4.0',
    '-p', 'active_buoy_latitude:=33.1',
    '-p', 'active_buoy_longitude:=-118.1',
]


class VerificationFailure(RuntimeError):
    """An expected bridge response was missing or incorrect."""


class TeamGcsMock(Node):
    """Provide telemetry and mission services, and verify the bridge's reports."""

    def __init__(self, timeout):
        super().__init__('sos_gcs_mock')
        self.timeout = timeout
        self.bridge_process = None
        self.start_requests = 0
        self.abort_requests = 0
        self.telemetry_enabled = False
        self.evidence = None

        self.lifecycle_epoch = 0
        self.lifecycle_run_id = 0
        self.lifecycle_revision = 1
        self.lifecycle_phase = RunLifecycle.READY
        self.lifecycle_task = RxTask.TASK_NONE
        self.lifecycle_event = RunLifecycle.SNAPSHOT
        self.lifecycle_reason = ''
        self.lifecycle_ready = True
        self.accepted_at = None
        self.sim_depth = 2.0
        self.water_depth_m = 2.0

        self.reports = {}
        self.report_subscriptions = []
        subscriptions = (
            ('heartbeat', Heartbeat, '/system/vehicle/uuv/heartbeat'),
            ('status', MissionStatus, '/system/mission/status'),
            ('survey', PipelineSurvey, '/system/vehicle/uuv/task2/pipeline_survey'),
            ('submission', Task2Submission, '/system/vehicle/uuv/task2/submission'),
            ('ack', IncidentAck, '/system/vehicle/uuv/task4/incident_ack'),
            ('readiness', ReadinessReport, '/system/vehicle/uuv/task4/readiness'),
        )
        for name, message_type, topic in subscriptions:
            messages = deque(maxlen=200)
            self.reports[name] = messages
            qos = (QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                   if name in ('survey', 'submission') else 10)
            self.report_subscriptions.append(self.create_subscription(
                message_type, topic, lambda msg, target=messages: target.append(msg), qos))

        self.command_pub = self.create_publisher(Command, '/system/mission/command', 10)
        self.course_pub = self.create_publisher(Course, '/system/mission/course', 10)
        self.gps_pub = self.create_publisher(
            NavSatFix, '/mavros/global_position/global', qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/mavros/local_position/pose', qos_profile_sensor_data)
        self.velocity_pub = self.create_publisher(
            TwistStamped, '/mavros/local_position/velocity_local', qos_profile_sensor_data)
        self.state_pub = self.create_publisher(State, '/mavros/state', 10)
        self.vehicle_status_pub = self.create_publisher(
            VehicleStatus, '/banyu/vehicle_status', 10)
        self.lifecycle_pub = self.create_publisher(
            RunLifecycle, '/banyu/mission/run_lifecycle',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.evidence_pub = self.create_publisher(
            Task2SurveyEvidence, '/banyu/pipeline/survey_evidence',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.depth_pub = self.create_publisher(
            WaterDepth, '/banyu/vehicle/water_depth', 5)

        self.input_publishers = (
            self.command_pub, self.course_pub, self.gps_pub, self.pose_pub,
            self.velocity_pub, self.state_pub, self.vehicle_status_pub,
            self.lifecycle_pub, self.evidence_pub, self.depth_pub,
        )

        self.start_server = self.create_service(
            Trigger, MISSION_START_SERVICE, self.start_mission)
        self.abort_server = self.create_service(
            Trigger, MISSION_ABORT_SERVICE, self.abort_mission)
        self.seal_server = self.create_service(
            SealTask2Report, SEAL_SERVICE, self.seal_report)
        self.survey_client = self.create_client(TriggerPipelineSurvey, SURVEY_SERVICE)
        self.telemetry_timer = self.create_timer(0.05, self.publish_telemetry)

        run_id_type = Command.get_fields_and_field_types()['run_id']
        self.wire_run_id = (
            zlib.crc32(RUN_ID.encode('utf-8')) if run_id_type == 'uint32' else RUN_ID)
        self.task_field = (
            'task_id' if 'task_id' in MissionStatus.get_fields_and_field_types()
            else 'current_task')
        self.running_state = getattr(MissionStatus, 'STATE_RUNNING', RobotState.STATE_AUTO)
        self.sequence_field = (
            'sequence_id' if 'sequence_id' in Command.get_fields_and_field_types()
            else 'command_seq')

    def start_mission(self, request, response):
        """Accept the vehicle's mission-start request."""
        self.start_requests += 1
        response.success = True
        response.message = 'mock mission started'
        self.get_logger().info('/banyu/mission/start called: mock mission started')
        return response

    def abort_mission(self, request, response):
        """Acknowledge the vehicle's mission-abort request."""
        self.abort_requests += 1
        response.success = True
        response.message = 'mock mission aborted'
        self.get_logger().info('/banyu/mission/abort called: mock mission aborted')
        return response

    def seal_report(self, request, response):
        """Mock seal authorization from banyu_mission."""
        response.success = True
        response.authorization_revision = 1
        response.message = 'AUTHORIZED'
        self.get_logger().info('/banyu/mission/seal_task2_report called: report sealed')
        return response

    def check_running(self):
        """Fail promptly when ROS shuts down or the spawned bridge exits."""
        if not rclpy.ok(context=self.context):
            raise VerificationFailure('ROS context shut down before verification completed')
        if self.bridge_process is not None:
            return_code = self.bridge_process.poll()
            if return_code is not None:
                raise VerificationFailure(f'uuv_comm_node exited with code {return_code}')

    def wait_until(self, predicate, description):
        """Spin with a monotonic deadline so missing responses cannot hang the test."""
        deadline = time.monotonic() + self.timeout
        while True:
            self.check_running()
            if predicate():
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise VerificationFailure(f'Timeout after {self.timeout:g}s: {description}')
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))

    def wait_report(self, name, predicate, description):
        """Wait for a matching report and include the last report on timeout."""
        def find_match():
            return next((msg for msg in self.reports[name] if predicate(msg)), None)

        try:
            self.wait_until(lambda: find_match() is not None, description)
        except VerificationFailure as error:
            latest = self.reports[name][-1] if self.reports[name] else 'none received'
            raise VerificationFailure(f'{error}; latest {name}: {latest}') from error
        return find_match()

    def publish_telemetry(self):
        """Stream vehicle telemetry, course context, and mission lifecycle."""
        if not self.telemetry_enabled:
            return
        stamp = self.get_clock().now().to_msg()
        gps = NavSatFix()
        gps.header.stamp = stamp
        gps.header.frame_id = 'gps'
        gps.status.status = NavSatStatus.STATUS_FIX
        gps.status.service = NavSatStatus.SERVICE_GPS
        gps.latitude, gps.longitude, gps.altitude = 34.0, -119.0, 4.0
        self.gps_pub.publish(gps)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'map'
        pose.pose.position.z = -self.sim_depth
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        velocity = TwistStamped()
        velocity.header = pose.header
        velocity.twist.linear.x = 0.5
        self.velocity_pub.publish(velocity)

        state = State()
        state.header.stamp = stamp
        state.connected = True
        state.armed = True
        state.guided = True
        state.mode = 'GUIDED'
        state.system_status = 4
        self.state_pub.publish(state)

        status = VehicleStatus()
        status.header.stamp = stamp
        status.fcu_connected = True
        status.armed = True
        status.fcu_status_fresh = True
        status.rc_fresh = True
        status.mode = 'GUIDED'
        status.authority = 'AUTONOMY'
        status.gps_valid = True
        status.gps_fresh = True
        self.vehicle_status_pub.publish(status)

        course = Course()
        course.course_id = 'course-a'
        course.pinger_freq_hz = 25000
        self.course_pub.publish(course)

        lifecycle = RunLifecycle()
        lifecycle.header.stamp = stamp
        lifecycle.run_id = self.lifecycle_run_id
        lifecycle.epoch = self.lifecycle_epoch
        lifecycle.revision = self.lifecycle_revision
        lifecycle.authority_instance = 'mock-mission'
        lifecycle.phase = self.lifecycle_phase
        lifecycle.current_task = self.lifecycle_task
        lifecycle.event = self.lifecycle_event
        lifecycle.reason = self.lifecycle_reason
        lifecycle.autonomous_ready = self.lifecycle_ready
        lifecycle.accepted_at = self.accepted_at if self.accepted_at is not None else stamp
        self.lifecycle_pub.publish(lifecycle)

        depth = WaterDepth()
        depth.header.stamp = stamp
        depth.run_id = self.lifecycle_run_id
        depth.epoch = self.lifecycle_epoch
        depth.depth_m = self.water_depth_m
        depth.valid = True
        depth.calibrated = True
        depth.datum = 'water_surface'
        depth.source_id = 'pressure_depth'
        depth.calibration_id = 'calib-1'
        self.depth_pub.publish(depth)

    def send_command(self, command_type, sequence):
        """Address a uniquely sequenced command to the UUV."""
        command = Command()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'team_gcs'
        command.target_vehicle = VehicleType.TYPE_UUV
        command.command_type = command_type
        command.run_id = self.wire_run_id
        setattr(command, self.sequence_field, sequence)
        command.position.latitude, command.position.longitude = 33.1, -118.1
        self.command_pub.publish(command)

    def run_verification(self):
        """Execute the six communication checks in order."""
        log = self.get_logger()
        if self.wire_run_id != RUN_ID:
            log.warning(f'rx_msgs uses uint32 run_id: {RUN_ID} maps to {self.wire_run_id}')
        if self.task_field == 'current_task':
            log.info('MissionStatus uses current_task and RobotState.STATE_AUTO for running')

        log.info('Step 1/6: Verify reception of a UUV heartbeat')
        self.wait_report('heartbeat', lambda msg: msg.vehicle_type == VehicleType.TYPE_UUV,
                         'heartbeat with vehicle_type == TYPE_UUV')
        log.info('Step 1/6 PASS: UUV heartbeat received')

        log.info('Step 2/6: Publish telemetry and verify depth=2.0m, GPS=(34.0, -119.0)')
        self.wait_until(
            lambda: all(pub.get_subscription_count() > 0 for pub in self.input_publishers)
            and all(sub.get_publisher_count() > 0 for sub in self.report_subscriptions),
            'discovery of bridge publishers and subscribers')
        self.reports['heartbeat'].clear()
        self.telemetry_enabled = True
        self.publish_telemetry()
        self.wait_report(
            'heartbeat',
            lambda msg: msg.vehicle_type == VehicleType.TYPE_UUV
            and math.isclose(msg.depth_m, 2.0, abs_tol=1e-3)
            and math.isclose(msg.position.latitude, 34.0, abs_tol=1e-6)
            and math.isclose(msg.position.longitude, -119.0, abs_tol=1e-6)
            and math.isclose(msg.heading_deg, 90.0, abs_tol=1e-3)
            and msg.state == RobotState.STATE_AUTO,
            'heartbeat reflecting simulated depth, coordinates, and automatic control')
        log.info('Step 2/6 PASS: Heartbeat reflects simulated vehicle telemetry')

        log.info(f'Step 3/6: Send CMD_RUN_START for {RUN_ID} and verify mission start')
        self.reports['status'].clear()
        self.send_command(Command.CMD_RUN_START, 1)
        self.accepted_at = self.get_clock().now().to_msg()
        self.lifecycle_run_id = self.wire_run_id
        self.lifecycle_epoch = 1
        self.lifecycle_phase = RunLifecycle.ACTIVE
        self.lifecycle_task = RxTask.TASK_INFRA_SURVEY_REPAIR
        self.lifecycle_event = RunLifecycle.START_ACCEPTED
        self.lifecycle_reason = 'Mission started: mock mission started'
        self.lifecycle_revision += 1
        self.publish_telemetry()
        self.wait_report(
            'status',
            lambda msg: msg.run_id == self.wire_run_id
            and getattr(msg, self.task_field) == RxTask.TASK_INFRA_SURVEY_REPAIR
            and msg.state == self.running_state
            and msg.message == 'Mission started: mock mission started',
            'running infrastructure survey mission status after successful mission start')
        log.info('Step 3/6 PASS: Mission lifecycle updated and running status received')

        log.info('Step 4/6: Publish 4-node ring inspections and trigger the pipeline survey')
        evidence = Task2SurveyEvidence()
        stamp = self.get_clock().now().to_msg()
        evidence.header.stamp = stamp
        evidence.run_id = self.wire_run_id
        evidence.epoch = self.lifecycle_epoch
        evidence.revision = 1
        evidence.source_id = 'pipeline_topology'
        evidence.tier = 2
        evidence.traversal_complete = True
        evidence.low_progress_endpoint_id = 'start'
        evidence.high_progress_endpoint_id = 'end'

        evidence.active_buoy.header.stamp = stamp
        evidence.active_buoy.run_id = self.wire_run_id
        evidence.active_buoy.epoch = self.lifecycle_epoch
        evidence.active_buoy.revision = 1
        evidence.active_buoy.source_id = 'uuv_buoy_localizer'
        evidence.active_buoy.course_id = 'course-a'
        evidence.active_buoy.endpoint_id = 'start'
        evidence.active_buoy.green_confirmed = True
        evidence.active_buoy.pinger_confirmed = True
        evidence.active_buoy.pinger_freq_hz = 25000
        evidence.active_buoy.uncertainty_m = 0.5
        evidence.active_buoy.fix.latitude = 33.1
        evidence.active_buoy.fix.longitude = -118.1
        evidence.active_buoy.fix.status.status = NavSatStatus.STATUS_FIX
        evidence.active_buoy.fix.header.stamp = stamp

        for i in range(4):
            ring = Task2RingRecord()
            ring.run_id = self.wire_run_id
            ring.epoch = self.lifecycle_epoch
            ring.node_id = 101 + i
            ring.frame_sequence = i + 1
            ring.color = 1 if i == 0 else 2
            ring.confidence = 0.95
            ring.physical_identity_verified = True
            ring.ordering_verified = True
            ring.pipeline_progress_m = 10.0 * (i + 1)
            ring.vehicle_pose_map.header.stamp = stamp
            ring.vehicle_pose_map.header.frame_id = 'map'
            ring.vehicle_pose_map.pose.position.x = float(i)
            ring.vehicle_pose_map.pose.position.y = 0.0
            ring.vehicle_pose_map.pose.position.z = -2.0
            ring.vehicle_pose_map.pose.orientation.w = 1.0
            evidence.rings.append(ring)

        self.evidence_pub.publish(evidence)
        self.lifecycle_phase = RunLifecycle.SURFACED
        self.lifecycle_reason = 'CURRENT_SURFACE_ACTION_SUCCEEDED'
        self.lifecycle_revision += 1
        self.sim_depth = 0.0
        self.water_depth_m = 0.05
        self.publish_telemetry()

        hold_deadline = time.monotonic() + 0.5
        self.wait_until(lambda: time.monotonic() >= hold_deadline, 'surface hold duration')
        self.wait_until(self.survey_client.service_is_ready, 'pipeline survey service discovery')

        self.reports['survey'].clear()
        self.reports['status'].clear()
        survey_request = TriggerPipelineSurvey.Request()
        survey_request.run_id = self.wire_run_id
        survey_request.epoch = self.lifecycle_epoch
        survey_request.report_id = 'mock-report-1'
        future = self.survey_client.call_async(survey_request)
        self.wait_until(future.done, 'pipeline survey service response')
        response = future.result()
        if response is None or not response.success:
            raise VerificationFailure(f'Pipeline survey service failed: {response}')

        expected_segments = [
            PipelineSegmentStatus.PIPELINE_SEGMENT_DAMAGED,
            PipelineSegmentStatus.PIPELINE_SEGMENT_INTACT,
            PipelineSegmentStatus.PIPELINE_SEGMENT_INTACT,
            PipelineSegmentStatus.PIPELINE_SEGMENT_INTACT,
        ]
        has_survey_run = 'run_id' in PipelineSurvey.get_fields_and_field_types()
        self.wait_report(
            'survey',
            lambda msg: math.isclose(msg.active_buoy_position.latitude, 33.1, abs_tol=1e-6)
            and math.isclose(msg.active_buoy_position.longitude, -118.1, abs_tol=1e-6)
            and list(msg.segments) == expected_segments
            and (not has_survey_run or msg.run_id == self.wire_run_id),
            'pipeline survey with configured buoy and ordered 4-node segments')
        log.info(
            'Step 4/6 PASS: Survey received with buoy=(33.1, -118.1), segments=[2, 1, 1, 1]')

        log.info('Step 5/6: Send CMD_TASK4_ASSISTANCE and verify acknowledgement and readiness')
        self.reports['ack'].clear()
        self.reports['readiness'].clear()
        sequence = 2
        self.send_command(Command.CMD_TASK4_ASSISTANCE, sequence)
        self.wait_report(
            'ack', lambda msg: getattr(msg, self.sequence_field) == sequence,
            f'incident acknowledgement matching sequence {sequence}')
        has_readiness_status = 'readiness_status' in ReadinessReport.get_fields_and_field_types()
        self.wait_report(
            'readiness',
            lambda msg: getattr(msg, self.sequence_field) == sequence
            and (not has_readiness_status or msg.readiness_status == ReadinessReport.READY),
            f'readiness report matching assistance sequence {sequence}')
        log.info(
            'Step 5/6 PASS: Incident acknowledgement and readiness match the assistance command')

        log.info('Step 6/6: Verify stop commands trigger mission abort and clear the active task')
        stop_commands = [('CMD_MISSION_DONE', Command.CMD_MISSION_DONE)]
        if hasattr(Command, 'CMD_RUN_STOP'):
            stop_commands.insert(0, ('CMD_RUN_STOP', Command.CMD_RUN_STOP))
        for command_name, command_type in stop_commands:
            sequence += 1
            self.reports['status'].clear()
            self.send_command(Command.CMD_RUN_START, sequence)
            self.accepted_at = self.get_clock().now().to_msg()
            self.lifecycle_run_id = self.wire_run_id
            self.lifecycle_epoch += 1
            self.lifecycle_phase = RunLifecycle.ACTIVE
            self.lifecycle_task = RxTask.TASK_INFRA_SURVEY_REPAIR
            self.lifecycle_event = RunLifecycle.START_ACCEPTED
            self.lifecycle_reason = 'Mission started: mock mission started'
            self.lifecycle_revision += 1
            self.publish_telemetry()
            self.wait_report(
                'status',
                lambda msg: msg.run_id == self.wire_run_id
                and getattr(msg, self.task_field) == RxTask.TASK_INFRA_SURVEY_REPAIR
                and msg.message == 'Mission started: mock mission started',
                f'active mission before {command_name}')

            sequence += 1
            self.reports['status'].clear()
            self.reports['heartbeat'].clear()
            self.send_command(command_type, sequence)
            self.lifecycle_phase = RunLifecycle.INTERRUPTED
            self.lifecycle_task = RxTask.TASK_NONE
            self.lifecycle_event = RunLifecycle.TERMINAL
            self.lifecycle_reason = 'Mission stopped'
            self.lifecycle_revision += 1
            self.publish_telemetry()
            self.wait_report(
                'status',
                lambda msg: msg.run_id == self.wire_run_id
                and getattr(msg, self.task_field) == RxTask.TASK_NONE
                and msg.message == 'Mission stopped',
                f'stopped mission status after {command_name}')
            self.wait_report(
                'heartbeat', lambda msg: getattr(msg, self.task_field) == RxTask.TASK_NONE,
                f'heartbeat with no active task after {command_name}')
            log.info(f'{command_name} PASS: Lifecycle updated and active task cleared')
        log.info('Step 6/6 PASS: Supported stop commands trigger mission abort')


def stop_bridge(process):
    """Terminate the ros2 launcher and every child in its process group."""
    def signal_group(signum):
        try:
            os.killpg(process.pid, signum)
        except ProcessLookupError:
            pass

    signal_group(signal.SIGTERM)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        signal_group(signal.SIGKILL)
        process.wait(timeout=5.0)
    finally:
        signal_group(signal.SIGKILL)


def positive_timeout(value):
    """Parse a finite, positive per-check timeout."""
    timeout = float(value)
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise argparse.ArgumentTypeError('timeout must be finite and positive')
    return timeout


def main(argv=None):
    """Run the mock and return zero only when every supported check passes."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-spawn', action='store_true',
                        help='use an existing configured uuv_comm_node on ROS_DOMAIN_ID=42')
    parser.add_argument('--timeout', type=positive_timeout, default=10.0,
                        help='timeout in seconds for each check (default: 10)')
    args = parser.parse_args(argv)
    os.environ['ROS_DOMAIN_ID'] = '42'
    shutil.rmtree(OUTBOX_DIR, ignore_errors=True)
    os.makedirs(OUTBOX_DIR, exist_ok=True)
    node = None
    process = None
    try:
        rclpy.init(args=[])
        node = TeamGcsMock(args.timeout)
        if args.no_spawn:
            node.get_logger().info('Using an existing uuv_comm_node on ROS_DOMAIN_ID=42')
        else:
            node.get_logger().info('Starting uuv_comm_node on ROS_DOMAIN_ID=42')
            process = subprocess.Popen(
                BRIDGE_COMMAND, env=os.environ.copy(), start_new_session=True)
            node.bridge_process = process
        node.run_verification()
        node.get_logger().info('All six Team GCS communication checks passed')
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        print('FAIL: Communication verification interrupted', file=sys.stderr)
        return 1
    except Exception as error:
        print(f'FAIL: {error}', file=sys.stderr)
        return 1
    finally:
        if process is not None:
            stop_bridge(process)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        shutil.rmtree(OUTBOX_DIR, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
