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
import signal
import subprocess
import sys
import time
import zlib

from banyu_msgs.msg import PipelineRingState, VehicleStatus
from banyu_msgs.srv import TriggerPipelineSurvey
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rx_msgs.msg import (
    Command,
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
MISSION_START_SERVICE = '/banyu/mission/start'
SURVEY_SERVICE = '/banyu/communication/trigger_pipeline_survey'
BRIDGE_COMMAND = [
    'ros2', 'run', 'banyu_communication', 'uuv_comm_node', '--ros-args',
    '-p', 'heartbeat_rate_hz:=10.0',
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
        self.telemetry_enabled = False
        self.rings = []
        self.reports = {}
        self.report_subscriptions = []
        subscriptions = (
            ('heartbeat', Heartbeat, '/system/vehicle/uuv/heartbeat'),
            ('status', MissionStatus, '/system/mission/status'),
            ('survey', PipelineSurvey, '/system/vehicle/uuv/task2/pipeline_survey'),
            ('ack', IncidentAck, '/system/vehicle/uuv/task4/incident_ack'),
            ('readiness', ReadinessReport, '/system/vehicle/uuv/task4/readiness'),
        )
        for name, message_type, topic in subscriptions:
            messages = deque(maxlen=200)
            self.reports[name] = messages
            qos = (QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                   if name == 'survey' else 10)
            self.report_subscriptions.append(self.create_subscription(
                message_type, topic, lambda msg, target=messages: target.append(msg), qos))

        self.command_pub = self.create_publisher(Command, '/system/mission/command', 10)
        self.gps_pub = self.create_publisher(
            NavSatFix, '/mavros/global_position/global', qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/mavros/local_position/pose', qos_profile_sensor_data)
        self.velocity_pub = self.create_publisher(
            TwistStamped, '/mavros/local_position/velocity_local', qos_profile_sensor_data)
        self.state_pub = self.create_publisher(State, '/mavros/state', 10)
        self.vehicle_status_pub = self.create_publisher(
            VehicleStatus, '/banyu/vehicle_status', 10)
        self.ring_pub = self.create_publisher(
            PipelineRingState, '/banyu/pipeline/ring_state', 10)
        self.input_publishers = (
            self.command_pub, self.gps_pub, self.pose_pub, self.velocity_pub,
            self.state_pub, self.vehicle_status_pub, self.ring_pub,
        )
        self.start_server = self.create_service(
            Trigger, MISSION_START_SERVICE, self.start_mission)
        self.survey_client = self.create_client(TriggerPipelineSurvey, SURVEY_SERVICE)
        self.telemetry_timer = self.create_timer(0.1, self.publish_telemetry)

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
        """Stream an armed, guided vehicle at two metres depth with a valid GPS fix."""
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
        pose.pose.position.z = -2.0
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
        status.fcu_status_fresh = True
        status.rc_fresh = True
        status.mode = 'GUIDED'
        status.authority = 'AUTONOMY'
        status.gps_valid = True
        status.gps_fresh = True
        self.vehicle_status_pub.publish(status)

        for ring in self.rings:
            ring.header.stamp = stamp
            self.ring_pub.publish(ring)

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
        """Execute the five communication checks in order."""
        log = self.get_logger()
        if self.wire_run_id != RUN_ID:
            log.warning(f'rx_msgs uses uint32 run_id: {RUN_ID} maps to {self.wire_run_id}')
        if self.task_field == 'current_task':
            log.info('MissionStatus uses current_task and RobotState.STATE_AUTO for running')

        log.info('Step 1/5: Verify reception of a UUV heartbeat')
        self.wait_report('heartbeat', lambda msg: msg.vehicle_type == VehicleType.TYPE_UUV,
                         'heartbeat with vehicle_type == TYPE_UUV')
        log.info('Step 1/5 PASS: UUV heartbeat received')

        log.info('Step 2/5: Publish telemetry and verify depth=2.0m, GPS=(34.0, -119.0)')
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
        log.info('Step 2/5 PASS: Heartbeat reflects simulated vehicle telemetry')

        log.info(f'Step 3/5: Send CMD_RUN_START for {RUN_ID} and verify mission start')
        self.wait_until(
            lambda: self.count_clients(MISSION_START_SERVICE) > 0,
            'bridge mission-start client discovery')
        self.reports['status'].clear()
        initial_requests = self.start_requests
        self.send_command(Command.CMD_RUN_START, 1)
        self.wait_until(lambda: self.start_requests > initial_requests,
                        '/banyu/mission/start invocation')
        self.wait_report(
            'status',
            lambda msg: msg.run_id == self.wire_run_id
            and getattr(msg, self.task_field) == RxTask.TASK_INFRA_SURVEY_REPAIR
            and msg.state == self.running_state
            and msg.message == 'Mission started: mock mission started',
            'running infrastructure survey mission status after successful mission start')
        log.info('Step 3/5 PASS: Mission service called and running status received')

        log.info('Step 4/5: Publish ring inspections and trigger the pipeline survey')
        for index, color in ((0, PipelineRingState.COLOR_RED), (1, PipelineRingState.COLOR_BLUE)):
            ring = PipelineRingState()
            ring.header.frame_id = 'map'
            ring.index = index
            ring.color = color
            ring.confidence = 1.0
            self.rings.append(ring)
        self.publish_telemetry()
        # Ring topics and service requests travel independently; allow ingestion first.
        ring_deadline = time.monotonic() + 0.5
        self.wait_until(lambda: time.monotonic() >= ring_deadline, 'ring-state ingestion')
        self.wait_until(self.survey_client.service_is_ready, 'pipeline survey service discovery')
        self.reports['survey'].clear()
        self.reports['status'].clear()
        future = self.survey_client.call_async(TriggerPipelineSurvey.Request())
        self.wait_until(future.done, 'pipeline survey service response')
        response = future.result()
        if response is None or not response.success:
            raise VerificationFailure(f'Pipeline survey service failed: {response}')
        expected_segments = [PipelineSegmentStatus.PIPELINE_SEGMENT_DAMAGED,
                             PipelineSegmentStatus.PIPELINE_SEGMENT_INTACT,
                             PipelineSegmentStatus.PIPELINE_SEGMENT_UNKNOWN,
                             PipelineSegmentStatus.PIPELINE_SEGMENT_UNKNOWN,
                             PipelineSegmentStatus.PIPELINE_SEGMENT_UNKNOWN]
        has_survey_run = 'run_id' in PipelineSurvey.get_fields_and_field_types()
        self.wait_report(
            'survey',
            lambda msg: math.isclose(msg.active_buoy_position.latitude, 33.1, abs_tol=1e-6)
            and math.isclose(msg.active_buoy_position.longitude, -118.1, abs_tol=1e-6)
            and list(msg.segments) == expected_segments
            and (not has_survey_run or msg.run_id == self.wire_run_id),
            'pipeline survey with configured buoy and ordered red/damaged, blue/intact segments')
        if not has_survey_run:
            self.wait_report(
                'status',
                lambda msg: msg.run_id == self.wire_run_id
                and getattr(msg, self.task_field) == RxTask.TASK_NONE
                and msg.message == 'Pipeline survey published',
                'survey publication status carrying the active run ID')
            log.warning('PipelineSurvey has no run_id; checked run ID on survey MissionStatus')
        log.info(
            'Step 4/5 PASS: Survey received with buoy=(33.1, -118.1), segments=[2, 1, 0, 0, 0]')

        log.info('Step 5/5: Send CMD_TASK4_ASSISTANCE and verify acknowledgement and readiness')
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
        if not has_readiness_status:
            log.warning(
                'ReadinessReport has no readiness_status; matched on-station report sequence')
        log.info(
            'Step 5/5 PASS: Incident acknowledgement and readiness match the assistance command')


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
        # The ros2 wrapper can exit before its executable has finished shutting down.
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
        node.get_logger().info('All five Team GCS communication checks passed')
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


if __name__ == '__main__':
    sys.exit(main())
