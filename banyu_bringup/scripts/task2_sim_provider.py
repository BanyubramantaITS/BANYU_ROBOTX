#!/usr/bin/env python3
"""
Provide deterministic Task 2 ROS topics without a simulator plugin.

The start-buoy GPS topic is a surface-only fixture matched to the active
MAVROS datum, not a true buoy GPS receiver.
"""

import math

from banyu_msgs.msg import (
    ActiveBuoyFix,
    HydrophoneHeading,
    PipelineEndCandidate,
    PipelineRingState,
    RunLifecycle,
)
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rx_msgs.msg import Course
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String


COLORS = {
    "red": (1.0, 0.0, 0.0, 1.0),
    "green": (0.0, 1.0, 0.0, 1.0),
    "blue": (0.0, 0.0, 1.0, 1.0),
    "yellow": (1.0, 1.0, 0.0, 1.0),
    "orange": (1.0, 0.5, 0.0, 1.0),
    "purple": (0.5, 0.0, 0.5, 1.0),
    "cyan": (0.0, 1.0, 1.0, 1.0),
    "magenta": (1.0, 0.0, 1.0, 1.0),
    "white": (1.0, 1.0, 1.0, 1.0),
    "black": (0.0, 0.0, 0.0, 1.0),
}

RING_COLOR_IDS = {
    "red": PipelineRingState.COLOR_RED,
    "green": PipelineRingState.COLOR_GREEN,
    "blue": PipelineRingState.COLOR_BLUE,
}

CORE_RING_COLORS = {"red", "green"}

PIPELINE_CENTERLINE_POSES = (
    (0.0, 0.13, 1.55),
    (-0.29, 1.03, 1.55),
    (-0.74, 1.84, 1.55),
    (-0.575, 2.68, 1.55),
    (0.0, 3.46, 1.55),
    (0.0, 4.15, 1.55),
)

PIPELINE_WORLD_X = 51.381
PIPELINE_WORLD_Y = 13.8475
PIPELINE_WORLD_Z = -15.0
PIPELINE_WORLD_YAW = 1.570
WGS84_SEMI_MAJOR_M = 6378137.0
WGS84_ECCENTRICITY_SQUARED = 6.6943799901413165e-3


def wrap_angle(angle):
    """Return an angle in the shortest signed range [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(quaternion):
    """Extract ENU yaw from a ROS quaternion."""
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def wgs84_radii(latitude_rad):
    """Return prime-vertical and meridional WGS84 radii at a latitude."""
    sin_latitude = math.sin(latitude_rad)
    denominator = math.sqrt(
        1.0 - WGS84_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude
    )
    radius_n = WGS84_SEMI_MAJOR_M / denominator
    radius_m = WGS84_SEMI_MAJOR_M * (1.0 - WGS84_ECCENTRICITY_SQUARED) / (
        denominator * denominator * denominator
    )
    return radius_n, radius_m


class Task2SimProvider(Node):
    def __init__(self, *args, **kwargs):
        super().__init__("task2_sim_provider", *args, **kwargs)

        # These are ENU map coordinates for the configured Task 2 buoy.
        self.declare_parameter("buoy_x", 51.381)
        self.declare_parameter("buoy_y", 13.8475)
        self.declare_parameter("start_buoy_map_x", 43.95)
        self.declare_parameter("start_buoy_map_y", 13.85)
        self.declare_parameter("buoy_z", 0.0)
        self.declare_parameter("buoy_yaw", 0.0)
        self.declare_parameter("heading_offset", 0.0)
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("pipeline_standoff_m", 1.0)
        self.declare_parameter("surface_depth", -0.4)
        self.declare_parameter("surface_tolerance", 0.5)
        self.declare_parameter("global_pose_max_skew_s", 0.5)
        self.declare_parameter("receipt_pair_timeout_s", 3.0)
        self.declare_parameter("task2_pose_timeout_s", 3.0)
        self.declare_parameter("task2_gps_fix_timeout_s", 1.0)
        self.declare_parameter(
            "ring_colors", ["green", "green", "green", "red"]
        )
        # Simulation ground truth standing in for the not-yet-built buoy localizer and
        # end-of-pipe detector; published only by this sim-only fixture node.
        self.declare_parameter("pipeline_start_map_x", 47.11)
        self.declare_parameter("pipeline_start_map_y", 13.85)
        self.declare_parameter("pipeline_end_map_x", 51.64)
        self.declare_parameter("pipeline_end_map_y", 13.86)
        self.declare_parameter("end_candidate_window_m", 0.8)
        self.declare_parameter("end_candidate_max_lateral_m", 1.0)
        self.declare_parameter("active_buoy_source_id", "uuv_buoy_localizer")
        self.declare_parameter("active_buoy_endpoint_id", "start_buoy")

        self.buoy_x = float(self.get_parameter("buoy_x").value)
        self.buoy_y = float(self.get_parameter("buoy_y").value)
        self.start_buoy_map_x = float(
            self.get_parameter("start_buoy_map_x").value
        )
        self.start_buoy_map_y = float(
            self.get_parameter("start_buoy_map_y").value
        )
        self.buoy_z = float(self.get_parameter("buoy_z").value)
        self.buoy_yaw = float(self.get_parameter("buoy_yaw").value)
        self.heading_offset = float(self.get_parameter("heading_offset").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.pipeline_standoff_m = float(
            self.get_parameter("pipeline_standoff_m").value
        )
        self.surface_depth = float(self.get_parameter("surface_depth").value)
        self.surface_tolerance = float(
            self.get_parameter("surface_tolerance").value
        )
        self.global_pose_max_skew_s = float(
            self.get_parameter("global_pose_max_skew_s").value
        )
        self.receipt_pair_timeout_s = float(
            self.get_parameter("receipt_pair_timeout_s").value
        )
        self.task2_pose_timeout_s = float(
            self.get_parameter("task2_pose_timeout_s").value
        )
        self.task2_gps_fix_timeout_s = float(
            self.get_parameter("task2_gps_fix_timeout_s").value
        )
        self.pipeline_start = (
            float(self.get_parameter("pipeline_start_map_x").value),
            float(self.get_parameter("pipeline_start_map_y").value),
        )
        self.pipeline_end = (
            float(self.get_parameter("pipeline_end_map_x").value),
            float(self.get_parameter("pipeline_end_map_y").value),
        )
        self.end_candidate_window_m = float(
            self.get_parameter("end_candidate_window_m").value
        )
        self.end_candidate_max_lateral_m = float(
            self.get_parameter("end_candidate_max_lateral_m").value
        )
        self.active_buoy_source_id = str(self.get_parameter("active_buoy_source_id").value)
        self.active_buoy_endpoint_id = str(self.get_parameter("active_buoy_endpoint_id").value)
        if math.hypot(self.pipeline_end[0] - self.pipeline_start[0],
                      self.pipeline_end[1] - self.pipeline_start[1]) < 0.5:
            raise ValueError("pipeline start and end must be at least 0.5 m apart")
        if self.end_candidate_window_m <= 0.0 or self.end_candidate_max_lateral_m <= 0.0:
            raise ValueError("end candidate window and lateral limit must be positive")
        self._validate_task2_parameters()
        if publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")
        if self.pipeline_standoff_m < 0.5:
            raise ValueError("pipeline_standoff_m must be at least 0.5 m")
        configured_colors = self.get_parameter("ring_colors").value
        if (
            not isinstance(configured_colors, (list, tuple))
            or len(configured_colors) != 4
        ):
            self.get_logger().fatal(
                "Task 2 ring_colors must contain exactly four entries: "
                "red or green"
            )
            raise ValueError("ring_colors must contain exactly four entries")

        self.ring_colors = []
        for configured_color in configured_colors:
            color = str(configured_color).strip().lower()
            if color not in CORE_RING_COLORS:
                self.get_logger().fatal(
                    f"Invalid Task 2 ring color '{configured_color}'. "
                    "Valid colors: green, red"
                )
                raise ValueError(f"Invalid Task 2 ring color: {configured_color}")
            self.ring_colors.append(color)

        self.pose_pub = self.create_publisher(
            PoseStamped, "/banyu/debug/task2/active_buoy_pose", 10
        )
        self.route_pub = self.create_publisher(
            Path, "/banyu/debug/task2/route", 10
        )
        self.heading_pub = self.create_publisher(
            HydrophoneHeading, "/banyu/hydrophone/relative_heading", 10
        )
        self.ring_state_pub = self.create_publisher(
            PipelineRingState, "/banyu/debug/task2/ring_state", 10
        )
        # Surface-only simulation fixture matching the active MAVROS datum;
        # this is not a true buoy GPS receiver.
        start_buoy_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.start_buoy_fix_pub = self.create_publisher(
            NavSatFix, "/banyu/task2/start_buoy_fix", start_buoy_qos
        )

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.active_buoy_pub = self.create_publisher(
            ActiveBuoyFix, "/banyu/task2/active_buoy_fix", latched
        )
        self.end_candidate_pub = self.create_publisher(
            PipelineEndCandidate, "/banyu/pipeline/end_candidate_state", 10
        )
        self._lifecycle = None
        self._course = None
        self._start_buoy_fix = None
        self._active_buoy_run = None
        self.create_subscription(
            RunLifecycle, "/banyu/mission/run_lifecycle", self._on_lifecycle, latched
        )
        self.create_subscription(Course, "/system/mission/course", self._on_course, latched)

        self._latest_global_fix = None
        self._latest_vehicle_pose = None
        self._global_fix_received = None
        self._vehicle_pose_received = None
        self._last_gate_diagnostic = ""
        self._last_gate_diagnostic_time = 0.0
        self._source_stamp_fallback_logged = False
        self._start_buoy_fix_published = False

        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._on_vehicle_pose,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self._on_global_fix,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String, "/banyu/robocommand", self._on_robo_command, 10
        )
        self.create_timer(1.0 / publish_rate, self._publish_fixture)
        self.get_logger().info(
            "Task 2 sim provider active: start_buoy_fix is reliable/transient-local; "
            f"surface={self.surface_depth:.2f}+-{self.surface_tolerance:.2f} m, "
            f"buoy_map=({self.start_buoy_map_x:.2f}, {self.start_buoy_map_y:.2f})"
        )

    def _publish_fixture(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.buoy_x
        pose.pose.position.y = self.buoy_y
        pose.pose.position.z = self.buoy_z
        pose.pose.orientation.z = math.sin(self.buoy_yaw / 2.0)
        pose.pose.orientation.w = math.cos(self.buoy_yaw / 2.0)
        self.pose_pub.publish(pose)

        for index, color_name in enumerate(self.ring_colors):
            ring_state = PipelineRingState()
            ring_state.header.stamp = pose.header.stamp
            ring_state.header.frame_id = "map"
            ring_state.index = index
            ring_state.color = RING_COLOR_IDS.get(
                color_name, PipelineRingState.COLOR_UNKNOWN
            )
            ring_state.confidence = 1.0
            self.ring_state_pub.publish(ring_state)

        route = Path()
        route.header.stamp = pose.header.stamp
        route.header.frame_id = "map"
        cos_yaw = math.cos(PIPELINE_WORLD_YAW)
        sin_yaw = math.sin(PIPELINE_WORLD_YAW)
        for local_x, local_y, local_z in PIPELINE_CENTERLINE_POSES:
            route_pose = PoseStamped()
            route_pose.header.stamp = pose.header.stamp
            route_pose.header.frame_id = "map"
            route_pose.pose.position.x = (
                PIPELINE_WORLD_X + cos_yaw * local_x - sin_yaw * local_y
            )
            route_pose.pose.position.y = (
                PIPELINE_WORLD_Y + sin_yaw * local_x + cos_yaw * local_y
            )
            route_pose.pose.position.z = (
                PIPELINE_WORLD_Z + local_z + self.pipeline_standoff_m
            )
            route_pose.pose.orientation.w = 1.0
            route.poses.append(route_pose)
        self.route_pub.publish(route)
        self._maybe_publish_start_buoy_fix()
        self._publish_run_fixtures()

    def _on_lifecycle(self, msg):
        self._lifecycle = msg

    def _on_course(self, msg):
        self._course = msg

    def _publish_run_fixtures(self):
        lifecycle = self._lifecycle
        if lifecycle is None or lifecycle.run_id == 0 or lifecycle.phase not in (
                RunLifecycle.ACTIVE, RunLifecycle.SURVEY_COMPLETE):
            return
        run = (lifecycle.run_id, lifecycle.epoch)
        stamp = self.get_clock().now().to_msg()
        if self._start_buoy_fix is not None and self._active_buoy_run != run:
            buoy = ActiveBuoyFix()
            buoy.header.stamp = stamp
            buoy.header.frame_id = "map"
            buoy.run_id, buoy.epoch = run
            buoy.revision = 1
            buoy.source_id = self.active_buoy_source_id
            buoy.course_id = self._course.course_id if self._course else ""
            buoy.endpoint_id = self.active_buoy_endpoint_id
            buoy.fix = self._start_buoy_fix
            buoy.uncertainty_m = 1.0
            buoy.green_confirmed = True
            buoy.pinger_confirmed = False
            buoy.pinger_freq_hz = self._course.pinger_freq_hz if self._course else 0
            self.active_buoy_pub.publish(buoy)
            self._active_buoy_run = run
            self.get_logger().info(
                f"Simulated active buoy fix published for run {run[0]} epoch {run[1]}"
            )
        pose = self._latest_vehicle_pose
        if not self._valid_vehicle_pose(pose):
            return
        candidate = PipelineEndCandidate()
        candidate.header.stamp = stamp
        candidate.run_id, candidate.epoch = run
        candidate.candidate = self._near_pipeline_end(pose.pose.position)
        self.end_candidate_pub.publish(candidate)

    def _near_pipeline_end(self, position):
        axis_x = self.pipeline_end[0] - self.pipeline_start[0]
        axis_y = self.pipeline_end[1] - self.pipeline_start[1]
        length = math.hypot(axis_x, axis_y)
        rel_x = position.x - self.pipeline_start[0]
        rel_y = position.y - self.pipeline_start[1]
        progress = (rel_x * axis_x + rel_y * axis_y) / length
        lateral = abs(rel_x * axis_y - rel_y * axis_x) / length
        return (length - self.end_candidate_window_m <= progress <= length + 0.5 and
                lateral <= self.end_candidate_max_lateral_m)

    def _on_vehicle_pose(self, msg):
        self._latest_vehicle_pose = msg
        self._vehicle_pose_received = self.get_clock().now()

        vehicle_yaw = yaw_from_quaternion(msg.pose.orientation)
        target_bearing = math.atan2(
            self.buoy_y - msg.pose.position.y,
            self.buoy_x - msg.pose.position.x,
        )
        relative_heading = HydrophoneHeading()
        relative_heading.header.stamp = self.get_clock().now().to_msg()
        relative_heading.header.frame_id = "map"
        relative_heading.relative_heading = wrap_angle(
            target_bearing - vehicle_yaw - self.heading_offset
        )
        relative_heading.confidence = 1.0
        relative_heading.valid = True
        self.heading_pub.publish(relative_heading)

    def _on_global_fix(self, msg):
        self._latest_global_fix = msg
        self._global_fix_received = self.get_clock().now()

    def _maybe_publish_start_buoy_fix(self):
        global_fix = self._latest_global_fix
        vehicle_pose = self._latest_vehicle_pose
        if not self._valid_global_fix(global_fix):
            self._diagnose_gate("waiting for a valid global fix")
            return
        if not self._valid_vehicle_pose(vehicle_pose):
            self._diagnose_gate("waiting for a valid map pose")
            return
        if self._global_fix_received is None or self._vehicle_pose_received is None:
            self._diagnose_gate("waiting for global-fix and pose receipt times")
            return
        now = self.get_clock().now()
        if not self._receipt_fresh(
            now, self._global_fix_received, self.task2_gps_fix_timeout_s
        ) or not self._receipt_fresh(
            now, self._vehicle_pose_received, self.task2_pose_timeout_s
        ):
            self._diagnose_gate("global fix or map pose receipt is stale")
            return
        if (
            abs(
                self._global_fix_received.nanoseconds
                - self._vehicle_pose_received.nanoseconds
            )
            > self._receipt_pair_limit() * 1.0e9
        ):
            self._diagnose_gate("global fix and map pose receipts are not synchronized")
            return

        # GPS homing is valid only while the vehicle remains in the configured
        # surface band. Recompute the target from each synchronized pair.
        if (
            abs(vehicle_pose.pose.position.z - self.surface_depth)
            > self.surface_tolerance
        ):
            self._diagnose_gate("vehicle is outside the configured surface band")
            return

        global_stamp = self._stamp_seconds(global_fix.header.stamp)
        pose_stamp = self._stamp_seconds(vehicle_pose.header.stamp)
        now_stamp = float(now.nanoseconds) * 1.0e-9
        global_stamp_available = self._source_stamp_available(global_fix.header.stamp)
        pose_stamp_available = self._source_stamp_available(vehicle_pose.header.stamp)
        if global_stamp_available and (
            now_stamp < global_stamp
            or now_stamp - global_stamp > self.task2_gps_fix_timeout_s
        ):
            self._diagnose_gate("global-fix source timestamp is stale or future-dated")
            return
        if pose_stamp_available and (
            now_stamp < pose_stamp or now_stamp - pose_stamp > self.task2_pose_timeout_s
        ):
            self._diagnose_gate("map-pose source timestamp is stale or future-dated")
            return
        if global_stamp_available and pose_stamp_available and (
            abs(global_stamp - pose_stamp) > self.global_pose_max_skew_s
        ):
            self._diagnose_gate("global-fix and map-pose source timestamps are skewed")
            return

        east_m = self.start_buoy_map_x - vehicle_pose.pose.position.x
        north_m = self.start_buoy_map_y - vehicle_pose.pose.position.y
        latitude_rad = math.radians(global_fix.latitude)
        if abs(math.cos(latitude_rad)) < 1.0e-12:
            return

        # Use the same WGS84 radii of curvature as the mission's local-tangent
        # conversion rather than treating the earth as a sphere.
        buoy_latitude_rad = latitude_rad
        for _ in range(2):
            mean_latitude = (latitude_rad + buoy_latitude_rad) * 0.5
            radius_n, radius_m = wgs84_radii(mean_latitude)
            buoy_latitude_rad = latitude_rad + north_m / radius_m
        mean_latitude = (latitude_rad + buoy_latitude_rad) * 0.5
        radius_n, _ = wgs84_radii(mean_latitude)
        buoy_longitude_rad = math.radians(global_fix.longitude) + east_m / (
            radius_n * math.cos(mean_latitude)
        )
        if not -math.pi / 2.0 <= buoy_latitude_rad <= math.pi / 2.0:
            return

        buoy_latitude = math.degrees(buoy_latitude_rad)
        buoy_longitude = math.degrees(buoy_longitude_rad)
        buoy_longitude = (buoy_longitude + 180.0) % 360.0 - 180.0
        if not math.isfinite(buoy_latitude) or not math.isfinite(buoy_longitude):
            return

        buoy_fix = NavSatFix()
        buoy_fix.header = global_fix.header
        if not global_stamp_available or not pose_stamp_available:
            buoy_fix.header.stamp = now.to_msg()
            if not self._source_stamp_fallback_logged:
                self.get_logger().warning(
                    "Task 2 GPS input source stamp unavailable; using current ROS time "
                    "for /banyu/task2/start_buoy_fix"
                )
                self._source_stamp_fallback_logged = True
        buoy_fix.latitude = buoy_latitude
        buoy_fix.longitude = buoy_longitude
        buoy_fix.altitude = (
            global_fix.altitude if math.isfinite(global_fix.altitude) else 0.0
        )
        buoy_fix.status = global_fix.status
        if self._valid_covariance(global_fix):
            buoy_fix.position_covariance = list(global_fix.position_covariance)
            buoy_fix.position_covariance_type = global_fix.position_covariance_type
        else:
            buoy_fix.position_covariance = [0.0] * 9
            buoy_fix.position_covariance[0] = 1.0
            buoy_fix.position_covariance[4] = 1.0
            buoy_fix.position_covariance[8] = 1.0
            buoy_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.start_buoy_fix_pub.publish(buoy_fix)
        self._start_buoy_fix = buoy_fix
        self._last_gate_diagnostic = ""
        if not self._start_buoy_fix_published:
            self.get_logger().info(
                "Task 2 start-buoy NavSatFix publishing while vehicle is surfaced"
            )
            self._start_buoy_fix_published = True

    def _diagnose_gate(self, reason):
        if reason != self._last_gate_diagnostic:
            self.get_logger().warning(f"Task 2 start-buoy fix waiting: {reason}")
            self._last_gate_diagnostic = reason

    def _validate_task2_parameters(self):
        parameters = {
            "surface_depth": self.surface_depth,
            "surface_tolerance": self.surface_tolerance,
            "global_pose_max_skew_s": self.global_pose_max_skew_s,
            "receipt_pair_timeout_s": self.receipt_pair_timeout_s,
            "task2_pose_timeout_s": self.task2_pose_timeout_s,
            "task2_gps_fix_timeout_s": self.task2_gps_fix_timeout_s,
            "buoy_x": self.buoy_x,
            "buoy_y": self.buoy_y,
            "start_buoy_map_x": self.start_buoy_map_x,
            "start_buoy_map_y": self.start_buoy_map_y,
        }
        if any(not math.isfinite(value) for value in parameters.values()):
            raise ValueError("Task 2 surface, skew, and buoy map parameters must be finite")
        if self.surface_depth > 0.0:
            raise ValueError("surface_depth must be at or below the map surface")
        if self.surface_tolerance < 0.0:
            raise ValueError("surface_tolerance must not be negative")
        if self.global_pose_max_skew_s < 0.0:
            raise ValueError("global_pose_max_skew_s must not be negative")
        if self.receipt_pair_timeout_s <= 0.0:
            raise ValueError("receipt_pair_timeout_s must be greater than zero")
        if self.task2_pose_timeout_s <= 0.0:
            raise ValueError("task2_pose_timeout_s must be greater than zero")
        if self.task2_gps_fix_timeout_s <= 0.0:
            raise ValueError("task2_gps_fix_timeout_s must be greater than zero")

        surface_band_min = self.surface_depth - self.surface_tolerance
        surface_band_max = self.surface_depth + self.surface_tolerance
        if (
            not math.isfinite(surface_band_min)
            or not math.isfinite(surface_band_max)
            or surface_band_min > surface_band_max
        ):
            raise ValueError("surface depth band has invalid bounds")

    @staticmethod
    def _stamp_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    @staticmethod
    def _source_stamp_available(stamp):
        return (
            stamp.sec >= 0
            and stamp.nanosec < 1000000000
            and (stamp.sec != 0 or stamp.nanosec != 0)
        )

    @staticmethod
    def _receipt_fresh(now, received, timeout_s):
        age_s = (now.nanoseconds - received.nanoseconds) * 1.0e-9
        return 0.0 <= age_s <= timeout_s

    def _receipt_pair_limit(self):
        if (
            self._latest_global_fix is not None
            and self._latest_vehicle_pose is not None
            and self._source_stamp_available(self._latest_global_fix.header.stamp)
            and self._source_stamp_available(self._latest_vehicle_pose.header.stamp)
        ):
            return self.global_pose_max_skew_s
        return self.receipt_pair_timeout_s

    @classmethod
    def _valid_global_fix(cls, fix):
        if fix is None:
            return False
        return (
            fix.status.status >= NavSatStatus.STATUS_FIX
            and math.isfinite(fix.latitude)
            and math.isfinite(fix.longitude)
            and -90.0 <= fix.latitude <= 90.0
            and -180.0 <= fix.longitude <= 180.0
        )

    @staticmethod
    def _valid_vehicle_pose(pose):
        if pose is None or pose.header.frame_id != "map":
            return False
        return (
            pose.header.stamp.sec >= 0
            and pose.header.stamp.nanosec < 1000000000
            and math.isfinite(pose.pose.position.x)
            and math.isfinite(pose.pose.position.y)
            and math.isfinite(pose.pose.position.z)
        )

    @staticmethod
    def _valid_covariance(fix):
        if fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            return False
        return all(
            math.isfinite(value)
            and (index not in (0, 4, 8) or value >= 0.0)
            for index, value in enumerate(fix.position_covariance)
        )

    def _on_robo_command(self, msg):
        self.get_logger().info(f"RoboCommand sent: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = Task2SimProvider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
