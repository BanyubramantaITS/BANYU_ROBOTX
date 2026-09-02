#!/usr/bin/env python3
"""Provide deterministic Task 2 ROS topics without a simulator plugin."""

import math

from banyu_msgs.msg import HydrophoneHeading, PipelineRingState
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
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


def wrap_angle(angle):
    """Return an angle in the shortest signed range [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(quaternion):
    """Extract ENU yaw from a ROS quaternion."""
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


class Task2SimProvider(Node):
    def __init__(self):
        super().__init__("task2_sim_provider")

        self.declare_parameter("buoy_x", 51.381)
        self.declare_parameter("buoy_y", 13.8475)
        self.declare_parameter("buoy_z", 0.0)
        self.declare_parameter("buoy_yaw", 0.0)
        self.declare_parameter("heading_offset", 0.0)
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("pipeline_standoff_m", 1.0)
        self.declare_parameter(
            "ring_colors", ["green", "green", "green", "red", "green"]
        )

        self.buoy_x = float(self.get_parameter("buoy_x").value)
        self.buoy_y = float(self.get_parameter("buoy_y").value)
        self.buoy_z = float(self.get_parameter("buoy_z").value)
        self.buoy_yaw = float(self.get_parameter("buoy_yaw").value)
        self.heading_offset = float(self.get_parameter("heading_offset").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.pipeline_standoff_m = float(
            self.get_parameter("pipeline_standoff_m").value
        )
        if publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")
        if self.pipeline_standoff_m < 0.5:
            raise ValueError("pipeline_standoff_m must be at least 0.5 m")

        configured_colors = self.get_parameter("ring_colors").value
        if (
            not isinstance(configured_colors, (list, tuple))
            or len(configured_colors) != 5
        ):
            self.get_logger().fatal(
                "Task 2 ring_colors must contain exactly five entries: "
                "red or green"
            )
            raise ValueError("ring_colors must contain exactly five entries")

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
            PoseStamped, "/banyu/active_buoy_pose", 10
        )
        self.route_pub = self.create_publisher(Path, "/banyu/pipeline/route", 10)
        self.heading_pub = self.create_publisher(
            HydrophoneHeading, "/banyu/hydrophone/relative_heading", 10
        )
        self.ring_state_pub = self.create_publisher(
            PipelineRingState, "/banyu/pipeline/ring_state", 10
        )

        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._on_vehicle_pose,
            10,
        )
        self.create_subscription(
            String, "/banyu/robocommand", self._on_robo_command, 10
        )
        self.create_timer(1.0 / publish_rate, self._publish_fixture)

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

    def _on_vehicle_pose(self, msg):
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
