#!/usr/bin/env python3
"""Publish a live flag for accepted bottom-lock Gazebo DVL samples."""

from gz.msgs10.dvl_tracking_target_pb2 import DVLTrackingTarget
from gz.msgs10.dvl_velocity_tracking_pb2 import DVLVelocityTracking
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class DvlHealth(Node):
    def __init__(self):
        super().__init__("dvl_health")
        self.live_pub = self.create_publisher(Bool, "/banyu/dvl/live", 10)
        self.last_stamp_ns = 0
        self.gz_node = GzNode()
        if not self.gz_node.subscribe(
            DVLVelocityTracking, "/dvl/velocity", self._on_dvl
        ):
            self.get_logger().fatal("Could not subscribe to Gazebo /dvl/velocity")
            raise RuntimeError("Gazebo DVL subscription failed")

    def _on_dvl(self, msg):
        if msg.target.type != DVLTrackingTarget.DVL_TARGET_BOTTOM:
            return

        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nsec
        if stamp_ns <= 0 or stamp_ns <= self.last_stamp_ns:
            return
        self.last_stamp_ns = stamp_ns

        live = Bool()
        live.data = True
        self.live_pub.publish(live)


def main(args=None):
    rclpy.init(args=args)
    node = DvlHealth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
