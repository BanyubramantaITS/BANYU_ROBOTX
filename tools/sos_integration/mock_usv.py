#!/usr/bin/env python3
"""Stand-in for the USV's Task 2 buoy feed: rx_msgs/PipelineBuoys at about 1 Hz.

Publishes /system/vehicle/usv/task2/pipeline_buoys (reliable, transient_local, depth 1) the way
the USV team's README describes it: buoy A first (buoy_b_valid false) when --second-after-s is
set, then both buoys; seq increases only when the content changes. course_id is copied from the
latched /system/mission/course unless --course-id is given.

Run it in the GCS domain (the relay forwards it to the UUV) or directly in the UUV domain:
  ROS_DOMAIN_ID=10 python3 mock_usv.py --a 33.810438 -118.393392 --b 33.810438 -118.393272 \\
      --active a --second-after-s 5
"""
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rx_msgs.msg import Course, LatLng, PipelineBuoys, VehicleType

LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
ACTIVE = {'unknown': PipelineBuoys.ACTIVE_UNKNOWN, 'a': PipelineBuoys.ACTIVE_A,
          'b': PipelineBuoys.ACTIVE_B}


class MockUsv(Node):
    def __init__(self, args):
        super().__init__('mock_usv_pipeline_buoys')
        self.args = args
        self.course_id = args.course_id
        self.seq = args.first_seq - 1
        self.sent = None
        self.started = self.get_clock().now()
        self.pub = self.create_publisher(
            PipelineBuoys, '/system/vehicle/usv/task2/pipeline_buoys', LATEST)
        self.create_subscription(Course, '/system/mission/course', self.on_course, LATEST)
        self.create_timer(1.0 / args.rate_hz, self.tick)

    def on_course(self, msg):
        if not self.args.course_id and msg.course_id != self.course_id:
            self.course_id = msg.course_id
            self.get_logger().info(f'course {msg.course_id}')

    def tick(self):
        if not self.course_id:
            return
        elapsed = (self.get_clock().now() - self.started).nanoseconds * 1e-9
        both = self.args.b is not None and elapsed >= self.args.second_after_s
        active = ACTIVE[self.args.active]
        if not both and active == PipelineBuoys.ACTIVE_B:
            active = PipelineBuoys.ACTIVE_UNKNOWN
        content = (self.course_id, tuple(self.args.a), both, active)
        if content != self.sent:
            self.seq += 1
            self.sent = content
            self.get_logger().info(
                f'seq={self.seq} course={self.course_id} '
                f"{'both buoys' if both else 'buoy A only'} active={self.args.active}")
        msg = PipelineBuoys()
        if not self.args.no_stamp:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.course_id = self.course_id
        msg.source_vehicle = VehicleType.TYPE_USV
        msg.seq = self.seq
        msg.buoy_a = LatLng(latitude=self.args.a[0], longitude=self.args.a[1])
        if both:
            msg.buoy_b = LatLng(latitude=self.args.b[0], longitude=self.args.b[1])
        msg.buoy_b_valid = both
        msg.active_buoy = active
        msg.uncertainty_m = self.args.uncertainty_m
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--a', type=float, nargs=2, required=True, metavar=('LAT', 'LON'))
    parser.add_argument('--b', type=float, nargs=2, metavar=('LAT', 'LON'))
    parser.add_argument('--active', choices=sorted(ACTIVE), default='a')
    parser.add_argument('--second-after-s', type=float, default=0.0,
                        help='publish buoy A alone for this long before both buoys')
    parser.add_argument('--course-id', default='', help='default: copy /system/mission/course')
    parser.add_argument('--first-seq', type=int, default=1)
    parser.add_argument('--uncertainty-m', type=float, default=2.0, help='0 = unknown')
    parser.add_argument('--no-stamp', action='store_true', help='leave header.stamp zero')
    parser.add_argument('--rate-hz', type=float, default=1.0)
    args = parser.parse_args()
    rclpy.init()
    try:
        rclpy.spin(MockUsv(args))
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
