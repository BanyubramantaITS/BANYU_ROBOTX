#!/usr/bin/env python3
"""Send RoboCommand's Task 4 keep-out commands to the UUV (stands in for the GCS bridge).

Publishes rx_msgs/Command on /system/mission/command, as gcs_ros_bridge.py does for RoboCommand:

  zone         CMD_TASK4_KEEP_OUT_ZONE: a static circle until ALL_CLEAR
  all-clear    CMD_TASK4_ALL_CLEAR
  moving       CMD_TASK4_MOVING_OBJECT once (position, compass heading, speed, radius)
  moving-sim   a moving object updated every --period s along its heading, as RoboCommand would (until Ctrl-C)

The place is --lat/--lon, or --north/--east metres from the UUV's current GPS fix (/mavros/global_position/global).
--target picks the addressed vehicle (uuv|usv|all); a USV-targeted zone is still avoided but not acknowledged.
Acknowledgements (/system/vehicle/uuv/task4/incident_ack) and /banyu/task4/zones are printed.

Examples (UUV domain, after sourcing the workspace):
  ROS_DOMAIN_ID=40 ./tools/scripts/run/task4_command.py zone --north 12 --east 0 --radius 8
  ROS_DOMAIN_ID=40 ./tools/scripts/run/task4_command.py all-clear
  ROS_DOMAIN_ID=40 ./tools/scripts/run/task4_command.py moving-sim --north 40 --east 0 --heading 180 --speed 0.8 --radius 6
"""
import argparse
import math
import sys
import time

EARTH_RADIUS_M = 6378137.0
TARGETS = {'uuv': 2, 'usv': 1, 'all': 0}


def build_parser():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=['zone', 'all-clear', 'moving', 'moving-sim'])
    parser.add_argument('--lat', type=float, help='latitude, degrees')
    parser.add_argument('--lon', type=float, help='longitude, degrees')
    parser.add_argument('--north', type=float, default=None, help='metres north of the UUV fix (instead of --lat/--lon)')
    parser.add_argument('--east', type=float, default=None, help='metres east of the UUV fix')
    parser.add_argument('--radius', type=float, default=8.0, help='keep-out radius, metres (default 8)')
    parser.add_argument('--heading', type=float, default=0.0,
                        help='moving object: compass heading of travel, degrees clockwise from north (default 0)')
    parser.add_argument('--speed', type=float, default=0.5, help='moving object: speed, m/s (default 0.5)')
    parser.add_argument('--period', type=float, default=2.0, help='moving-sim: seconds between updates (default 2)')
    parser.add_argument('--duration', type=float, default=0.0, help='moving-sim: stop after this long (0 = Ctrl-C)')
    parser.add_argument('--target', choices=sorted(TARGETS), default='uuv',
                        help='addressed vehicle (default uuv; usv = avoided but not acknowledged)')
    parser.add_argument('--seq', type=int, default=0,
                        help='first command_seq (default: from the clock, so it never repeats a seq of an earlier run)')
    parser.add_argument('--confirm-s', type=float, default=15.0,
                        help='resend until /banyu/task4/zones reports the command, at most this long (default 15)')
    parser.add_argument('--wait-s', type=float, default=3.0, help='seconds to wait for the acknowledgement (default 3)')
    return parser


def offset_to_latlon(lat, lon, north, east):
    new_lat = lat + math.degrees(north / EARTH_RADIUS_M)
    new_lon = lon + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(lat))))
    return new_lat, new_lon


def main():
    args = build_parser().parse_args()
    if args.command in ('zone', 'moving', 'moving-sim') and args.radius <= 0.0:
        sys.exit('--radius must be positive')
    uses_offset = args.north is not None or args.east is not None
    if args.command != 'all-clear' and not uses_offset and (args.lat is None or args.lon is None):
        sys.exit('give --lat and --lon, or --north/--east from the current fix')

    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rx_msgs.msg import Command, IncidentAck
    from sensor_msgs.msg import NavSatFix
    from banyu_msgs.msg import Task4Zones

    rclpy.init()
    node = rclpy.create_node('task4_command')
    system_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(Command, '/system/mission/command', system_qos)
    state = {'fix': None, 'acks': [], 'zones_seq': None}
    node.create_subscription(NavSatFix, '/mavros/global_position/global',
                             lambda m: state.update(fix=(m.latitude, m.longitude)), qos_profile_sensor_data)
    node.create_subscription(IncidentAck, '/system/vehicle/uuv/task4/incident_ack',
                             lambda m: (state['acks'].append(m.command_seq), print(f'ack for seq {m.command_seq}')), system_qos)

    def on_zones(m):
        state['zones_seq'] = m.last_command_seq
        print(f'zones [{m.last_event}]: static={list(zip(m.static_latitude_deg, m.static_longitude_deg, m.static_radius_m))}'
              f' moving={m.moving_active}')

    node.create_subscription(Task4Zones, '/banyu/task4/zones', on_zones, latched)

    def spin_for(seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    lat, lon = args.lat, args.lon
    if uses_offset:
        deadline = time.time() + 10.0
        while state['fix'] is None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if state['fix'] is None:
            sys.exit('no GPS fix on /mavros/global_position/global; give --lat/--lon instead')
        lat, lon = offset_to_latlon(*state['fix'], args.north or 0.0, args.east or 0.0)

    seq = args.seq or int(time.time()) % 1000000

    def send(kind, **fields):
        nonlocal seq
        msg = Command()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.command_seq = seq
        msg.target_vehicle = TARGETS[args.target]
        msg.command_type = kind
        for key, value in fields.items():
            if key in ('center', 'position'):
                getattr(msg, key).latitude, getattr(msg, key).longitude = value
            else:
                setattr(msg, key, value)
        # The command topic is volatile, so a publish before discovery is lost: repeat (the node drops duplicates) until
        # the zone state reports this seq.
        deadline = time.time() + args.confirm_s
        while state['zones_seq'] != seq and time.time() < deadline and rclpy.ok():
            pub.publish(msg)
            spin_for(0.2)
        if state['zones_seq'] != seq:
            print(f'NOT CONFIRMED: seq {seq} type {kind} did not reach uuv_comm_node within {args.confirm_s:.0f} s', file=sys.stderr)
            state['failed'] = True
        else:
            print(f'confirmed seq {seq} type {kind} target {args.target} {fields}')
        seq += 1

    spin_for(1.0)
    if args.command == 'zone':
        send(Command.CMD_TASK4_KEEP_OUT_ZONE, center=(lat, lon), radius_m=args.radius)
    elif args.command == 'all-clear':
        send(Command.CMD_TASK4_ALL_CLEAR)
    elif args.command == 'moving':
        send(Command.CMD_TASK4_MOVING_OBJECT, position=(lat, lon), radius_m=args.radius,
             heading_deg=args.heading, speed_mps=args.speed)
    else:
        start = time.time()
        heading = math.radians(args.heading)
        try:
            while rclpy.ok() and (args.duration <= 0.0 or time.time() - start < args.duration):
                travelled = args.speed * (time.time() - start)
                now_lat, now_lon = offset_to_latlon(lat, lon, travelled * math.cos(heading), travelled * math.sin(heading))
                send(Command.CMD_TASK4_MOVING_OBJECT, position=(now_lat, now_lon), radius_m=args.radius,
                     heading_deg=args.heading, speed_mps=args.speed)
                spin_for(args.period)
        except KeyboardInterrupt:
            pass
    if args.command != 'moving-sim':
        spin_for(args.wait_s)
    node.destroy_node()
    rclpy.shutdown()
    if state.get('failed'):
        sys.exit(1)


if __name__ == '__main__':
    main()
