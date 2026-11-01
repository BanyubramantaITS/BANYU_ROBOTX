#!/usr/bin/env python3
"""SIMULATION ONLY: FCU sensor faults for Task 2 scenarios, set on ArduSub SITL through /mavros/param/set.

  sim_fcu_faults.py --faults "compass=20,gps_offset=0.3:0.2,gps_drop=40:15"

compass=<deg>       heading off by <deg> (COMPASS_AUTODEC 0, COMPASS_DEC = the auto declination + <deg>)
gps_offset=<e>:<n>  RTK float: the GPS fix off by <e> m east and <n> m north (SIM_GPS1_GLTCH_X/Y, north = X)
gps_drop=<t>:<d>    no GPS fix for <d> s, <t> s after start (SIM_GPS1_ENABLE 0, then 1): a lost fix in the transit
Self-check: --self-check.
"""
import argparse
import math
import sys
import time


def parse_faults(text):
    faults = {}
    for item in filter(None, (t.strip() for t in (text or '').split(','))):
        key, _, value = item.partition('=')
        if key == 'compass':
            faults['compass'] = float(value)
        elif key == 'gps_offset':
            e, n = value.split(':')
            faults['gps_offset'] = (float(e), float(n))
        elif key == 'gps_drop':
            t, d = value.split(':')
            faults['gps_drop'] = (float(t), float(d))
        else:
            raise ValueError(f'unknown fault {key!r}')
    return faults


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--faults', default='')
    ap.add_argument('--self-check', action='store_true')
    args, _ = ap.parse_known_args(argv)
    if args.self_check:
        f = parse_faults('compass=20,gps_offset=0.3:-0.2,gps_drop=40:15')
        assert f == {'compass': 20.0, 'gps_offset': (0.3, -0.2), 'gps_drop': (40.0, 15.0)}
        assert parse_faults('') == {}
        print('sim_fcu_faults self-check OK')
        return 0
    faults = parse_faults(args.faults)
    if not faults:
        return 0

    import rclpy
    from mavros_msgs.srv import ParamSetV2
    from rcl_interfaces.msg import ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters

    rclpy.init()
    node = rclpy.create_node('sim_fcu_faults')
    set_cli = node.create_client(ParamSetV2, '/mavros/param/set')
    get_cli = node.create_client(GetParameters, '/mavros/param/get_parameters')

    def call(cli, req, timeout=5.0):
        if not cli.wait_for_service(timeout_sec=60.0):
            node.get_logger().error('MAVROS param service not up')
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        return fut.result()

    def set_param(name, value):
        req = ParamSetV2.Request()
        req.param_id = name
        req.force_set = False
        if isinstance(value, int):
            req.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
        else:
            req.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        for _ in range(5):
            res = call(set_cli, req)
            if res is not None and res.success:
                node.get_logger().warn(f'scenario fault: {name} = {value}')
                return True
            time.sleep(1.0)
        node.get_logger().error(f'could not set {name}')
        return False

    time.sleep(20.0)  # MAVROS and the parameter list up
    if 'compass' in faults:
        res = call(get_cli, GetParameters.Request(names=['COMPASS_DEC']))
        dec = res.values[0].double_value if res and res.values else 0.0
        set_param('COMPASS_AUTODEC', 0)
        set_param('COMPASS_DEC', dec + math.radians(faults['compass']))
    if 'gps_offset' in faults:
        e, n = faults['gps_offset']
        set_param('SIM_GPS1_GLTCH_X', n)
        set_param('SIM_GPS1_GLTCH_Y', e)
    if 'gps_drop' in faults:
        t, d = faults['gps_drop']
        start = time.time()
        while time.time() - start < max(0.0, t - 20.0):
            rclpy.spin_once(node, timeout_sec=0.5)
        set_param('SIM_GPS1_ENABLE', 0)
        time.sleep(d)
        set_param('SIM_GPS1_ENABLE', 1)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
