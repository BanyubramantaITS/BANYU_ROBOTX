#!/usr/bin/env python3
"""Compare a Pixhawk parameter dump with what the BANYU stack expects.

Checks every value in banyu_bringup/config/banyu_defaults.parm, plus hardware facts the
stack relies on (vectored frame, motors on SERVO1-8, Bar30 detected). Takes the file that
QGroundControl saves (Parameters -> Tools -> Save to file) or a plain "NAME VALUE" list.

usage: check_pixhawk_params.py DUMP.params [--defaults banyu_defaults.parm]
Exit code 1 when something must be fixed before an autonomous run.
"""
import argparse
import os
import sys

DEFAULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..",
                        "banyu_bringup", "config", "banyu_defaults.parm")

# Not in banyu_defaults.parm, but the stack depends on them.
HARDWARE = {
    "FRAME_CONFIG": (2, "vectored 6DOF (BlueROV2 Heavy layout)"),
    **{f"SERVO{i}_FUNCTION": (32 + i, f"motor {i}") for i in range(1, 9)},
    # GPS from run_gps_bridge.sh as MAVLink GPS_INPUT (docs/hardware_setup.md section 4)
    "GPS1_TYPE": (14, "MAVLink GPS_INPUT from run_gps_bridge.sh"),
    "SERIAL3_PROTOCOL": (5, "GPS"),
    "GPS_NAVFILTER": (5, "sea dynamic model"),
    "GPS_MIN_ELEV": (10, "10 deg elevation mask"),
    "GPS1_RATE_MS": (200, "5 Hz"),
}


def load(path):
    params = {}
    for line in open(path):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        fields = line.replace(",", " ").split()
        # QGC: sysid compid NAME VALUE TYPE; plain: NAME VALUE
        name, value = (fields[2], fields[3]) if len(fields) >= 5 else (fields[0], fields[1])
        try:
            params[name] = float(value)
        except ValueError:
            pass
    return params


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dump")
    ap.add_argument("--defaults", default=DEFAULTS)
    a = ap.parse_args()

    dump = load(a.dump)
    expected = {k: (v, "banyu_defaults.parm") for k, v in load(a.defaults).items()}
    expected.update(HARDWARE)

    problems = 0
    for name, (want, why) in sorted(expected.items()):
        have = dump.get(name)
        if have is None:
            print(f"MISSING  {name:<18} expected {want:g} ({why})")
            problems += 1
        elif abs(have - want) > 1e-3 * max(1.0, abs(want)):
            print(f"DIFFERS  {name:<18} is {have:g}, expected {want:g} ({why})")
            problems += 1

    if dump.get("BARO2_DEVID", 0) == 0 and dump.get("BARO3_DEVID", 0) == 0:
        print("WARNING  no external baro detected (BARO2/3_DEVID 0): ALT_HOLD needs the Bar30")
    if dump.get("RC8_OPTION", 0) or dump.get("RC6_OPTION", 0):
        print("NOTE     SB (CH6) / SC (CH8) also carry RCx_OPTION functions in ArduSub: "
              f"RC6_OPTION {dump.get('RC6_OPTION', 0):g}, RC8_OPTION {dump.get('RC8_OPTION', 0):g}")

    print("OK: matches what the stack expects" if problems == 0 else f"{problems} parameter(s) to fix")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
