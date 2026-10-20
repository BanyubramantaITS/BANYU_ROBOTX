#!/usr/bin/env python3
"""Generate the dead-thruster POR profile from a few measured numbers.

For a vectored BlueROV2 Heavy frame with two diagonal horizontal thrusters dead
(default: 2 front-left + 3 back-right). The live pair can only push along one
body diagonal and still yaw, so every leg is commanded along that diagonal:

  dive        descend and yaw so the live diagonal points down the course
  approach    translate along the diagonal to orbit-radius before the marker
  turn 1      yaw 90 deg (IMU cap) so the diagonal points sideways
  orbit       translate along the diagonal while yawing; ends on IMU (360 deg)
  turn 2      yaw 90 deg back
  return      back up along the diagonal through the gate

Example (pool numbers):
  python3 tools/scripts/dead_thruster_por.py --measured-speed 0.42 \\
      --start-to-gate 3.0 --gate-to-marker 10.0 \\
      --out banyu_bringup/config/por_params_dead_thruster_2_3.yaml

Measure --measured-speed by commanding the approach leg alone for 10 s and
taping the distance travelled along the course.
"""
import argparse
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "..", "..", "banyu_bringup", "config")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--base", default=os.path.join(CONFIG, "por_params_timed_fast.yaml"),
                   help="profile to copy everything else from")
    p.add_argument("--out", default=None,
                   help="output file (default: por_params_dead_thruster_2_3.yaml, or "
                        "..._no_rc.yaml with --no-rc)")
    p.add_argument("--no-rc", action="store_true",
                   help="run without a transmitter: sets banyu_vehicle_status.bypass_rc so the "
                        "mission needs no RC switches (launch with enable_teleop:=False)")
    p.add_argument("--live-diagonal", choices=["forward-left", "forward-right"], default="forward-left",
                   help="direction the LIVE thruster pair pushes for +forward (2+3 dead -> forward-left)")
    p.add_argument("--speed-cmd", type=float, default=0.5, help="commanded speed along the diagonal (m/s)")
    p.add_argument("--measured-speed", type=float, default=None,
                   help="real speed along the diagonal at --speed-cmd (m/s); default = --speed-cmd")
    p.add_argument("--start-to-gate", type=float, default=3.2, help="start to gate plane (m)")
    p.add_argument("--gate-to-marker", type=float, default=10.0, help="gate plane to marker (m)")
    p.add_argument("--orbit-radius", type=float, default=2.3, help="orbit radius around the marker (m)")
    p.add_argument("--past-gate", type=float, default=2.0, help="how far to back up past the gate (m)")
    p.add_argument("--dead-time", type=float, default=0.8, help="per-leg start-up lag (s)")
    p.add_argument("--dive-turn-deg", type=float, default=45.0, help="yaw during the dive (deg)")
    p.add_argument("--dive-yaw-rate", type=float, default=0.49, help="dive yaw rate (rad/s, timed)")
    p.add_argument("--turn-rate", type=float, default=0.5, help="90 deg turn yaw rate (rad/s)")
    p.add_argument("--turn-cap", type=float, default=1.45,
                   help="IMU stop angle for the 90 deg turns (rad, short of pi/2 to absorb overshoot)")
    p.add_argument("--orbit-cap", type=float, default=6.23,
                   help="IMU stop angle for the orbit (rad, just short of 2*pi)")
    a = p.parse_args()
    if a.out is None:
        name = "por_params_dead_thruster_2_3_no_rc.yaml" if a.no_rc else "por_params_dead_thruster_2_3.yaml"
        a.out = os.path.join(CONFIG, name)

    v_cmd = a.speed_cmd
    v_real = a.measured_speed or v_cmd
    comp = round(v_cmd / math.sqrt(2.0), 3)              # per-axis body command
    sy = 1.0 if a.live_diagonal == "forward-left" else -1.0
    # Yaw that points the live diagonal down the course: forward-left needs -45 deg.
    yaw_sign = -sy
    omega = v_cmd / a.orbit_radius
    approach_m = a.start_to_gate + a.gate_to_marker - a.orbit_radius
    return_m = a.gate_to_marker - a.orbit_radius + a.past_gate

    values = {
        "use_vision": "false", "use_vision_return": "false", "use_sensors": "true",
        "dive_duration_s": round(math.radians(a.dive_turn_deg) / a.dive_yaw_rate, 2),
        "dive_speed_x_mps": 0.0, "dive_speed_y_mps": 0.0, "dive_speed_z_mps": -0.6,
        "dive_yaw_rate_rad_s": round(yaw_sign * a.dive_yaw_rate, 3), "max_depth_speed_mps": 0.6,
        "timed_pass_gate_duration_s": 0.0,
        "approach_pole_duration_s": round(approach_m / v_real + a.dead_time, 1),
        "approach_pole_speed_x_mps": comp, "approach_pole_speed_y_mps": round(sy * comp, 3),
        "approach_pole_yaw_rate_rad_s": 0.0,
        "circle_turn_1_duration_s": 4.0, "circle_turn_1_speed_x_mps": 0.0, "circle_turn_1_speed_y_mps": 0.0,
        "circle_turn_1_yaw_rate_rad_s": round(yaw_sign * a.turn_rate, 3),
        "circle_turn_1_target_angle_rad": a.turn_cap,
        "circle_forward_1_duration_s": 0.0,
        "circle_turn_2_duration_s": round(1.15 * 2.0 * math.pi / omega, 1),
        "circle_turn_2_speed_x_mps": comp, "circle_turn_2_speed_y_mps": round(sy * comp, 3),
        "circle_turn_2_yaw_rate_rad_s": round(-yaw_sign * omega, 4),
        "circle_turn_2_target_angle_rad": a.orbit_cap,
        "circle_forward_2_duration_s": 0.0,
        "circle_turn_3_duration_s": 4.0, "circle_turn_3_speed_x_mps": 0.0, "circle_turn_3_speed_y_mps": 0.0,
        "circle_turn_3_yaw_rate_rad_s": round(-yaw_sign * a.turn_rate, 3),
        "circle_turn_3_target_angle_rad": a.turn_cap,
        "circle_forward_3_duration_s": 0.0, "circle_turn_4_duration_s": 0.0, "circle_forward_4_duration_s": 0.0,
        "return_turn_duration_s": 0.0,
        "return_to_gate_duration_s": round(return_m / v_real + a.dead_time + 0.1, 1),
        "return_to_gate_speed_x_mps": -comp, "return_to_gate_speed_y_mps": round(-sy * comp, 3),
        "return_to_gate_yaw_rate_rad_s": 0.0,
    }

    text = open(a.base).read()
    for key, value in values.items():
        text, n = re.subn(rf"^(\s+{key}:) [^\s#]+", lambda m: f"{m.group(1)} {value}", text,
                          count=1, flags=re.M)
        if n != 1:
            sys.exit(f"{key} not found in {a.base}")
    header = f"""      # DEAD-THRUSTER profile (generated by tools/scripts/dead_thruster_por.py)
      # Live horizontal pair pushes {a.live_diagonal}; every leg runs along that diagonal.
      #   dive {a.dive_turn_deg:.0f} deg yaw -> approach {approach_m:.1f} m -> turn 90 (IMU)
      #   -> orbit r={a.orbit_radius} m, stops on IMU at {a.orbit_cap} rad -> turn 90 back
      #   -> back up {return_m:.1f} m through the gate
      # Inputs: speed {v_cmd} m/s cmd, {v_real} m/s measured, start-gate {a.start_to_gate} m,
      # gate-marker {a.gate_to_marker} m. Re-run the script after measuring at the pool.
"""
    start = text.index("      # FAST strafe-box profile")
    end = text.index("      # =====", start)
    text = text[:start] + header + text[end:]
    if a.no_rc:
        anchor = "    kill_switch_timeout_s: 1.0\n"
        if text.count(anchor) != 1:
            sys.exit("banyu_vehicle_status.kill_switch_timeout_s not found in " + a.base)
        text = text.replace(anchor, anchor + "    bypass_rc: true            # no transmitter needed\n"
                            "    bypass_operational_mode: 2  # 2 = autonomous\n")
    open(a.out, "w").write(text)
    print(f"wrote {a.out}")
    for key in ("dive_duration_s", "approach_pole_duration_s", "circle_turn_2_duration_s",
                "circle_turn_2_yaw_rate_rad_s", "return_to_gate_duration_s"):
        print(f"  {key}: {values[key]}")


if __name__ == "__main__":
    main()
