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
  python3 tools/scripts/por/dead_thruster_por.py --measured-speed 0.42 \\
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
CONFIG = os.path.join(HERE, "..", "..", "..", "banyu_bringup", "config")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--base", default=os.path.join(CONFIG, "por_params_timed_safe.yaml"),
                   help="profile to copy everything else from")
    p.add_argument("--out", default=None,
                   help="output file (default: por_params_dead_thruster_2_3*.yaml); RC or no RC "
                        "is chosen at launch (launch_por.sh --no-rc)")
    p.add_argument("--live-diagonal", choices=["forward-left", "forward-right"], default="forward-left",
                   help="direction the LIVE thruster pair pushes for +forward (2+3 dead -> forward-left)")
    p.add_argument("--no-position", action="store_true",
                   help="vehicle has no position estimate (no DVL/GPS): fly ALT_HOLD and drive the "
                        "sticks over /mavros/rc/override instead of GUIDED setpoints")
    p.add_argument("--rc-takeover-channel", type=int, default=None,
                   help="banyu_vehicle_status.rc_takeover_channel (0-based; -1 = no stick takeover). "
                        "Default -1 for the no-position profile, as set on BANYU")
    p.add_argument("--no-depth-sensor", action="store_true",
                   help="Pixhawk only, no external depth sensor (Bar30): implies --no-position, flies "
                        "STABILIZE (ALT_HOLD needs a water baro), dives open-loop and does the first "
                        "45 deg turn on the IMU")
    p.add_argument("--dive-depth", type=float, default=1.8,
                   help="dive depth (m): gate_depth_m, or the open-loop dive target with --no-position")
    p.add_argument("--dive-time", type=float, default=None,
                   help="--no-position: open-loop dive time (s); default dive-depth / dive-vz + 1.5 s lag")
    p.add_argument("--dive-vz", type=float, default=0.4, help="--no-depth-sensor: dive command (m/s, down)")
    p.add_argument("--depth-trim", type=float, default=0.0,
                   help="--no-depth-sensor: constant down command on every leg (m/s) to cancel buoyancy")
    p.add_argument("--flight-mode", default="ALT_HOLD", choices=["ALT_HOLD", "STABILIZE", "MANUAL"],
                   help="flight mode for --no-position")
    p.add_argument("--rc-channels", default="2,1,3,4",
                   help="--no-position: forward,lateral,throttle,yaw RC channels (1-based, = RCMAP_*)")
    p.add_argument("--rc-gain-xy", type=float, default=300.0, help="PWM per m/s forward/lateral")
    p.add_argument("--rc-gain-z", type=float, default=300.0, help="PWM per m/s up/down")
    p.add_argument("--rc-gain-yaw", type=float, default=None,
                   help="PWM per rad/s yaw (default: computed from --acro-yaw-p)")
    p.add_argument("--rc-yaw-dz", type=float, default=40.0, help="Pixhawk RCx_DZ of the yaw channel (BANYU RC4_DZ 40)")
    p.add_argument("--thr-dz", type=float, default=100.0, help="Pixhawk THR_DZ (BANYU 100)")
    p.add_argument("--js-gain", type=float, default=0.5, help="Pixhawk JS_GAIN_DEFAULT; scales the dead zones")
    p.add_argument("--pilot-speed", type=float, default=200.0, help="Pixhawk PILOT_SPEED (cm/s); ALT_HOLD climb rate at full stick")
    p.add_argument("--acro-yaw-p", type=float, default=3.375,
                   help="Pixhawk ACRO_YAW_P (BANYU: 3.375). Full yaw stick = 4500 cdeg x ACRO_YAW_P "
                        "per s, so the yaw gain is 400 PWM / that rate")
    p.add_argument("--speed-cmd", type=float, default=0.5, help="commanded speed along the diagonal (m/s)")
    p.add_argument("--measured-speed", type=float, default=None,
                   help="real speed along the diagonal at --speed-cmd (m/s); default = --speed-cmd")
    p.add_argument("--start-to-gate", type=float, default=3.2, help="start to gate plane (m)")
    p.add_argument("--gate-to-marker", type=float, default=10.0, help="gate plane to marker (m)")
    p.add_argument("--orbit-radius", type=float, default=2.3, help="orbit radius around the marker (m)")
    p.add_argument("--past-gate", type=float, default=None,
                   help="how far to back up past the gate (m); default 3.0 with --no-position, else 2.0")
    p.add_argument("--dead-time", type=float, default=0.8, help="per-leg start-up lag (s)")
    p.add_argument("--settle-s", type=float, default=None,
                   help="neutral-stick pause before the orbit and before the turn back, so the "
                        "vehicle stops gliding before it spins in place (default 1.5 with "
                        "--no-position, 0 for GUIDED)")
    p.add_argument("--brake-s", type=float, default=None,
                   help="after the orbit, push back along the live diagonal this long to kill the "
                        "orbit glide before the turn back (default 0.5 with --no-position, 0 = off; "
                        "sim: 1.0 s over-corrects)")
    p.add_argument("--coast-s", type=float, default=None,
                   help="seconds the vehicle keeps gliding after a leg (and drifts in the next turn); "
                        "the approach stops short by speed x coast-s. Default 1.2 with --no-position "
                        "(open-loop sticks, sim: 1.4 m at 1.15 m/s), 0 for GUIDED")
    p.add_argument("--dive-turn-deg", type=float, default=45.0, help="yaw during the dive (deg)")
    p.add_argument("--dive-yaw-rate", type=float, default=0.49, help="dive yaw rate (rad/s, timed)")
    p.add_argument("--turn-rate", type=float, default=0.5, help="90 deg turn yaw rate (rad/s)")
    p.add_argument("--turn-cap", type=float, default=1.45,
                   help="IMU stop angle for the 90 deg turns (rad, short of pi/2 to absorb overshoot)")
    p.add_argument("--orbit-cap", type=float, default=6.23,
                   help="IMU stop angle for the orbit (rad, just short of 2*pi)")
    a = p.parse_args()
    if a.no_depth_sensor:
        a.no_position = True
        a.flight_mode = "STABILIZE"
    # ArduSub stick maths (ArduSub/Attitude.cpp, RC_Channel::pwm_to_angle_dz_trim):
    #   yaw rate  = 4500 cdeg * ACRO_YAW_P per s over the 400 - dead-zone PWM past the dead zone
    #   climb     = PILOT_SPEED * (stick - THR_DZ dead zone) in ALT_HOLD
    #   forward / lateral / STABILIZE throttle = thrust fraction, no dead zone (open loop)
    yaw_dz = round(a.rc_yaw_dz * a.js_gain)
    if a.rc_gain_yaw is None:
        full_stick_rate = math.radians(4500.0 * a.acro_yaw_p / 100.0)
        a.rc_gain_yaw = round((400.0 - yaw_dz) / full_stick_rate, 1)
    z_dz, z_gain = 0, a.rc_gain_z
    if a.flight_mode == "ALT_HOLD":
        z_dz = round(a.thr_dz * a.js_gain)
        z_gain = round(400.0 / (a.pilot_speed / 100.0), 1)
    if a.out is None:
        name = "por_params_dead_thruster_2_3" + \
            ("_pixhawk_only" if a.no_depth_sensor else "_no_pos" if a.no_position else "") + ".yaml"
        a.out = os.path.join(CONFIG, name)

    v_cmd = a.speed_cmd
    v_real = a.measured_speed or v_cmd
    comp = round(v_cmd / math.sqrt(2.0), 3)              # per-axis body command
    sy = 1.0 if a.live_diagonal == "forward-left" else -1.0
    # Yaw that points the live diagonal down the course: forward-left needs -45 deg.
    yaw_sign = -sy
    # Orbit yaw rate from the REAL speed: with RC override the speed is open loop, so a
    # rate from the commanded speed would give radius = v_real / omega != orbit_radius.
    omega = v_real / a.orbit_radius
    if a.brake_s is None:
        a.brake_s = 0.5 if (a.no_position or a.no_depth_sensor) else 0.0
    if a.past_gate is None:
        a.past_gate = 3.0 if (a.no_position or a.no_depth_sensor) else 2.0
    if a.settle_s is None:
        a.settle_s = 1.5 if (a.no_position or a.no_depth_sensor) else 0.0
    if a.coast_s is None:
        a.coast_s = 1.2 if (a.no_position or a.no_depth_sensor) else 0.0
    approach_m = a.start_to_gate + a.gate_to_marker - a.orbit_radius - v_real * a.coast_s
    return_m = a.gate_to_marker - a.orbit_radius + a.past_gate

    values = {
        "use_vision": "false", "use_vision_return": "false", "use_sensors": "true",
        "gate_depth_m": a.dive_depth, "gate_depth_tolerance_m": 0.3,
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

    if a.dive_time is None:
        a.dive_time = round(a.dive_depth / a.dive_vz + 1.5, 1)
    if a.no_position:
        # Stick (RC override) output is open-loop, so a timed yaw would not land on 45 deg:
        # dive straight down, then do the 45 deg alignment turn in pass_gate, where it ends
        # on the IMU. ALT_HOLD holds the reached depth itself (Bar30); without a depth
        # sensor (STABILIZE) a constant --depth-trim cancels buoyancy instead. The
        # vehicle's own depth estimate is not used by the legs (no pose -> no rel_alt).
        turn = math.radians(a.dive_turn_deg)
        values.update({
            "use_baro_depth": "false",
            "dive_duration_s": a.dive_time, "dive_speed_z_mps": round(-a.dive_vz, 3),
            "dive_yaw_rate_rad_s": 0.0,
            "timed_pass_gate_duration_s": round(turn / 0.3 + 3.0, 1),
            "timed_pass_gate_speed_x_mps": 0.0, "timed_pass_gate_speed_y_mps": 0.0,
            "timed_pass_gate_speed_z_mps": round(-a.depth_trim, 3),
            "timed_pass_gate_yaw_rate_rad_s": round(yaw_sign * 0.3, 3),
            "timed_pass_gate_target_angle_rad": round(max(0.05, turn - 0.05), 3),
        })
        for seg in ("approach_pole", "circle_turn_1", "circle_turn_2", "circle_turn_3", "return_to_gate"):
            values[f"{seg}_speed_z_mps"] = round(-a.depth_trim, 3)
    if a.settle_s > 0.0:
        # circle_forward_1 runs between turn_1 and the orbit, circle_forward_2 between the
        # orbit and turn_3: hold neutral sticks there so the glide dies out first.
        for seg in ("circle_forward_1", "circle_forward_2"):
            values.update({f"{seg}_duration_s": a.settle_s, f"{seg}_speed_x_mps": 0.0,
                           f"{seg}_speed_y_mps": 0.0, f"{seg}_speed_z_mps": round(-a.depth_trim, 3),
                           f"{seg}_yaw_rate_rad_s": 0.0})
    if a.brake_s > 0.0:
        # The orbit ends still gliding along its tangent (= the live diagonal): brake by
        # pushing the other way along it, then the turn back starts from (nearly) rest.
        values.update({"circle_forward_2_duration_s": a.brake_s,
                       "circle_forward_2_speed_x_mps": -comp,
                       "circle_forward_2_speed_y_mps": round(-sy * comp, 3),
                       "circle_forward_2_yaw_rate_rad_s": 0.0})
    text = open(a.base).read()
    anchor_pg = "      timed_pass_gate_yaw_rate_rad_s: "
    if "timed_pass_gate_target_angle_rad" not in text and anchor_pg in text:
        i = text.index(anchor_pg); j = text.index("\n", i) + 1
        text = text[:i] + text[i:j] + "      timed_pass_gate_target_angle_rad: 0.0\n" + text[j:]
    for key, value in values.items():
        text, n = re.subn(rf"^(\s+{key}:) [^\s#]+", lambda m: f"{m.group(1)} {value}", text,
                          count=1, flags=re.M)
        if n != 1:
            sys.exit(f"{key} not found in {a.base}")
    header = f"""      # DEAD-THRUSTER profile (generated by tools/scripts/por/dead_thruster_por.py)
      # Live horizontal pair pushes {a.live_diagonal}; every leg runs along that diagonal.
      #   dive {a.dive_turn_deg:.0f} deg yaw -> approach {approach_m:.1f} m -> turn 90 (IMU)
      #   -> orbit r={a.orbit_radius} m, stops on IMU at {a.orbit_cap} rad -> turn 90 back
      #   -> back up {return_m:.1f} m through the gate
      # Inputs: speed {v_cmd} m/s cmd, {v_real} m/s measured, start-gate {a.start_to_gate} m,
      # gate-marker {a.gate_to_marker} m. Re-run the script after measuring at the pool.
"""
    # Replace the base profile's description banner (between its two ===== rules).
    rule = "      # ====="
    if text.count(rule) < 2:
        sys.exit("profile description banner (# ===== rules) not found in " + a.base)
    start = text.index("\n", text.index(rule)) + 1
    end = text.index(rule, start)
    text = text[:start] + header + text[end:]
    if a.no_position:
        try:
            fwd, lat, thr, yaw = (int(c) for c in a.rc_channels.split(","))
        except ValueError:
            sys.exit("--rc-channels needs four integers: forward,lateral,throttle,yaw")
        mode = a.flight_mode
        # The POR base profiles already fly ALT_HOLD + rc_override; set the mode in all three nodes.
        for pattern, what in [(r'^(    autonomy_mode: )"[A-Z_]+".*$', "banyu_mission/banyu_vehicle_status"),
                              (r'^(      autonomy_mode: )"[A-Z_]+"', "vehicle_execution_node control")]:
            text, n = re.subn(pattern, lambda m: f'{m.group(1)}"{mode}"', text, flags=re.M)
            expected = 2 if what.startswith("banyu_mission") else 1
            if n != expected:
                sys.exit(f"{what} autonomy_mode not found in {a.base}")
        if not re.search(r'^      output_mode: "rc_override"', text, flags=re.M):
            sys.exit("control.output_mode rc_override not found in " + a.base)
        text, n = re.subn(r"^    rc_override:.*\n(?:      .*\n)+", "", text, count=1, flags=re.M)
        if n != 1:
            sys.exit("vehicle_execution_node rc_override block not found in " + a.base)
        block = ("    rc_override:   # channels must match the Pixhawk RCMAP_* (1-based)\n"
                 f"      channel_forward: {fwd}\n      channel_lateral: {lat}\n"
                 f"      channel_throttle: {thr}\n      channel_yaw: {yaw}\n"
                 f"      gain_xy_pwm_per_mps: {a.rc_gain_xy}\n      gain_z_pwm_per_mps: {z_gain}\n"
                 f"      gain_yaw_pwm_per_rad_s: {a.rc_gain_yaw}\n      max_delta_pwm: 400\n"
                 "      neutral_pwm: 1500\n      deadzone_xy_pwm: 0\n"
                 f"      deadzone_z_pwm: {z_dz}\n      deadzone_yaw_pwm: {yaw_dz}\n")
        anchor_ve = "vehicle_execution_node:\n  ros__parameters:\n"
        if text.count(anchor_ve) != 1:
            sys.exit("vehicle_execution_node section not found in " + a.base)
        text = text.replace(anchor_ve, anchor_ve + block)
    takeover = a.rc_takeover_channel
    if takeover is None and a.no_position:
        takeover = -1
    if takeover is not None:
        text, n = re.subn(r"^(    rc_takeover_channel:) [^\s#]+", lambda m: f"{m.group(1)} {takeover}",
                          text, count=1, flags=re.M)
        if n != 1:
            sys.exit("banyu_vehicle_status.rc_takeover_channel not found in " + a.base)
    if a.no_position:
        text, n = re.subn(r"^(    enable_mock_auto:) [^\s#]+", lambda m: f"{m.group(1)} false",
                          text, count=1, flags=re.M)
        text, n = re.subn(r"^(    fallback_mode:) [^\s#]+", lambda m: f"{m.group(1)} \"DISARM\"",
                          text, count=1, flags=re.M)
    open(a.out, "w").write(text)
    print(f"wrote {a.out}")
    for key in ("dive_duration_s", "approach_pole_duration_s", "circle_turn_2_duration_s",
                "circle_turn_2_yaw_rate_rad_s", "return_to_gate_duration_s"):
        print(f"  {key}: {values[key]}")


if __name__ == "__main__":
    main()
