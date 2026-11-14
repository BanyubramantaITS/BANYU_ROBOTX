import math
import os
import random
import re
import sys
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node, SetParameter

PIPE_LENGTH = 4.57
PIPE_Z = -15.0
COURSE_X, COURSE_Y = (-8.0, 8.0), (-8.0, 8.0)
BUOY_GAP = (1.5, 2.5)
BUOY_LATERAL = 0.5
SPAWN_RANGE = (6.0, 12.0)
SPAWN_Z = -0.1


def random_layout(seed):
    rng = random.Random(seed)
    x0, y0 = rng.uniform(*COURSE_X), rng.uniform(*COURSE_Y)
    yaw = rng.uniform(-math.pi, math.pi)
    dx, dy, nx, ny = math.cos(yaw), math.sin(yaw), -math.sin(yaw), math.cos(yaw)

    def past(along):
        side = rng.uniform(-BUOY_LATERAL, BUOY_LATERAL)
        return (x0 + along * dx + side * nx, y0 + along * dy + side * ny)

    green = past(-rng.uniform(*BUOY_GAP))
    red = past(PIPE_LENGTH + rng.uniform(*BUOY_GAP))
    while True:
        r, b = rng.uniform(*SPAWN_RANGE), rng.uniform(-math.pi, math.pi)
        spawn = (green[0] + r * math.cos(b), green[1] + r * math.sin(b))
        if math.dist(spawn, red) > 3.0:
            break
    return {
        "pipeline": (x0, y0, PIPE_Z, yaw),
        "green": green,
        "red": red,
        "spawn": (*spawn, SPAWN_Z, rng.uniform(-math.pi, math.pi)),
    }


def layout_world(template, layout):
    def set_pose(text, name, pose):
        pat = r"(<name>%s</name>\s*<uri>[^<]*</uri>\s*<pose>)[^<]*(</pose>)" % re.escape(name)
        text, n = re.subn(pat, lambda m: m.group(1) + pose + m.group(2), text)
        assert n == 1, name
        return text

    x, y, z, yaw = layout["pipeline"]
    text = set_pose(template, "pipeline", f"{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.5f}")
    text = set_pose(text, "robobuoy_green", "%.3f %.3f 0 0 0 0" % layout["green"])
    return set_pose(text, "robobuoy_red", "%.3f %.3f 0 0 0 0" % layout["red"])


NODE_ABOVE_PIPE_ORIGIN = 1.596   # light nodes sit this far above the pipeline model's origin


def set_pipe_depth(template, node_depth_m):
    """Move the pipeline (and the nodes placed relative to it) so the light nodes are node_depth_m below the surface."""
    if not 1.0 <= node_depth_m <= -PIPE_Z - NODE_ABOVE_PIPE_ORIGIN:
        raise ValueError(f"pipe_depth {node_depth_m} m: must be 1 to {-PIPE_Z - NODE_ABOVE_PIPE_ORIGIN:.1f} m")
    pat = r"(<name>pipeline</name>\s*<uri>[^<]*</uri>\s*<pose>)([^<]*)(</pose>)"
    m = re.search(pat, template)
    assert m, "pipeline pose"
    v = m.group(2).split()
    v[2] = f"{-node_depth_m - NODE_ABOVE_PIPE_ORIGIN:.3f}"
    return template[:m.start(2)] + " ".join(v) + template[m.end(2):]


def move_pipe(template, dx, dy, dyaw_deg):
    """Scenario: rotate the pipeline about its origin (its start end) by dyaw_deg, then shift it (m); the buoys stay."""
    pat = r"(<name>pipeline</name>\s*<uri>[^<]*</uri>\s*<pose>)([^<]*)(</pose>)"
    m = re.search(pat, template)
    assert m, "pipeline pose"
    v = [float(x) for x in m.group(2).split()]
    v[0] += dx
    v[1] += dy
    v[5] += math.radians(dyaw_deg)
    return template[:m.start(2)] + " ".join(f"{x:.5f}" for x in v) + template[m.end(2):]


def set_visibility(template, visibility_m):
    """Scenario: scale the turbidity's beam attenuation so the green channel sees visibility_m (contrast 5%: 3 / beta)."""
    m = re.search(r"<attenuation>([^<]*)</attenuation>", template)
    assert m, "attenuation"
    att = [float(x) for x in m.group(1).split()]
    k = (3.0 / visibility_m) / att[1]
    return template[:m.start(1)] + " ".join(f"{a * k:.3f}" for a in att) + template[m.end(1):]


# Barrel distortion (Brown model) standing in for the HORUS T10 fisheye; the detector undistorts with camera_info.
BOTTOM_CAMERA_DISTORTION = (
    "<distortion><k1>-0.28</k1><k2>0.07</k2><k3>0.0</k3><p1>0.0</p1><p2>0.0</p2>"
    "<center>0.5 0.5</center></distortion>")


def set_bottom_camera_width(robot_xml, width):
    """Bottom camera resolution (4:3); the sim default is 320x240 and the real HORUS is 1920x1080."""
    tag = '<camera name="bottom_camera">'
    assert tag in robot_xml, "bottom camera"
    image = f"<image><width>{width}</width><height>{width * 3 // 4}</height></image>"
    return robot_xml.replace(tag, tag + image, 1)


def distort_bottom_camera(robot_xml):
    tag = '<camera name="bottom_camera">'
    assert tag in robot_xml, "bottom camera"
    return robot_xml.replace(tag, tag + BOTTOM_CAMERA_DISTORTION, 1)


# The HORUS T10's IR LED ring as a spot light along the bottom camera's axis (Gazebo cameras look along +X).
IR_LED = (
    '<gazebo reference="bottom_camera_link"><light name="bottom_led" type="spot">'
    "<pose>0 0 0 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><specular>0.1 0.1 0.1 1</specular>"
    "<direction>1 0 0</direction><attenuation><range>6</range><constant>0.3</constant><linear>0.2</linear>"
    "<quadratic>0.05</quadratic></attenuation><spot><inner_angle>0.6</inner_angle><outer_angle>1.0</outer_angle>"
    "<falloff>1.0</falloff></spot><cast_shadows>false</cast_shadows></light></gazebo>")


def add_ir_led(robot_xml):
    assert "</robot>" in robot_xml, "robot"
    return robot_xml.replace("</robot>", IR_LED + "</robot>", 1)


def self_check():
    for seed in range(200):
        lay = random_layout(seed)
        (x0, y0, _, yaw), g, r = lay["pipeline"], lay["green"], lay["red"]
        along = lambda p: (p[0] - x0) * math.cos(yaw) + (p[1] - y0) * math.sin(yaw)
        assert along(g) < 0 and along(r) > PIPE_LENGTH, "pipeline must lie between the buoys"
        assert SPAWN_RANGE[0] - 1e-9 <= math.dist(lay["spawn"][:2], g) <= SPAWN_RANGE[1] + 1e-9
    assert random_layout(7) == random_layout(7)
    tpl = ("<name>pipeline</name><uri>u</uri><pose>0 0 0 0 0 0</pose>"
           "<name>robobuoy_green</name><uri>u</uri><pose>0 0 0 0 0 0</pose>"
           "<name>robobuoy_red</name><uri>u</uri><pose>0 0 0 0 0 0</pose>")
    assert "0 0 0 0 0 0" not in layout_world(tpl, random_layout(1))
    assert "<pose>0 0 -4.596 0 0 0</pose>" in set_pipe_depth(tpl, 3.0)
    assert "<pose>1.00000 -2.00000 0.00000 0.00000 0.00000 0.17453</pose>" in move_pipe(tpl, 1.0, -2.0, 10.0)
    assert "<attenuation>5.143 3.000 4.714</attenuation>" in set_visibility(
        "<attenuation>0.60 0.35 0.55</attenuation>", 1.0)
    print("gazebo_main layout self-check OK")


def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        name="use_sim",
        default_value="True",
        description="Whether to launch the Gazebo simulation environment",
        choices=["True", "False"],
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        choices=["True", "False"],
    )

    headless_arg = DeclareLaunchArgument(
        name="headless",
        default_value="False",
        description="Run the Gazebo server only (no GUI): faster real-time factor for test runs.",
        choices=["True", "False"],
    )

    layout_seed_arg = DeclareLaunchArgument(
        name="layout_seed",
        default_value="",
        description=(
            "Seed for the course layout (pipeline pose, buoys, ROV spawn). Empty "
            "picks a fresh random layout each run; set an integer to reproduce one."
        ),
    )

    light_seed_arg = DeclareLaunchArgument(
        name="light_seed",
        default_value="",
        description=(
            "Seed for choosing which pipeline section is damaged (RED). "
            "Empty picks a fresh random one each run; set an integer to "
            "reproduce a specific pattern."
        ),
    )

    tier_arg = DeclareLaunchArgument(
        name="tier",
        default_value="advanced",
        description="Task 2 tier: 'core', 'advanced', or 'disruptive'.",
        choices=["core", "advanced", "disruptive"],
    )

    pipe_depth_arg = DeclareLaunchArgument(
        name="pipe_depth", default_value="",
        description="Depth (m) of the light nodes below the surface; empty = the world's 13.4 m.")
    pipe_dx_arg = DeclareLaunchArgument(name="pipe_dx", default_value="", description="Scenario: shift the pipe east (m).")
    pipe_dy_arg = DeclareLaunchArgument(name="pipe_dy", default_value="", description="Scenario: shift the pipe north (m).")
    pipe_dyaw_arg = DeclareLaunchArgument(
        name="pipe_dyaw_deg", default_value="", description="Scenario: rotate the pipe about its start end (deg).")
    camera_model_arg = DeclareLaunchArgument(
        name="camera_model", default_value="pinhole", choices=["pinhole", "fisheye"],
        description="Scenario: 'fisheye' adds barrel distortion to the bottom camera (the HORUS T10 is a fisheye).")
    camera_width_arg = DeclareLaunchArgument(
        name="camera_width", default_value="",
        description="Bottom camera width in pixels (4:3); empty = the model's 320x240.")
    ir_led_arg = DeclareLaunchArgument(
        name="ir_led", default_value="off", choices=["on", "off"],
        description="Scenario: 'on' adds the camera's LED ring as a spot light along the bottom camera.")
    use_hydrophone_arg = DeclareLaunchArgument(
        name="use_hydrophone", default_value="False", choices=["True", "False"],
        description="Add the sim HydrophoneArray plugin and its bridge (the competition vehicle has no hydrophone).")
    light_fault_arg = DeclareLaunchArgument(
        name="light_fault", default_value="",
        description="Scenario: a functional node fault '<node>:thermistor|mag|hot' (blinks RED or goes dark).")
    visibility_arg = DeclareLaunchArgument(
        name="visibility_m", default_value="", description="Scenario: water visibility (m); empty = the world's.")
    spawn_x_arg = DeclareLaunchArgument(name="spawn_x", default_value="0")
    spawn_y_arg = DeclareLaunchArgument(name="spawn_y", default_value="0")
    spawn_z_arg = DeclareLaunchArgument(name="spawn_z", default_value="-0.06")
    spawn_yaw_arg = DeclareLaunchArgument(name="spawn_yaw", default_value="0", description="Spawn heading in rad.")

    set_use_sim_time = SetParameter(
        name="use_sim_time",
        value=True,
        condition=IfCondition(
            AndSubstitution(
                EqualsSubstitution(LaunchConfiguration("use_sim"), "True"),
                EqualsSubstitution(LaunchConfiguration("use_sim_time"), "True"),
            )
        ),
    )

    _banyu_sim_pkg_share = get_package_share_directory("banyu_simulation")

    _world_file = os.path.join(
        _banyu_sim_pkg_share,
        "worlds",
        "main_world.world",
    )

    _bluerov2_heavy_file = os.path.join(
        _banyu_sim_pkg_share, "models", "bluerov2_task2", "model.xacro"
    )

    _gui_config = os.path.join(_banyu_sim_pkg_share, "worlds", "main_world.gui.config")

    def launch_course(context):
        tier_val = LaunchConfiguration("tier").perform(context)
        probe = "false" if tier_val == "core" else "true"
        hydrophone = "true" if LaunchConfiguration("use_hydrophone").perform(context) == "True" else "false"
        bluerov2_heavy_xml = xacro.process(
            _bluerov2_heavy_file, mappings={"probe": probe, "hydrophone": hydrophone})
        if LaunchConfiguration("camera_model").perform(context) == "fisheye":
            bluerov2_heavy_xml = distort_bottom_camera(bluerov2_heavy_xml)
        camera_width = LaunchConfiguration("camera_width").perform(context)
        if camera_width:
            bluerov2_heavy_xml = set_bottom_camera_width(bluerov2_heavy_xml, int(camera_width))
        if LaunchConfiguration("ir_led").perform(context) == "on":
            bluerov2_heavy_xml = add_ir_led(bluerov2_heavy_xml)

        seed_text = LaunchConfiguration("layout_seed").perform(context)
        if seed_text:
            seed = int(seed_text)
            layout = random_layout(seed)
            with open(_world_file) as f:
                world = layout_world(f.read(), layout)
            world_path = os.path.join(tempfile.gettempdir(), "banyu_main_world.world")
            with open(world_path, "w") as f:
                f.write(world)
            sx, sy, sz, syaw = layout["spawn"]
            info_msg = (
                f"[layout] seed {seed}: pipeline {tuple(round(v, 2) for v in layout['pipeline'])}, "
                f"green {tuple(round(v, 2) for v in layout['green'])}, "
                f"red {tuple(round(v, 2) for v in layout['red'])}, "
                f"spawn {tuple(round(v, 2) for v in layout['spawn'])}"
            )
        else:
            world_path = _world_file
            pipe_depth = LaunchConfiguration("pipe_depth").perform(context)
            pipe_move = [float(LaunchConfiguration(n).perform(context) or 0.0)
                         for n in ("pipe_dx", "pipe_dy", "pipe_dyaw_deg")]
            visibility = LaunchConfiguration("visibility_m").perform(context)
            if pipe_depth or any(pipe_move) or visibility:
                with open(_world_file) as f:
                    world = f.read()
                if pipe_depth:
                    world = set_pipe_depth(world, float(pipe_depth))
                if any(pipe_move):
                    world = move_pipe(world, *pipe_move)
                if visibility:
                    world = set_visibility(world, float(visibility))
                world_path = os.path.join(tempfile.gettempdir(), "banyu_main_world.world")
                with open(world_path, "w") as f:
                    f.write(world)
            sx = float(LaunchConfiguration("spawn_x").perform(context))
            sy = float(LaunchConfiguration("spawn_y").perform(context))
            sz = float(LaunchConfiguration("spawn_z").perform(context))
            syaw = float(LaunchConfiguration("spawn_yaw").perform(context))
            info_msg = (f"[layout] using world {world_path} with spawn ({sx}, {sy}, {sz}, yaw {syaw})"
                        + (f", light nodes at {pipe_depth} m" if pipe_depth else "")
                        + (f", pipe moved {pipe_move}" if any(pipe_move) else "")
                        + (f", visibility {visibility} m" if visibility else ""))

        return [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="both",
                parameters=[{"robot_description": bluerov2_heavy_xml}],
                condition=IfCondition(LaunchConfiguration("use_sim")),
            ),
            LogInfo(
                msg=info_msg,
                condition=IfCondition(LaunchConfiguration("use_sim")),
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-v3", "-r"]
                + (["-s"] if LaunchConfiguration("headless").perform(context) == "True" else ["--gui-config", _gui_config])
                + [world_path],
                additional_env={
                    "BANYU_LIGHT_SEED": LaunchConfiguration("light_seed").perform(context),
                    "BANYU_LIGHT_FAULT": LaunchConfiguration("light_fault").perform(context),
                    "BANYU_TIER": tier_val,
                },
                output="both",
                condition=IfCondition(LaunchConfiguration("use_sim")),
            ),
            Node(
                executable="create",
                package="ros_gz_sim",
                name="spawn_bluerov2_heavy",
                arguments=[
                    "-string", bluerov2_heavy_xml,
                    "-name", "bluerov2_heavy",
                    "-x", str(sx),
                    "-y", str(sy),
                    "-z", str(sz),
                    "-Y", str(syaw),
                ],
                condition=IfCondition(LaunchConfiguration("use_sim")),
            ),
            ExecuteProcess(
                cmd=[
                    "python3", "-u",
                    os.path.join(_banyu_sim_pkg_share, "scripts", "robotx_light_controller.py"),
                    "ocean",
                ],
                additional_env={
                    "BANYU_LIGHT_SEED": LaunchConfiguration("light_seed").perform(context),
                    "BANYU_LIGHT_FAULT": LaunchConfiguration("light_fault").perform(context),
                    "BANYU_TIER": tier_val,
                },
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_sim")),
            ),
        ]

    gazebo_ros_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/front_camera/image_raw"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    bottom_camera_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/bottom_camera/image_raw"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_info_bridge",
        arguments=["/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    bottom_camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bottom_camera_info_bridge",
        arguments=[
            "/bottom_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    sonar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="down_sonar_bridge",
        arguments=["/down_sonar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    hydrophone_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="hydrophone_bridge",
        arguments=["/banyu/pinger/arrivals@ros_gz_interfaces/msg/Float32Array[gz.msgs.Float_V"],
        output="screen",
        condition=IfCondition(
            AndSubstitution(LaunchConfiguration("use_sim"), LaunchConfiguration("use_hydrophone"))),
    )

    return LaunchDescription(
        [
            use_sim_arg,
            use_sim_time_arg,
            headless_arg,
            layout_seed_arg,
            light_seed_arg,
            tier_arg,
            pipe_depth_arg,
            pipe_dx_arg,
            pipe_dy_arg,
            pipe_dyaw_arg,
            visibility_arg,
            light_fault_arg,
            camera_model_arg,
            ir_led_arg,
            use_hydrophone_arg,
            camera_width_arg,
            spawn_x_arg,
            spawn_y_arg,
            spawn_z_arg,
            spawn_yaw_arg,
            set_use_sim_time,
            clock_bridge,
            OpaqueFunction(function=launch_course),
            gazebo_ros_image_bridge,
            bottom_camera_image_bridge,
            camera_info_bridge,
            bottom_camera_info_bridge,
            sonar_bridge,
            hydrophone_bridge,
        ]
    )


if __name__ == "__main__" and "--self-check" in sys.argv:
    self_check()
