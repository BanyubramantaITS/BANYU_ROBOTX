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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetUseSimTime

# --- RobotX 2026 Task 2 course -------------------------------------------
#
# The props (pipeline, light nodes, buoys) live in main_world.world and the
# damaged light is picked by the banyu::PipelineLights world system. Their
# placement is randomised here per run (launch arg layout_seed): the only fixed
# rule, as in the handbook's Task 2 figure, is that the pipeline lies between
# the two buoys, each buoy a little past its end on the pipe axis.

PIPE_LENGTH = 4.57            # robotx_pipeline origin (start end) to far end, along its +x
PIPE_Z = -15.0                # pipeline model origin sits on the seabed
COURSE_X, COURSE_Y = (-8.0, 8.0), (-8.0, 8.0)   # start-end box; seabed plane is 50 m, spawn <= ~22 m out
BUOY_GAP = (1.5, 2.5)         # buoy distance past its pipe end
BUOY_LATERAL = 0.5            # +- sideways offset of a buoy from the pipe axis
SPAWN_RANGE = (6.0, 12.0)     # ROV spawn distance from the GREEN buoy
SPAWN_Z = -0.1                # at the float line: the mission starts with a surface transit


def random_layout(seed):
    """Pipeline pose, buoy xy, ROV spawn (x, y, z, yaw) for one run."""
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
    """World SDF text with the pipeline and buoy include poses replaced."""
    def set_pose(text, name, pose):
        pat = r"(<name>%s</name>\s*<uri>[^<]*</uri>\s*<pose>)[^<]*(</pose>)" % re.escape(name)
        text, n = re.subn(pat, lambda m: m.group(1) + pose + m.group(2), text)
        assert n == 1, name
        return text

    x, y, z, yaw = layout["pipeline"]
    text = set_pose(template, "pipeline", f"{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.5f}")
    text = set_pose(text, "robobuoy_green", "%.3f %.3f 0 0 0 0" % layout["green"])
    return set_pose(text, "robobuoy_red", "%.3f %.3f 0 0 0 0" % layout["red"])


def self_check():
    """python3 gazebo_main.launch.py --self-check"""
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
        description="Task 2 tier. 'core' spawns the vehicle without the repair probe; it also picks the "
                    "light's post-repair flash pattern.",
        choices=["core", "advanced", "disruptive"],
    )

    set_use_sim_time = SetUseSimTime(
        True, condition=IfCondition(LaunchConfiguration("use_sim_time"))
    )

    _banyu_sim_pkg_share = get_package_share_directory("banyu_simulation")
    _bluerov2_gz_pkg_share = get_package_share_directory("bluerov2_gz")

    _world_file = os.path.join(
        _banyu_sim_pkg_share,
        "worlds",
        "main_world.world",
    )

    _bluerov2_heavy_file = os.path.join(
        _banyu_sim_pkg_share, "models", "bluerov2_task2", "model.xacro"
    )

    # Stock GUI plus a bottom-camera Image Display (turbidity is only visible there).
    _gui_config = os.path.join(_banyu_sim_pkg_share, "worlds", "main_world.gui.config")

    def launch_course(context):
        probe = "false" if LaunchConfiguration("tier").perform(context) == "core" else "true"
        bluerov2_heavy_xml = xacro.process(_bluerov2_heavy_file, mappings={"probe": probe})
        seed_text = LaunchConfiguration("layout_seed").perform(context)
        seed = int(seed_text) if seed_text else random.randrange(1 << 31)
        layout = random_layout(seed)
        with open(_world_file) as f:
            world = layout_world(f.read(), layout)
        world_path = os.path.join(tempfile.gettempdir(), "banyu_main_world.world")
        with open(world_path, "w") as f:
            f.write(world)
        sx, sy, sz, syaw = layout["spawn"]
        return [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="both",
                parameters=[{"robot_description": bluerov2_heavy_xml}],
            ),
            LogInfo(msg=f"[layout] seed {seed}: pipeline {tuple(round(v, 2) for v in layout['pipeline'])}, "
                        f"green {tuple(round(v, 2) for v in layout['green'])}, "
                        f"red {tuple(round(v, 2) for v in layout['red'])}, "
                        f"spawn {tuple(round(v, 2) for v in layout['spawn'])}"),
            ExecuteProcess(
                cmd=["gz", "sim", "-v3", "-r"]
                + (["-s"] if LaunchConfiguration("headless").perform(context) == "True" else ["--gui-config", _gui_config])
                + [world_path],
                additional_env={"BANYU_LIGHT_SEED": LaunchConfiguration("light_seed"),
                                "BANYU_TIER": LaunchConfiguration("tier")},
                output="both",
                condition=IfCondition(LaunchConfiguration("use_sim")),
            ),
            Node(
                executable="create",
                package="ros_gz_sim",
                name="spawn_bluerov2_heavy",
                arguments=["-string", bluerov2_heavy_xml, "-name", "bluerov2_heavy",
                           "-x", str(sx), "-y", str(sy), "-z", str(sz), "-Y", str(syaw)],
                condition=IfCondition(LaunchConfiguration("use_sim")),
            ),
            ExecuteProcess(
                cmd=[
                    "python3", "-u",
                    os.path.join(_banyu_sim_pkg_share, "scripts", "robotx_light_controller.py"),
                    "ocean",
                ],
                additional_env={
                    "BANYU_LIGHT_SEED": LaunchConfiguration("light_seed"),
                    "BANYU_TIER": LaunchConfiguration("tier"),
                },
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_sim")),
            ),
        ]

    gazebo_ros_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/front_camera/image_raw", "/bottom_camera/image_raw"],
        output="screen",
    )

    camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_info_bridge",
        arguments=["/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    # Down-facing echosounder (gpu_lidar cone in model.xacro) -> sensor_msgs/LaserScan.
    sonar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="down_sonar_bridge",
        arguments=["/down_sonar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    # Hydrophone arrival times from the HydrophoneArray gz system (model.xacro).
    hydrophone_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="hydrophone_bridge",
        arguments=["/banyu/pinger/arrivals@ros_gz_interfaces/msg/Float32Array[gz.msgs.Float_V"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim")),
    )

    return LaunchDescription(
        [
            use_sim_arg,
            use_sim_time_arg,
            headless_arg,
            layout_seed_arg,
            light_seed_arg,
            tier_arg,
            set_use_sim_time,
            OpaqueFunction(function=launch_course),
            gazebo_ros_image_bridge,
            camera_info_bridge,
            sonar_bridge,
            hydrophone_bridge,
        ]
    )


if __name__ == "__main__" and "--self-check" in sys.argv:
    self_check()
