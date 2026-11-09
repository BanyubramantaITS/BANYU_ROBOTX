import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    EqualsSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, SetParameter


def _make_camera_node(name, image_topic, info_topic, hardware_params, enable_cameras, target_mode, mode):
    return Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name=name,
        output="screen",
        parameters=[hardware_params],
        # usb_cam does not carry the image_raw remap over to its transport subtopics,
        # so the compressed stream the detectors and recorder use is remapped explicitly.
        remappings=[
            ("image_raw", image_topic),
            ("camera_info", info_topic),
        ] + [
            (f"image_raw/{transport}", f"{image_topic}/{transport}")
            for transport in ("compressed", "compressedDepth", "theora", "zstd")
        ],
        condition=IfCondition(
            AndSubstitution(
                EqualsSubstitution(enable_cameras, "True"),
                EqualsSubstitution(mode, target_mode),
            )
        ),
    )


def _check_single_esp32_owner(context):
    # banyu_teleop, esp32_bridge_node and esp32_trigger_node all open the same ESP32 port.
    owners = [
        name for name in ("enable_teleop", "enable_esp32_bridge", "enable_esp32_trigger")
        if LaunchConfiguration(name).perform(context) == "True"
    ]
    if len(owners) > 1:
        raise RuntimeError(
            "Only one ESP32 serial owner may run, got: " + ", ".join(owners)
            + ". Use enable_teleop:=True with a TX16S, or enable_esp32_trigger:=True without one."
        )
    return []


def generate_launch_description():
    banyu_bringup_share = get_package_share_directory("banyu_bringup")
    banyu_teleop_share = get_package_share_directory("banyu_teleop")

    default_por_params = os.path.join(
        banyu_bringup_share, "config", "por_params_timed_safe.yaml"
    )
    default_task2_params = os.path.join(
        banyu_bringup_share, "config", "task2_params.yaml"
    )
    default_hardware_params = os.path.join(
        banyu_bringup_share, "config", "hardware_params.yaml"
    )
    default_teleop_params = os.path.join(
        banyu_teleop_share, "config", "teleop_rcin.yaml"
    )

    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="por",
        description="Which competition mission to launch on the real AUV: 'por' or 'task2'.",
        choices=["por", "task2"],
    )

    tier_arg = DeclareLaunchArgument(
        name="tier",
        default_value="core",
        description="Task 2 tier the UUV attempts: 'core', 'advanced' or 'disruptive' (mode:=task2 only).",
        choices=["core", "advanced", "disruptive"],
    )

    gamble_red_arg = DeclareLaunchArgument(
        name="gamble_red", default_value="2,3",
        description="sensors:=gamble: RED positions, 1 = closest to the active buoy; '2,3' = one of them at random, '3' = always 3.",
    )
    sensors_arg = DeclareLaunchArgument(
        name="sensors",
        default_value="full",
        description="Task 2 sensor set: 'full' (Bar30 depth, ALT_HOLD), 'baro' (Bar30 + ALT_HOLD, dive stopped by camera, order dead-reckoned) or 'minimal' (bottom camera, IMU/compass, "
        "surface GPS; STABILIZE under water, heave from the camera).",
        choices=["full", "baro", "minimal", "gamble"],
    )

    dive_point_arg = DeclareLaunchArgument(
        name="dive_point",
        default_value="auto",
        description="Task 2 dive point: 'buoy' (inside the start buoy), 'midpoint' (midway between the buoys, back up to the pipe's start, survey forward) or 'auto' (= midpoint).",
        choices=["auto", "buoy", "midpoint"],
    )

    debug_topics_arg = DeclareLaunchArgument(
        name="debug_topics",
        default_value="False",
        description="Task 2: publish the pipeline detector's annotated frame and masks on /banyu/pipeline/debug/*.",
        choices=["True", "False"],
    )

    test_mission_arg = DeclareLaunchArgument(
        name="test_mission",
        default_value="False",
        description="Task 2: fly the test mission (USV waypoint by GPS, dive, timed forward/backward legs, surface, Core report).",
        choices=["True", "False"],
    )

    waypoint_mission_arg = DeclareLaunchArgument(
        name="waypoint_mission",
        default_value="False",
        description="Task 2: fly the waypoint navigation & communication test mission (task2_waypoint_mission.xml).",
        choices=["True", "False"],
    )

    use_hydrophone_arg = DeclareLaunchArgument(
        name="use_hydrophone",
        default_value="False",
        description="Task 2: opt in to the hydrophone (pinger_localization, pinger homing). The competition vehicle has none.",
        choices=["True", "False"],
    )

    use_gps_arg = DeclareLaunchArgument(
        name="use_gps",
        # POR runs GPS-denied; Task 2 needs the surface GPS leg (NavigateSurfaceGps).
        default_value=PythonExpression(
            ["'True' if '", LaunchConfiguration("mode"), "' == 'task2' else 'False'"]
        ),
        description=(
            "Whether GPS is available on the AUV. Default: False for mode:=por (runs "
            "nogps_odom_beacon, which selects EKF3 source set 2: baro + compass), True for "
            "mode:=task2."
        ),
        choices=["True", "False"],
    )

    debug_arg = DeclareLaunchArgument(
        name="debug",
        default_value="False",
        description="Enable OpenCV debug windows (only when a display is attached).",
        choices=["True", "False"],
    )

    enable_teleop_arg = DeclareLaunchArgument(
        name="enable_teleop",
        default_value="True",
        description="Whether to launch banyu_teleop for TX16S RC switches and ESP32 kill relay.",
        choices=["True", "False"],
    )

    enable_mock_auto_arg = DeclareLaunchArgument(
        name="enable_mock_auto",
        default_value="False",
        description="banyu_teleop land-test mode: SB up arms, sets MANUAL and spins all thrusters "
        "slowly. Set False for a real POR run so SB up means AUTONOMOUS for the mission.",
        choices=["True", "False"],
    )

    enable_esp32_bridge_arg = DeclareLaunchArgument(
        name="enable_esp32_bridge",
        default_value="False",
        description="Whether to launch esp32_bridge_node (keep False when enable_teleop is True).",
        choices=["True", "False"],
    )

    enable_esp32_trigger_arg = DeclareLaunchArgument(
        name="enable_esp32_trigger",
        default_value="False",
        description="Whether to launch the ESP32 trigger node.",
        choices=["True", "False"],
    )

    bypass_rc_arg = DeclareLaunchArgument(
        name="bypass_rc",
        default_value="False",
        description="No transmitter: mode comes from the ESP32 trigger, not RC (use with "
        "enable_esp32_trigger:=True).",
        choices=["True", "False"],
    )

    enable_cameras_arg = DeclareLaunchArgument(
        name="enable_cameras",
        default_value="True",
        description="Whether to launch the local usb_cam USB camera driver nodes.",
        choices=["True", "False"],
    )

    record_arg = DeclareLaunchArgument(
        name="record",
        default_value="True",
        description="Record the run to a rosbag under $BANYU_BAG_DIR (default ~/banyu_bags) "
        "for review after the dive, since the link drops underwater.",
        choices=["True", "False"],
    )

    record_images_arg = DeclareLaunchArgument(
        name="record_images",
        default_value="True",
        description="Include compressed camera images in the mission bag (raw images are never recorded).",
        choices=["True", "False"],
    )

    por_params_arg = DeclareLaunchArgument(
        name="por_params_file",
        default_value=default_por_params,
        description="Path to ROS 2 parameter YAML file for POR nodes.",
    )

    task2_params_arg = DeclareLaunchArgument(
        name="task2_params_file",
        default_value=default_task2_params,
        description="Path to ROS 2 parameter YAML file for Task 2 nodes.",
    )

    hardware_params_arg = DeclareLaunchArgument(
        name="hardware_params_file",
        default_value=default_hardware_params,
        description="Path to ROS 2 parameter YAML file for hardware camera/sensor nodes.",
    )

    teleop_params_arg = DeclareLaunchArgument(
        name="teleop_params_file",
        default_value=default_teleop_params,
        description="Path to ROS 2 parameter YAML file for banyu_teleop node.",
    )

    replay_track_arg = DeclareLaunchArgument(
        name="replay_track",
        default_value="latest",
        description=(
            "Teach track for profiles with use_replay: 'latest' (newest .csv in "
            "$BANYU_REPLAY_DIR, default ~/banyu_replays), a file name there, or an absolute path."
        ),
    )

    set_wall_time = SetParameter(name="use_sim_time", value=False)

    mode = LaunchConfiguration("mode")
    enable_cameras = LaunchConfiguration("enable_cameras")
    por_params = LaunchConfiguration("por_params_file")
    task2_params = LaunchConfiguration("task2_params_file")
    hardware_params = LaunchConfiguration("hardware_params_file")

    teleop_params = LaunchConfiguration("teleop_params_file")

    active_params = PythonExpression([
        "'", por_params, "' if '", mode, "' == 'por' else '", task2_params, "'"
    ])

    # Run directly, not via `ros2 run`: launch only signals the wrapper, which would leave the
    # beacon running after shutdown (MAVProxy stays up, so it would keep feeding the Pixhawk).
    nogps_odom_beacon = ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix("banyu_bringup"), "lib", "banyu_bringup", "nogps_odom_beacon.py"
            ),
            "--mavlink",
            "udpin:127.0.0.1:14552",
        ],
        name="nogps_odom_beacon",
        output="screen",
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("use_gps"), "False")
        ),
    )

    # Run the script directly rather than through `ros2 run` so launch's SIGINT reaches it
    # and the bag is closed cleanly; give it time to flush before SIGTERM/SIGKILL.
    mission_recorder = ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix("banyu_bringup"), "lib", "banyu_bringup", "record_mission.sh"
            ),
            mode,
        ],
        name="mission_recorder",
        output="screen",
        additional_env={
            "BANYU_BAG_IMAGES": LaunchConfiguration("record_images"),
            "BANYU_BAG_PARAMS": active_params,
        },
        sigterm_timeout="15",
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("record"), "True")),
    )

    teleop_node = Node(
        package="banyu_teleop",
        executable="banyu_teleop",
        name="banyu_teleop",
        output="screen",
        parameters=[
            teleop_params,
            {"enable_mock_auto": ParameterValue(
                LaunchConfiguration("enable_mock_auto"), value_type=bool)},
        ],
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("enable_teleop"), "True")
        ),
    )

    front_camera_node = _make_camera_node(
        name="front_camera",
        image_topic="/front_camera/image_raw",
        info_topic="/camera/camera_info",
        hardware_params=hardware_params,
        enable_cameras=enable_cameras,
        target_mode="por",
        mode=mode,
    )

    bottom_camera_node = _make_camera_node(
        name="bottom_camera",
        image_topic="/bottom_camera/image_raw",
        info_topic="/bottom_camera/camera_info",
        hardware_params=hardware_params,
        enable_cameras=enable_cameras,
        target_mode="task2",
        mode=mode,
    )

    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(banyu_bringup_share, "launch", "mission.launch.py")
        ),
        launch_arguments={
            "mode": mode,
            "use_sim": "False",
            "use_sim_time": "False",
            "debug": LaunchConfiguration("debug"),
            "debug_topics": LaunchConfiguration("debug_topics"),
            "tier": LaunchConfiguration("tier"),
            "sensors": LaunchConfiguration("sensors"),
            "use_hydrophone": LaunchConfiguration("use_hydrophone"),
            "dive_point": LaunchConfiguration("dive_point"),
            "test_mission": LaunchConfiguration("test_mission"),
            "waypoint_mission": LaunchConfiguration("waypoint_mission"),
            "gamble_red": LaunchConfiguration("gamble_red"),
            "enable_esp32_trigger": LaunchConfiguration("enable_esp32_trigger"),
            "enable_esp32_bridge": LaunchConfiguration("enable_esp32_bridge"),
            "bypass_rc": LaunchConfiguration("bypass_rc"),
            "por_params_file": por_params,
            "task2_params_file": task2_params,
            "replay_track": LaunchConfiguration("replay_track"),
            "hardware_params_file": hardware_params,
        }.items(),
    )

    return LaunchDescription(
        [
            mode_arg,
            tier_arg,
            sensors_arg,
            use_hydrophone_arg,
            dive_point_arg,
            test_mission_arg,
            waypoint_mission_arg,
            debug_topics_arg,
            gamble_red_arg,
            use_gps_arg,
            debug_arg,
            enable_teleop_arg,
            enable_mock_auto_arg,
            enable_esp32_bridge_arg,
            enable_esp32_trigger_arg,
            bypass_rc_arg,
            enable_cameras_arg,
            record_arg,
            record_images_arg,
            por_params_arg,
            task2_params_arg,
            hardware_params_arg,
            teleop_params_arg,
            replay_track_arg,
            set_wall_time,
            OpaqueFunction(function=_check_single_esp32_owner),
            nogps_odom_beacon,
            mission_recorder,
            teleop_node,
            front_camera_node,
            bottom_camera_node,
            mission_launch,
        ]
    )
