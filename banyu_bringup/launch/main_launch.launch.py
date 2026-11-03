import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    EqualsSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    _banyu_bringup_share = get_package_share_directory("banyu_bringup")

    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="por",
        description="Which competition simulation to launch: 'por' or 'task2'.",
        choices=["por", "task2"],
    )

    use_sim_arg = DeclareLaunchArgument(
        name="use_sim",
        default_value="True",
        description="Whether to run in simulation mode (Gazebo + SITL) or real ROV hardware mode",
        choices=["True", "False"],
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        choices=["True", "False"],
    )

    use_ardusub_sitl_arg = DeclareLaunchArgument(
        name="use_ardusub_sitl",
        default_value="True",
        description="Whether to start the ArduSub SITL binary alongside Gazebo",
        choices=["True", "False"],
    )

    tier_arg = DeclareLaunchArgument(
        name="tier",
        default_value="advanced",
        description="RobotX Task 2 tier: 'core' (survey + report, no probe), 'advanced' (repair + resource request) "
                    "or 'disruptive' (as advanced, and the tin colour).",
        choices=["core", "advanced", "disruptive"],
    )

    debug_arg = DeclareLaunchArgument(
        name="debug",
        default_value="False",
        description="Set the debug flag for example (Detection Windows)",
        choices=["True", "False"],
    )

    use_gps_arg = DeclareLaunchArgument(
        name="use_gps",
        default_value="True",
        description="False removes the SITL GPS from boot and runs nogps_odom_beacon, like the "
        "real AUV (POR). launch_por.sh --sim passes False.",
        choices=["True", "False"],
    )

    headless_arg = DeclareLaunchArgument(
        name="headless",
        default_value="False",
        description="Run Gazebo server only (no GUI) and MAVProxy without console GUI.",
        choices=["True", "False"],
    )

    task2_params_arg = DeclareLaunchArgument(
        name="task2_params_file",
        default_value=os.path.join(
            _banyu_bringup_share, "config", "task2_params.yaml"
        ),
        description="Path to ROS 2 parameter YAML file for Task 2 nodes",
    )

    por_params_arg = DeclareLaunchArgument(
        name="por_params_file",
        default_value=os.path.join(
            _banyu_bringup_share, "config", "por_params_timed_safe.yaml"
        ),
        description="Path to ROS 2 parameter YAML file for POR nodes",
    )

    # SetUseSimTime only configures launch's own node; SetParameter reaches every launched node.
    set_use_sim_time = SetParameter(
        name="use_sim_time",
        value=ParameterValue(
            AndSubstitution(
                EqualsSubstitution(LaunchConfiguration("use_sim"), "True"),
                EqualsSubstitution(LaunchConfiguration("use_sim_time"), "True"),
            ),
            value_type=bool,
        ),
    )

    task2_spawn_x = PythonExpression(
        ["'0' if '", LaunchConfiguration("mode"), "' == 'task2' else '-7'"]
    )
    task2_spawn_y = PythonExpression(
        ["'0' if '", LaunchConfiguration("mode"), "' == 'task2' else '0'"]
    )
    task2_spawn_z = PythonExpression(
        ["'-0.06' if '", LaunchConfiguration("mode"), "' == 'task2' else '-2'"]
    )

    gazebo_por_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_banyu_bringup_share, "launch", "gazebo_por.launch.py")
        ),
        condition=IfCondition(
            AndSubstitution(
                EqualsSubstitution(LaunchConfiguration("mode"), "por"),
                EqualsSubstitution(LaunchConfiguration("use_sim"), "True"),
            )
        ),
        launch_arguments={
            "use_sim": LaunchConfiguration("use_sim"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "headless": LaunchConfiguration("headless"),
        }.items(),
    )

    gazebo_main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_banyu_bringup_share, "launch", "gazebo_main.launch.py")
        ),
        condition=IfCondition(
            AndSubstitution(
                EqualsSubstitution(LaunchConfiguration("mode"), "task2"),
                EqualsSubstitution(LaunchConfiguration("use_sim"), "True"),
            )
        ),
        launch_arguments={
            "use_sim": LaunchConfiguration("use_sim"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "headless": LaunchConfiguration("headless"),
            "tier": LaunchConfiguration("tier"),
            "spawn_x": task2_spawn_x,
            "spawn_y": task2_spawn_y,
            "spawn_z": task2_spawn_z,
        }.items(),
    )

    # ArduSub SITL only; MAVROS runs on its own (tools/scripts/run/launch_mavros.sh --sim).
    sitl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_banyu_bringup_share, "launch", "sitl.launch.py")
        ),
        launch_arguments={
            "use_sim": LaunchConfiguration("use_sim"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_ardusub_sitl": LaunchConfiguration("use_ardusub_sitl"),
            "use_gps": LaunchConfiguration("use_gps"),
            "headless": LaunchConfiguration("headless"),
        }.items(),
    )

    replay_track_arg = DeclareLaunchArgument(
        name="replay_track",
        default_value="latest",
        description=(
            "Teach track for profiles with use_replay: 'latest' (newest .csv in "
            "$BANYU_REPLAY_DIR, default ~/banyu_replays), a file name there, or an absolute path."
        ),
    )

    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_banyu_bringup_share, "launch", "mission.launch.py")
        ),
        launch_arguments={
            "mode": LaunchConfiguration("mode"),
            "use_sim": LaunchConfiguration("use_sim"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "debug": LaunchConfiguration("debug"),
            "task2_params_file": LaunchConfiguration("task2_params_file"),
            "por_params_file": LaunchConfiguration("por_params_file"),
            "replay_track": LaunchConfiguration("replay_track"),
        }.items(),
    )

    record_arg = DeclareLaunchArgument(
        name="record",
        default_value="False",
        description="Record the sim run to $BANYU_BAG_DIR (default ~/banyu_bags) with sim time, "
        "including /sim/ground_truth/odom (POR), for teach & replay and por_path_compare.py.",
        choices=["True", "False"],
    )

    record_images_arg = DeclareLaunchArgument(
        name="record_images",
        default_value="False",
        description="Include compressed camera images in the sim bag (costs CPU: the sim already "
        "runs slower than real time).",
        choices=["True", "False"],
    )

    # Same recorder as sub_bringup.launch.py, run directly so SIGINT closes the bag cleanly.
    mission_recorder = ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix("banyu_bringup"), "lib", "banyu_bringup", "record_mission.sh"
            ),
            [LaunchConfiguration("mode"), "_sim"],
            "--sim-time",
        ],
        name="mission_recorder",
        output="screen",
        additional_env={
            "BANYU_BAG_IMAGES": LaunchConfiguration("record_images"),
            "BANYU_BAG_PARAMS": PythonExpression([
                "'", LaunchConfiguration("por_params_file"), "' if '", LaunchConfiguration("mode"),
                "' == 'por' else '", LaunchConfiguration("task2_params_file"), "'"
            ]),
        },
        sigterm_timeout="15",
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("record"), "True")),
    )

    # Same GPS-denied EKF aiding as sub_bringup.launch.py on the robot. Run directly, not via
    # `ros2 run`: launch only signals the wrapper, which would orphan the beacon on shutdown.
    nogps_odom_beacon = ExecuteProcess(
        cmd=[os.path.join(get_package_prefix("banyu_bringup"), "lib", "banyu_bringup",
                          "nogps_odom_beacon.py"),
             "--mavlink", "udpin:127.0.0.1:14552"],
        name="nogps_odom_beacon",
        output="screen",
        condition=IfCondition(
            AndSubstitution(
                EqualsSubstitution(LaunchConfiguration("use_sim"), "True"),
                EqualsSubstitution(LaunchConfiguration("use_gps"), "False"),
            )
        ),
    )

    return LaunchDescription(
        [
            mode_arg,
            use_sim_arg,
            use_sim_time_arg,
            use_ardusub_sitl_arg,
            tier_arg,
            debug_arg,
            use_gps_arg,
            headless_arg,
            task2_params_arg,
            por_params_arg,
            replay_track_arg,
            record_arg,
            record_images_arg,
            set_use_sim_time,
            gazebo_por_launch,
            gazebo_main_launch,
            sitl_launch,
            nogps_odom_beacon,
            mission_recorder,
            mission_launch,
        ]
    )
