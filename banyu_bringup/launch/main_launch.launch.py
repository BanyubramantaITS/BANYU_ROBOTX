import os

from ament_index_python.packages import get_package_share_directory
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

    fcu_url_arg = DeclareLaunchArgument(
        name="fcu_url",
        default_value="udp://127.0.0.1:14551@",
        description=(
            "FCU connection URL (udp://127.0.0.1:14551@ for SITL or "
            "sub_bringup.launch.py for the real AUV). SITL keeps 14550 free for QGC."
        ),
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

    gps_gate_arg = DeclareLaunchArgument(
        name="gps_gate",
        default_value="False",
        description="Simulation only: cut the SITL GPS while submerged (like the real AUV, whose GPS "
        "needs the antenna above water) and restore it after a reacquire delay. Task 2 sim only.",
        choices=["True", "False"],
    )

    gps_loss_depth_arg = DeclareLaunchArgument(
        name="gps_loss_depth_m",
        default_value="0.1",
        description="gps_gate: depth in metres below which the GPS is cut (worst case 0.1).",
    )

    gps_reacquire_delay_arg = DeclareLaunchArgument(
        name="gps_reacquire_delay_s",
        default_value="10.0",
        description="gps_gate: seconds at the surface before the GPS comes back.",
    )

    spawn_x_arg = DeclareLaunchArgument(
        name="spawn_x", default_value="", description="Task 2 sim spawn east (m, world origin = GPS home); empty = origin."
    )
    spawn_y_arg = DeclareLaunchArgument(
        name="spawn_y", default_value="", description="Task 2 sim spawn north (m); empty = 0."
    )
    spawn_yaw_arg = DeclareLaunchArgument(
        name="spawn_yaw", default_value="", description="Task 2 sim spawn heading (rad); empty = 0."
    )
    light_seed_arg = DeclareLaunchArgument(
        name="light_seed",
        default_value="",
        description="Simulation: seed that picks the damaged pipeline node and the flash colours (empty: node 2, "
        "tin RED, circle BLUE). Seeds 2, 1, 5, 9 give damaged node 1, 2, 3, 4.",
    )

    gamble_red_arg = DeclareLaunchArgument(
        name="gamble_red", default_value="2,3",
        description="sensors:=gamble: RED positions, 1 = closest to the active buoy; '2,3' = one of them at random, '3' = always 3.",
    )
    sensors_arg = DeclareLaunchArgument(
        name="sensors",
        default_value="full",
        description="Task 2 sensor set: 'full' (Bar30 depth), 'baro' (Bar30 + ALT_HOLD, dive stopped by camera, order dead-reckoned) or 'minimal' (bottom camera, Pixhawk IMU/compass, "
        "surface GPS; STABILIZE under water, heave from the camera).",
        choices=["full", "baro", "minimal", "gamble"],
    )
    pipe_depth_arg = DeclareLaunchArgument(
        name="pipe_depth",
        default_value="",
        description="Simulation: depth (m) of the pipeline's light nodes below the surface; empty = the world's 13.4 m.",
    )

    # Simulation scenarios (task2_future_plan_depth_free.md section 4); empty = none.
    scenario_args = [
        DeclareLaunchArgument(name=n, default_value="", description=d) for n, d in (
            ("pipe_dx", "Scenario: shift the pipe east against the buoys (m)."),
            ("pipe_dy", "Scenario: shift the pipe north against the buoys (m)."),
            ("pipe_dyaw_deg", "Scenario: rotate the pipe about its start end (deg)."),
            ("visibility_m", "Scenario: water visibility (m)."),
            ("buoy_error_x", "Scenario: the USV reports both buoys this far east of their place (m)."),
            ("buoy_error_y", "Scenario: the USV reports both buoys this far north of their place (m)."),
            ("current_east", "Scenario: water current east (m/s)."),
            ("current_north", "Scenario: water current north (m/s)."),
            ("current_gust", "Scenario: extra current along the steady one for 5 s of every 30 s (m/s)."),
            ("light_fault", "Scenario: a functional light node fault '<node>:thermistor|mag|hot'."),
            ("camera_model", "Scenario: 'fisheye' distorts the bottom camera (empty or 'pinhole': none)."),
            ("ir_led", "Scenario: 'on' adds the bottom camera's LED ring as a spot light (empty or 'off': none)."),
            ("site", "Simulation site: empty = Long Beach (RATBeach), 'marina_bay' = Marina Bay home and course."),
            ("camera_width", "Bottom camera width in pixels (4:3); empty = 320x240."),
            ("fcu_faults", "Scenario: FCU sensor faults, e.g. 'compass=20,gps_offset=0.3:0.2,gps_drop=40:15'."),
        )
    ]

    dive_point_arg = DeclareLaunchArgument(
        name="dive_point",
        default_value="buoy",
        description="Task 2 dive point: 'buoy' (inside the start buoy) or 'midpoint' (midway between the buoys, two legs).",
        choices=["buoy", "midpoint"],
    )

    use_hydrophone_arg = DeclareLaunchArgument(
        name="use_hydrophone",
        default_value="False",
        description="Task 2: opt in to the hydrophone (pinger_localization, sim HydrophoneArray plugin, pinger homing "
        "in SearchPipeline). The competition vehicle has none.",
        choices=["True", "False"],
    )

    use_dvl_arg = DeclareLaunchArgument(
        name="use_dvl",
        default_value="False",
        description=(
            "Enable DVL transport, source switching, and Task 2 DVL readiness "
            "checks. False leaves the estimator on its configured source set."
        ),
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
            _banyu_bringup_share, "config", "por_params.yaml"
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

    # spawn_x / spawn_y / spawn_yaw override the Task 2 sim spawn (default: at the world origin, between the buoys).
    task2_spawn_x = PythonExpression(
        ["'", LaunchConfiguration("spawn_x"), "' or ('0' if '", LaunchConfiguration("mode"),
         "' == 'task2' else '-7')"]
    )
    task2_spawn_y = PythonExpression(
        ["'", LaunchConfiguration("spawn_y"), "' or '0'"]
    )
    task2_spawn_yaw = PythonExpression(["'", LaunchConfiguration("spawn_yaw"), "' or '0'"])
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
            "light_seed": LaunchConfiguration("light_seed"),
            "pipe_depth": LaunchConfiguration("pipe_depth"),
            "pipe_dx": LaunchConfiguration("pipe_dx"),
            "pipe_dy": LaunchConfiguration("pipe_dy"),
            "pipe_dyaw_deg": LaunchConfiguration("pipe_dyaw_deg"),
            "visibility_m": LaunchConfiguration("visibility_m"),
            "light_fault": LaunchConfiguration("light_fault"),
            "camera_model": PythonExpression(["'", LaunchConfiguration("camera_model"), "' or 'pinhole'"]),
            "ir_led": PythonExpression(["'", LaunchConfiguration("ir_led"), "' or 'off'"]),
            "camera_width": LaunchConfiguration("camera_width"),
            "use_hydrophone": LaunchConfiguration("use_hydrophone"),
            "spawn_x": task2_spawn_x,
            "spawn_y": task2_spawn_y,
            "spawn_z": task2_spawn_z,
            "spawn_yaw": task2_spawn_yaw,
        }.items(),
    )

    sitl_mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_banyu_bringup_share, "launch", "sitl_mavros.launch.py")
        ),
        launch_arguments={
            "use_sim": LaunchConfiguration("use_sim"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_ardusub_sitl": LaunchConfiguration("use_ardusub_sitl"),
            "fcu_url": LaunchConfiguration("fcu_url"),
            "use_dvl": LaunchConfiguration("use_dvl"),
            "use_gps": LaunchConfiguration("use_gps"),
            "headless": LaunchConfiguration("headless"),
            "site": LaunchConfiguration("site"),
        }.items(),
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
            "tier": LaunchConfiguration("tier"),
            "sensors": LaunchConfiguration("sensors"),
            "use_hydrophone": LaunchConfiguration("use_hydrophone"),
            "dive_point": LaunchConfiguration("dive_point"),
            "gamble_red": LaunchConfiguration("gamble_red"),
            "site": LaunchConfiguration("site"),
            "pipe_dx": LaunchConfiguration("pipe_dx"),
            "pipe_dy": LaunchConfiguration("pipe_dy"),
            "pipe_dyaw_deg": LaunchConfiguration("pipe_dyaw_deg"),
            "buoy_error_x": LaunchConfiguration("buoy_error_x"),
            "buoy_error_y": LaunchConfiguration("buoy_error_y"),
            "task2_params_file": LaunchConfiguration("task2_params_file"),
            "por_params_file": LaunchConfiguration("por_params_file"),
        }.items(),
    )

    # Same GPS-denied EKF aiding as sub_bringup.launch.py on the robot.
    nogps_odom_beacon = ExecuteProcess(
        cmd=["ros2", "run", "banyu_bringup", "nogps_odom_beacon.py",
             "--mavlink", "udpin:127.0.0.1:14552"],
        name="nogps_odom_beacon",
        output="screen",
        condition=IfCondition(
            AndSubstitution(
                EqualsSubstitution(LaunchConfiguration("use_sim"), "True"),
                AndSubstitution(
                    EqualsSubstitution(LaunchConfiguration("use_gps"), "False"),
                    EqualsSubstitution(LaunchConfiguration("use_dvl"), "False"),
                ),
            )
        ),
    )

    sim_gps_gate = ExecuteProcess(
        cmd=["ros2", "run", "banyu_bringup", "sim_gps_gate.py", "--ros-args",
             "-p", ["loss_depth_m:=", LaunchConfiguration("gps_loss_depth_m")],
             "-p", ["reacquire_delay_s:=", LaunchConfiguration("gps_reacquire_delay_s")],
             # The world's depth for the gate: the SITL barometer, never the mission's water depth (which a depth fault or
             # sensors:=minimal turns into the GPS surface proof, so the GPS would never come back).
             "-p", "rel_alt_topic:=/mavros/global_position/rel_alt",
             "-p", "use_sim_time:=true"],
        name="sim_gps_gate",
        output="screen",
        condition=IfCondition(
            AndSubstitution(
                EqualsSubstitution(LaunchConfiguration("use_sim"), "True"),
                AndSubstitution(
                    EqualsSubstitution(LaunchConfiguration("gps_gate"), "True"),
                    EqualsSubstitution(LaunchConfiguration("mode"), "task2"),
                ),
            )
        ),
    )

    sim_current = ExecuteProcess(
        cmd=["ros2", "run", "banyu_bringup", "sim_current.py",
             "--east", PythonExpression(["'", LaunchConfiguration("current_east"), "' or '0'"]),
             "--north", PythonExpression(["'", LaunchConfiguration("current_north"), "' or '0'"]),
             "--gust", PythonExpression(["'", LaunchConfiguration("current_gust"), "' or '0'"])],
        name="sim_current",
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("use_sim"), "' == 'True' and ('", LaunchConfiguration("current_east"), "' or '",
            LaunchConfiguration("current_north"), "' or '", LaunchConfiguration("current_gust"), "') != ''"])),
    )

    sim_fcu_faults = ExecuteProcess(
        cmd=["ros2", "run", "banyu_bringup", "sim_fcu_faults.py", "--faults", LaunchConfiguration("fcu_faults"),
             "--ros-args", "-p", "use_sim_time:=true"],
        name="sim_fcu_faults",
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("use_sim"), "' == 'True' and '", LaunchConfiguration("fcu_faults"), "' != ''"])),
    )

    return LaunchDescription(
        scenario_args + [
            mode_arg,
            use_sim_arg,
            use_sim_time_arg,
            use_ardusub_sitl_arg,
            fcu_url_arg,
            tier_arg,
            debug_arg,
            use_gps_arg,
            gps_gate_arg,
            light_seed_arg,
            sensors_arg,
            gamble_red_arg,
            pipe_depth_arg,
            spawn_x_arg,
            spawn_y_arg,
            spawn_yaw_arg,
            gps_loss_depth_arg,
            gps_reacquire_delay_arg,
            use_dvl_arg,
            use_hydrophone_arg,
            dive_point_arg,
            headless_arg,
            task2_params_arg,
            por_params_arg,
            set_use_sim_time,
            gazebo_por_launch,
            gazebo_main_launch,
            sitl_mavros_launch,
            nogps_odom_beacon,
            sim_gps_gate,
            mission_launch,
            # After every argument is declared: their conditions read use_sim.
            sim_current,
            sim_fcu_faults,
        ]
    )
