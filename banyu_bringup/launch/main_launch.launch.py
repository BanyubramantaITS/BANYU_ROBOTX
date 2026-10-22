import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
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
            "headless": LaunchConfiguration("headless"),
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
            "task2_params_file": LaunchConfiguration("task2_params_file"),
            "por_params_file": LaunchConfiguration("por_params_file"),
        }.items(),
    )

    return LaunchDescription(
        [
            mode_arg,
            use_sim_arg,
            use_sim_time_arg,
            use_ardusub_sitl_arg,
            fcu_url_arg,
            tier_arg,
            debug_arg,
            use_dvl_arg,
            headless_arg,
            task2_params_arg,
            por_params_arg,
            set_use_sim_time,
            gazebo_por_launch,
            gazebo_main_launch,
            sitl_mavros_launch,
            mission_launch,
        ]
    )
