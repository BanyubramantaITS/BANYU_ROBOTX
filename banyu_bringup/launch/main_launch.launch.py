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
    EqualsSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import SetUseSimTime


def generate_launch_description():
    _banyu_bringup_share = get_package_share_directory("banyu_bringup")

    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="por",
        description="Which competition simulation to launch: 'por' (proof of readiness) or 'mission' (main mission).",
        choices=["por", "mission"],
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
        description="Use sim_vehicle.py wrapper (True) or bare ardusub binary (False) for simulation",
        choices=["True", "False"],
    )

    fcu_url_arg = DeclareLaunchArgument(
        name="fcu_url",
        default_value="udp://127.0.0.1:14551@",
        description="FCU connection URL (udp://127.0.0.1:14551@ for SITL or udp://192.168.2.1:14550@ for real ROV). SITL keeps 14550 free for QGC.",
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

    set_use_sim_time = SetUseSimTime(
        True, condition=IfCondition(LaunchConfiguration("use_sim_time"))
    )

    gazebo_por_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                _banyu_bringup_share, "launch", "utils", "gazebo_por.launch.py"
            )
        ),
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("mode"), "por")),
        launch_arguments={
            "use_sim": LaunchConfiguration("use_sim"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    gazebo_main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                _banyu_bringup_share, "launch", "utils", "gazebo_main.launch.py"
            )
        ),
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("mode"), "mission")
        ),
        launch_arguments={
            "use_sim": LaunchConfiguration("use_sim"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "tier": LaunchConfiguration("tier"),
        }.items(),
    )

    sitl_mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                _banyu_bringup_share, "launch", "utils", "sitl_mavros.launch.py"
            )
        ),
        launch_arguments={
            "use_sim": LaunchConfiguration("use_sim"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_ardusub_sitl": LaunchConfiguration("use_ardusub_sitl"),
            "fcu_url": LaunchConfiguration("fcu_url"),
        }.items(),
    )

    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_banyu_bringup_share, "launch", "utils", "mission.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "debug_arg": LaunchConfiguration("debug"),
            "mission_mode": LaunchConfiguration("mode"),
            "tier": LaunchConfiguration("tier"),
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
            set_use_sim_time,
            gazebo_por_launch,
            gazebo_main_launch,
            sitl_mavros_launch,
            mission_launch,
        ]
    )
