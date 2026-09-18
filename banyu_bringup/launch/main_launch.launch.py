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
    PythonExpression,
)
from launch_ros.actions import SetUseSimTime


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
        description="Use sim_vehicle.py wrapper (True) or bare ardusub binary (False) for simulation",
        choices=["True", "False"],
    )

    fcu_url_arg = DeclareLaunchArgument(
        name="fcu_url",
        default_value="udp://127.0.0.1:14551@",
        description="FCU connection URL (udp://127.0.0.1:14551@ for SITL or udp://192.168.2.1:14550@ for real ROV). SITL keeps 14550 free for QGC.",
    )

    debug_arg = DeclareLaunchArgument(
        name="debug",
        default_value="False",
        description="Set the debug flag for example (Detection Windows)",
        choices=["True", "False"],
    )

    task2_follow_source_arg = DeclareLaunchArgument(
        name="task2_follow_source",
        default_value="fixture_route",
        description="Task 2 pipeline-following source.",
        choices=["fixture_route", "camera"],
    )

    task2_pose_timeout_arg = DeclareLaunchArgument(
        name="task2_pose_timeout_s",
        default_value="3.0",
        description="Maximum age of Task 2 vehicle pose data in seconds.",
    )

    task2_gps_receipt_pair_timeout_arg = DeclareLaunchArgument(
        name="task2_gps_receipt_pair_timeout_s",
        default_value="3.0",
        description="Maximum Task 2 GPS/pose receipt-time association gap in seconds.",
    )

    task2_report_action_name_arg = DeclareLaunchArgument(
        name="task2_report_action_name",
        default_value="/banyu/actions/send_mission_report",
        description="Task 2 mission-report action name.",
    )

    task2_report_max_retries_arg = DeclareLaunchArgument(
        name="task2_report_max_retries",
        default_value="3",
        description="Number of Task 2 UAV report retries after the initial send.",
    )

    task2_report_retry_period_arg = DeclareLaunchArgument(
        name="task2_report_retry_period_ms",
        default_value="1000",
        description="Task 2 UAV report retry period in milliseconds.",
    )

    set_use_sim_time = SetUseSimTime(
        True, condition=IfCondition(LaunchConfiguration("use_sim_time"))
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
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("mode"), "por")),
        launch_arguments={
            "use_sim": LaunchConfiguration("use_sim"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    gazebo_main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_banyu_bringup_share, "launch", "gazebo_main.launch.py")
        ),
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("mode"), "task2")),
        launch_arguments={
            "use_sim": LaunchConfiguration("use_sim"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
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
            "use_dvl": LaunchConfiguration("use_dvl"),
            "fcu_url": LaunchConfiguration("fcu_url"),
        }.items(),
    )

    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_banyu_bringup_share, "launch", "mission.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_sim": LaunchConfiguration("use_sim"),
            "debug": LaunchConfiguration("debug"),
            "mission_mode": LaunchConfiguration("mode"),
            "task2_follow_source": LaunchConfiguration("task2_follow_source"),
            "task2_pose_timeout_s": LaunchConfiguration("task2_pose_timeout_s"),
            "task2_gps_receipt_pair_timeout_s": LaunchConfiguration(
                "task2_gps_receipt_pair_timeout_s"
            ),
            "task2_use_dvl": LaunchConfiguration("use_dvl"),
            "task2_report_action_name": LaunchConfiguration("task2_report_action_name"),
            "task2_report_max_retries": LaunchConfiguration("task2_report_max_retries"),
            "task2_report_retry_period_ms": LaunchConfiguration(
                "task2_report_retry_period_ms"
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            mode_arg,
            use_sim_arg,
            use_sim_time_arg,
            use_ardusub_sitl_arg,
            fcu_url_arg,
            debug_arg,
            task2_follow_source_arg,
            task2_pose_timeout_arg,
            task2_gps_receipt_pair_timeout_arg,
            task2_report_action_name_arg,
            task2_report_max_retries_arg,
            task2_report_retry_period_arg,
            use_dvl_arg,
            set_use_sim_time,
            gazebo_por_launch,
            gazebo_main_launch,
            sitl_mavros_launch,
            mission_launch,
        ]
    )
