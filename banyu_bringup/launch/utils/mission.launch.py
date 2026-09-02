import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import AndSubstitution, EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        choices=["True", "False"],
    )

    use_sim_arg = DeclareLaunchArgument(
        name="use_sim",
        default_value="True",
        choices=["True", "False"],
    )

    set_use_sim_time = SetUseSimTime(
        True, condition=IfCondition(LaunchConfiguration("use_sim_time"))
    )

    debug_arg = DeclareLaunchArgument(
        name="debug",
        default_value="False",
        description="Set the debug flag for example (Detection Windows)",
        choices=["True", "False"],
    )

    # Which competition behaviour to run: POR or Task 2.
    mission_mode_arg = DeclareLaunchArgument(
        name="mission_mode",
        default_value="por",
        description="Which behaviour to run: 'por' or 'task2'.",
        choices=["por", "task2"],
    )

    ring_colors_arg = DeclareLaunchArgument(
        name="ring_colors",
        default_value="['green', 'green', 'green', 'red', 'green']",
        description="Ordered Task 2 pipeline ring colors.",
    )

    publish_rate_arg = DeclareLaunchArgument(
        name="publish_rate",
        default_value="5.0",
        description="Task 2 simulation fixture publish rate in Hz.",
    )

    pipeline_standoff_m_arg = DeclareLaunchArgument(
        name="pipeline_standoff_m",
        default_value="1.0",
        description="Vertical vehicle-center offset above the pipeline mesh.",
    )

    task2_fixture_mode = AndSubstitution(
        EqualsSubstitution(LaunchConfiguration("mission_mode"), "task2"),
        EqualsSubstitution(LaunchConfiguration("use_sim"), "True"),
    )

    mission_manager = Node(
        package="banyu_mission",
        executable="banyu_mission",
        name="banyu_mission",
        output="screen",
        parameters=[
            {
                "mission_mode": LaunchConfiguration("mission_mode"),
                "task2_fixture_mode": ParameterValue(
                    task2_fixture_mode, value_type=bool
                ),
            }
        ],
    )

    detection_node = Node(
        package="banyu_detection",
        executable="por_detection",
        name="por_detection",
        parameters=[
            os.path.join(
                get_package_share_directory("banyu_detection"),
                "config",
                "por_detection_params.yaml",
            ),
            {"use_gui": LaunchConfiguration("debug_arg")},
        ],
        output="screen",
        condition=UnlessCondition(
            EqualsSubstitution(LaunchConfiguration("mission_mode"), "task2")
        ),
    )

    task2_sim_provider = Node(
        package="banyu_bringup",
        executable="task2_sim_provider.py",
        name="task2_sim_provider",
        parameters=[
            {
                "ring_colors": ParameterValue(
                    LaunchConfiguration("ring_colors"), value_type=List[str]
                ),
                "publish_rate": ParameterValue(
                    LaunchConfiguration("publish_rate"), value_type=float
                ),
                "pipeline_standoff_m": ParameterValue(
                    LaunchConfiguration("pipeline_standoff_m"), value_type=float
                ),
            }
        ],
        output="screen",
        condition=IfCondition(
            task2_fixture_mode
        ),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_sim_arg,
            debug_arg,
            mission_mode_arg,
            ring_colors_arg,
            publish_rate_arg,
            pipeline_standoff_m_arg,
            set_use_sim_time,
            mission_manager,
            detection_node,
            task2_sim_provider,
        ]
    )
