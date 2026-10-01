import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
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

    # Which competition behaviour to run: 'por' or 'mission'. Forwarded to the
    # mission node so POR and main-mission behaviour can diverge later.
    mission_mode_arg = DeclareLaunchArgument(
        name="mission_mode",
        default_value="por",
        description="Which mission behaviour to run: 'por' (proof of readiness) or 'mission' (main mission).",
        choices=["por", "mission"],
    )

    tier_arg = DeclareLaunchArgument(
        name="tier",
        default_value="advanced",
        description="Task 2 tier: 'core' (survey + report), 'advanced' (also repair + resource request) or "
                    "'disruptive' (as advanced, and the tin colour).",
        choices=["core", "advanced", "disruptive"],
    )

    mission_manager = Node(
        package="banyu_mission",
        executable="banyu_mission",
        name="banyu_mission",
        output="screen",
        parameters=[{"mission_mode": LaunchConfiguration("mission_mode"),
                     "task2_tier": LaunchConfiguration("tier")}],
    )

    # PoR uses the single-best-object yellow detector; Task 2 its own pipeline/pinger perception
    # below. Exactly one set runs per mode.
    detection_node = Node(
        package="banyu_detection",
        executable="por_detection",
        name="por_detection",
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("mission_mode"), "por")
        ),
        parameters=[
            os.path.join(
                get_package_share_directory("banyu_detection"),
                "config",
                "por_detection_params.yaml",
            ),
            {"use_gui": LaunchConfiguration("debug_arg")},
        ],
        output="screen",
    )

    image_noise_arg = DeclareLaunchArgument(
        name="image_noise",
        default_value="0.0",
        description="Sim-only extra gaussian noise (8-bit DN) on the bottom camera, added by pipeline_detection.",
    )

    # Task 2: pipeline + light nodes from the bottom camera, pinger bearing from the hydrophones.
    # The survey itself is banyu_mission's pipeline_survey_tree.xml.
    pipeline_detection_node = Node(
        package="banyu_detection",
        executable="pipeline_detection",
        name="pipeline_detection",
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("mission_mode"), "mission")
        ),
        parameters=[{
            "image_noise": ParameterValue(LaunchConfiguration("image_noise"), value_type=float),
            # Repair probe rod in the bottom camera (x y w h, 320x240), measured off sim frames;
            # all zero when Core fits no probe. ponytail: re-measure on the real camera.
            "probe_rect": ParameterValue(PythonExpression(
                ["[0, 0, 0, 0] if '", LaunchConfiguration("tier"), "' == 'core' else [148, 0, 24, 82]"])),
        }],
        output="screen",
    )

    pinger_localization_node = Node(
        package="banyu_detection",
        executable="pinger_localization",
        name="pinger_localization",
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("mission_mode"), "mission")
        ),
        output="screen",
    )

    return LaunchDescription(
        [
            image_noise_arg,
            use_sim_time_arg,
            debug_arg,
            mission_mode_arg,
            tier_arg,
            set_use_sim_time,
            mission_manager,
            detection_node,
            pipeline_detection_node,
            pinger_localization_node,
        ]
    )
