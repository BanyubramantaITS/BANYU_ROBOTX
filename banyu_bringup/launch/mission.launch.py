import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution,
    EqualsSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    banyu_bringup_share = get_package_share_directory("banyu_bringup")
    banyu_detection_share = get_package_share_directory("banyu_detection")

    default_task2_params = os.path.join(
        banyu_bringup_share, "config", "task2_params.yaml"
    )
    default_por_params = os.path.join(
        banyu_bringup_share, "config", "por_params.yaml"
    )
    default_hardware_params = os.path.join(
        banyu_bringup_share, "config", "hardware_params.yaml"
    )

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

    debug_arg = DeclareLaunchArgument(
        name="debug",
        default_value="False",
        description="Set the debug flag for example (Detection Windows)",
        choices=["True", "False"],
    )

    task2_params_arg = DeclareLaunchArgument(
        name="task2_params_file",
        default_value=default_task2_params,
        description="Path to ROS 2 parameter YAML file for Task 2 nodes",
    )

    por_params_arg = DeclareLaunchArgument(
        name="por_params_file",
        default_value=default_por_params,
        description="Path to ROS 2 parameter YAML file for POR nodes",
    )

    hardware_params_arg = DeclareLaunchArgument(
        name="hardware_params_file",
        default_value=default_hardware_params,
        description="Path to ROS 2 parameter YAML file for hardware camera/sensor overrides",
    )

    enable_esp32_trigger_arg = DeclareLaunchArgument(
        name="enable_esp32_trigger",
        default_value="False",
        description="Whether to launch the ESP32 trigger node",
        choices=["True", "False"],
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

    mode = LaunchConfiguration("mode")
    use_sim = LaunchConfiguration("use_sim")
    debug = LaunchConfiguration("debug")
    task2_params = LaunchConfiguration("task2_params_file")
    por_params = LaunchConfiguration("por_params_file")
    hardware_params = LaunchConfiguration("hardware_params_file")

    task2_fixture_mode = AndSubstitution(
        EqualsSubstitution(mode, "task2"),
        EqualsSubstitution(use_sim, "True"),
    )

    task2_hardware_override = PythonExpression([
        "'", task2_params, "' if '", use_sim, "'.lower() == 'true' else '", hardware_params, "'"
    ])

    por_mission_manager = Node(
        package="banyu_mission",
        executable="banyu_mission",
        name="banyu_mission",
        output="screen",
        parameters=[
            por_params,
            {
                "mission_mode": mode,
            },
        ],
        condition=IfCondition(EqualsSubstitution(mode, "por")),
    )

    task2_mission_manager = Node(
        package="banyu_mission",
        executable="banyu_mission",
        name="banyu_mission",
        output="screen",
        parameters=[
            task2_params,
            task2_hardware_override,
            {
                "mission_mode": mode,
            },
        ],
        condition=IfCondition(EqualsSubstitution(mode, "task2")),
    )

    por_detection = Node(
        package="banyu_detection",
        executable="por_detection",
        name="por_detection",
        parameters=[
            os.path.join(
                banyu_detection_share,
                "config",
                "por_detection_params.yaml",
            ),
            por_params,
            {"use_gui": debug},
        ],
        output="screen",
        condition=UnlessCondition(EqualsSubstitution(mode, "task2")),
    )

    por_vehicle_execution = Node(
        package="banyu_vehicle_execution",
        executable="vehicle_execution_node",
        name="vehicle_execution_node",
        parameters=[por_params],
        output="screen",
        condition=IfCondition(EqualsSubstitution(mode, "por")),
    )

    por_vehicle_status = Node(
        package="banyu_vehicle_interface",
        executable="vehicle_status_node",
        namespace="",
        name="banyu_vehicle_status",
        parameters=[
            por_params,
            {
                "default_operational_mode": ParameterValue(
                    PythonExpression(
                        ["2 if '", LaunchConfiguration("use_sim"), "'.lower() == 'true' else 0"]
                    ),
                    value_type=int,
                ),
                "rc_topic": ParameterValue(
                    PythonExpression(
                        ["'/sim/supervisor_rc' if '", LaunchConfiguration("use_sim"), "'.lower() == 'true' else '/mavros/rc/in'"]
                    ),
                    value_type=str,
                ),
            },
        ],
        output="screen",
        condition=IfCondition(EqualsSubstitution(mode, "por")),
    )

    task2_sim_provider = Node(
        package="banyu_bringup",
        executable="task2_sim_provider.py",
        name="task2_sim_provider",
        parameters=[task2_params],
        output="screen",
        condition=IfCondition(task2_fixture_mode),
    )

    task2_pipeline_detector = Node(
        package="banyu_detection",
        executable="task2_pipeline_detector",
        name="task2_pipeline_detector",
        parameters=[
            task2_params,
            {"use_gui": debug},
        ],
        output="screen",
        condition=IfCondition(EqualsSubstitution(mode, "task2")),
    )

    task2_vehicle_status = Node(
        package="banyu_vehicle_interface",
        executable="vehicle_status_node",
        namespace="",
        name="banyu_vehicle_status",
        parameters=[
            task2_params,
            {
                "default_operational_mode": ParameterValue(
                    PythonExpression(
                        ["2 if '", LaunchConfiguration("use_sim"), "'.lower() == 'true' else 0"]
                    ),
                    value_type=int,
                ),
                "rc_topic": ParameterValue(
                    PythonExpression(
                        ["'/sim/supervisor_rc' if '", LaunchConfiguration("use_sim"), "'.lower() == 'true' else '/mavros/rc/in'"]
                    ),
                    value_type=str,
                ),
            },
        ],
        output="screen",
        condition=IfCondition(EqualsSubstitution(mode, "task2")),
    )

    task2_ring_tracker = Node(
        package="banyu_detection",
        executable="ring_tracker_node",
        name="ring_tracker",
        parameters=[task2_params, task2_hardware_override],
        output="screen",
        condition=IfCondition(EqualsSubstitution(mode, "task2")),
    )

    task2_pipeline_guidance = Node(
        package="banyu_guidance",
        executable="pipeline_guidance_node",
        name="pipeline_guidance",
        parameters=[task2_params],
        output="screen",
        condition=IfCondition(EqualsSubstitution(mode, "task2")),
    )

    task2_uuv_comm = Node(
        package="banyu_communication",
        executable="uuv_comm_node",
        name="uuv_comm_node",
        parameters=[task2_params],
        output="screen",
        condition=IfCondition(EqualsSubstitution(mode, "task2")),
    )

    task2_pipeline_buoys = Node(
        package="banyu_communication",
        executable="pipeline_buoys_node",
        name="pipeline_buoys_node",
        parameters=[task2_params],
        output="screen",
        condition=IfCondition(EqualsSubstitution(mode, "task2")),
    )

    task2_vehicle_execution = Node(
        package="banyu_vehicle_execution",
        executable="vehicle_execution_node",
        name="vehicle_execution_node",
        parameters=[task2_params],
        output="screen",
        condition=IfCondition(EqualsSubstitution(mode, "task2")),
    )

    task2_pipeline_end = Node(
        package="banyu_detection",
        executable="pipeline_end_node",
        name="pipeline_end_node",
        parameters=[task2_params],
        output="screen",
        condition=IfCondition(EqualsSubstitution(mode, "task2")),
    )

    task2_calibrated_depth = Node(
        package="banyu_vehicle_execution",
        executable="calibrated_depth_node",
        name="calibrated_depth_node",
        parameters=[task2_params],
        output="screen",
        condition=IfCondition(EqualsSubstitution(mode, "task2")),
    )

    task2_survey_evidence = Node(
        package="banyu_detection",
        executable="survey_evidence_node",
        name="survey_evidence_node",
        parameters=[task2_params],
        output="screen",
        condition=IfCondition(EqualsSubstitution(mode, "task2")),
    )

    task2_pinger_localization = Node(
        package="banyu_detection",
        executable="pinger_localization",
        name="pinger_localization",
        parameters=[task2_params],
        output="screen",
        condition=IfCondition(EqualsSubstitution(mode, "task2")),
    )

    active_params = PythonExpression([
        "'", por_params, "' if '", mode, "' == 'por' else '", task2_params, "'"
    ])

    esp32_bridge = Node(
        package="banyu_vehicle_interface",
        executable="esp32_bridge_node",
        name="esp32_bridge_node",
        parameters=[active_params, hardware_params],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_sim")),
    )

    esp32_trigger = Node(
        package="banyu_vehicle_interface",
        executable="esp32_trigger_node",
        name="esp32_trigger_node",
        parameters=[active_params, hardware_params],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_esp32_trigger")),
    )

    sim_rc_deadman = Node(
        package="banyu_vehicle_interface",
        executable="sim_rc_deadman_node",
        name="sim_rc_deadman_node",
        parameters=[active_params],
        output="screen",
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("use_sim"), "True")
        ),
    )
    return LaunchDescription(
        [
            mode_arg,
            use_sim_arg,
            use_sim_time_arg,
            debug_arg,
            task2_params_arg,
            por_params_arg,
            hardware_params_arg,
            enable_esp32_trigger_arg,
            set_use_sim_time,
            por_mission_manager,
            task2_mission_manager,
            por_detection,
            por_vehicle_execution,
            por_vehicle_status,
            task2_sim_provider,
            task2_pipeline_detector,
            task2_vehicle_status,
            task2_ring_tracker,
            task2_pipeline_guidance,
            task2_uuv_comm,
            task2_pipeline_buoys,
            task2_vehicle_execution,
            task2_pipeline_end,
            task2_calibrated_depth,
            task2_survey_evidence,
            task2_pinger_localization,
            esp32_bridge,
            esp32_trigger,
            sim_rc_deadman,
        ]
    )
