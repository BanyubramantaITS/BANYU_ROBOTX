import os
from typing import List

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
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.parameter_descriptions import ParameterValue


def _task2_report_max_attempts_substitution():
    return PythonExpression(
        [
            "min(255, int('",
            LaunchConfiguration("task2_report_max_retries"),
            "') + 1)",
        ]
    )


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

    task2_follow_source_arg = DeclareLaunchArgument(
        name="task2_follow_source",
        default_value="fixture_route",
        description="Task 2 pipeline-following source.",
        choices=["fixture_route", "camera"],
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

    task2_gps_surface_depth_arg = DeclareLaunchArgument(
        name="task2_gps_surface_depth",
        default_value="-0.4",
        description="Task 2 GPS acquisition surface depth in map ENU meters.",
    )

    task2_gps_surface_tolerance_arg = DeclareLaunchArgument(
        name="task2_gps_surface_tolerance",
        default_value="0.5",
        description="Task 2 GPS acquisition surface depth tolerance in meters.",
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

    task2_use_dvl_arg = DeclareLaunchArgument(
        name="task2_use_dvl",
        default_value="False",
        description="Enable DVL freshness/readiness requirements for Task 2.",
        choices=["True", "False"],
    )

    task2_gps_fix_timeout_arg = DeclareLaunchArgument(
        name="task2_gps_fix_timeout_s",
        default_value="1.0",
        description="Maximum age of Task 2 GPS fix data in seconds.",
    )

    task2_start_buoy_map_x_arg = DeclareLaunchArgument(
        name="task2_start_buoy_map_x",
        default_value="43.95",
        description="Task 2 start-buoy ENU map X coordinate.",
    )

    task2_start_buoy_map_y_arg = DeclareLaunchArgument(
        name="task2_start_buoy_map_y",
        default_value="13.85",
        description="Task 2 start-buoy ENU map Y coordinate.",
    )

    task2_gps_max_fix_pose_skew_arg = DeclareLaunchArgument(
        name="task2_gps_max_fix_pose_skew_s",
        default_value="0.5",
        description="Maximum Task 2 GPS-fix/map-pose timestamp skew in seconds.",
    )

    task2_detector_image_topic_arg = DeclareLaunchArgument(
        name="task2_detector_image_topic",
        default_value="/front_camera/image_raw",
        description="Task 2 detector image topic.",
    )

    task2_detector_camera_info_topic_arg = DeclareLaunchArgument(
        name="task2_detector_camera_info_topic",
        default_value="/camera/camera_info",
        description="Task 2 detector camera-info topic.",
    )

    task2_detector_observation_topic_arg = DeclareLaunchArgument(
        name="task2_detector_observation_topic",
        default_value="/banyu/pipeline/observation",
        description="Task 2 pipeline observation topic.",
    )

    task2_detector_ring_observation_topic_arg = DeclareLaunchArgument(
        name="task2_detector_ring_observation_topic",
        default_value="/banyu/pipeline/ring_observation",
        description="Task 2 ring observation topic.",
    )

    task2_vehicle_state_topic_arg = DeclareLaunchArgument(
        name="task2_vehicle_state_topic",
        default_value="/mavros/state",
        description="Task 2 MAVROS FCU state topic.",
    )

    task2_vehicle_rc_topic_arg = DeclareLaunchArgument(
        name="task2_vehicle_rc_topic",
        default_value="/mavros/rc/in",
        description="Task 2 MAVROS RC input topic.",
    )

    task2_vehicle_gps_topic_arg = DeclareLaunchArgument(
        name="task2_vehicle_gps_topic",
        default_value="/mavros/global_position/raw/fix",
        description="Task 2 MAVROS GPS fix topic.",
    )

    task2_vehicle_status_topic_arg = DeclareLaunchArgument(
        name="task2_vehicle_status_topic",
        default_value="/banyu/vehicle_status",
        description="Task 2 normalized vehicle-status topic.",
    )

    task2_vehicle_status_publish_rate_arg = DeclareLaunchArgument(
        name="task2_vehicle_status_publish_rate_hz",
        default_value="10.0",
        description="Task 2 vehicle-status publish rate in Hz.",
    )

    task2_vehicle_fcu_timeout_arg = DeclareLaunchArgument(
        name="task2_vehicle_fcu_timeout_s",
        default_value="2.0",
        description="Task 2 FCU freshness timeout in seconds.",
    )

    task2_vehicle_rc_timeout_arg = DeclareLaunchArgument(
        name="task2_vehicle_rc_timeout_s",
        default_value="2.0",
        description="Task 2 RC freshness timeout in seconds.",
    )

    task2_vehicle_gps_timeout_arg = DeclareLaunchArgument(
        name="task2_vehicle_gps_timeout_s",
        default_value="2.0",
        description="Task 2 GPS freshness timeout in seconds.",
    )

    task2_vehicle_rc_takeover_channel_arg = DeclareLaunchArgument(
        name="task2_vehicle_rc_takeover_channel",
        default_value="4",
        description="Task 2 zero-based RC takeover channel.",
    )

    task2_vehicle_rc_takeover_threshold_arg = DeclareLaunchArgument(
        name="task2_vehicle_rc_takeover_threshold",
        default_value="1700",
        description="Task 2 RC takeover threshold.",
    )

    task2_vehicle_autonomy_mode_arg = DeclareLaunchArgument(
        name="task2_vehicle_autonomy_mode",
        default_value="GUIDED",
        description="Task 2 autonomy mode reported by the vehicle adapter.",
    )

    task2_ring_pose_topic_arg = DeclareLaunchArgument(
        name="task2_ring_pose_topic",
        default_value="/mavros/local_position/pose",
        description="Task 2 pose topic used to correlate ring observations.",
    )

    task2_ring_state_topic_arg = DeclareLaunchArgument(
        name="task2_ring_state_topic",
        default_value="/banyu/pipeline/ring_state",
        description="Task 2 tracked pipeline-ring state topic.",
    )

    task2_accepted_ring_topic_arg = DeclareLaunchArgument(
        name="task2_accepted_ring_topic",
        default_value="/banyu/pipeline/accepted_ring",
        description="Task 2 accepted-ring record topic.",
    )

    task2_ring_repair_service_arg = DeclareLaunchArgument(
        name="task2_ring_repair_service",
        default_value="/banyu/pipeline/acknowledge_ring_repair",
        description="Task 2 ring-repair acknowledgement service.",
    )

    task2_guidance_output_topic_arg = DeclareLaunchArgument(
        name="task2_guidance_output_topic",
        default_value="/banyu/guidance/velocity_intent",
        description="Task 2 guidance velocity-intent topic.",
    )

    task2_guidance_intent_frame_arg = DeclareLaunchArgument(
        name="task2_guidance_intent_frame",
        default_value="base_link",
        description="Task 2 frame used by guidance velocity intents.",
    )

    task2_guidance_publish_rate_arg = DeclareLaunchArgument(
        name="task2_guidance_publish_rate_hz",
        default_value="20.0",
        description="Task 2 guidance publish rate in Hz.",
    )

    task2_guidance_surge_gain_arg = DeclareLaunchArgument(
        name="task2_guidance_surge_gain_mps",
        default_value="0.8",
        description="Task 2 guidance surge gain.",
    )

    task2_guidance_max_surge_arg = DeclareLaunchArgument(
        name="task2_guidance_max_surge_mps",
        default_value="0.6",
        description="Task 2 guidance maximum surge speed.",
    )

    task2_guidance_sway_gain_arg = DeclareLaunchArgument(
        name="task2_guidance_sway_gain_mps",
        default_value="0.8",
        description="Task 2 guidance sway gain.",
    )

    task2_guidance_max_sway_arg = DeclareLaunchArgument(
        name="task2_guidance_max_sway_mps",
        default_value="0.4",
        description="Task 2 guidance maximum sway speed.",
    )

    task2_guidance_yaw_gain_arg = DeclareLaunchArgument(
        name="task2_guidance_yaw_gain_rad_s",
        default_value="0.8",
        description="Task 2 guidance yaw gain.",
    )

    task2_guidance_max_yaw_rate_arg = DeclareLaunchArgument(
        name="task2_guidance_max_yaw_rate_rad_s",
        default_value="0.6",
        description="Task 2 guidance maximum yaw rate.",
    )

    task2_guidance_min_confidence_arg = DeclareLaunchArgument(
        name="task2_guidance_min_confidence",
        default_value="0.2",
        description="Minimum Task 2 observation confidence for guidance.",
    )

    task2_guidance_max_observation_age_arg = DeclareLaunchArgument(
        name="task2_guidance_max_observation_age_s",
        default_value="1.0",
        description="Maximum Task 2 observation age for guidance.",
    )

    task2_guidance_max_future_skew_arg = DeclareLaunchArgument(
        name="task2_guidance_max_future_skew_s",
        default_value="0.2",
        description="Maximum Task 2 future observation skew for guidance.",
    )

    task2_guidance_yaw_control_arg = DeclareLaunchArgument(
        name="task2_guidance_yaw_control_enabled",
        default_value="True",
        description="Enable Task 2 guidance yaw control.",
        choices=["True", "False"],
    )

    task2_report_action_name_arg = DeclareLaunchArgument(
        name="task2_report_action_name",
        default_value="/banyu/actions/send_mission_report",
        description="Task 2 mission-report action name served by the UAV report node.",
    )

    task2_report_topic_arg = DeclareLaunchArgument(
        name="task2_report_topic",
        default_value="/banyu/uav/mission_report",
        description="Task 2 transport-neutral UAV mission-report topic.",
    )

    task2_report_ack_topic_arg = DeclareLaunchArgument(
        name="task2_report_ack_topic",
        default_value="/banyu/uav/mission_report_ack",
        description="Task 2 transport-neutral UAV report acknowledgement topic.",
    )

    task2_report_retry_period_arg = DeclareLaunchArgument(
        name="task2_report_retry_period_ms",
        default_value="1000",
        description="Task 2 UAV report retry period in milliseconds.",
    )

    task2_report_max_retries_arg = DeclareLaunchArgument(
        name="task2_report_max_retries",
        default_value="3",
        description="Number of Task 2 UAV report retries after the initial send.",
    )

    task2_report_max_attempts = _task2_report_max_attempts_substitution()

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
                "task2_follow_source": LaunchConfiguration("task2_follow_source"),
                "task2_fixture_mode": ParameterValue(
                    task2_fixture_mode, value_type=bool
                ),
                "task2_pose_timeout_s": ParameterValue(
                    LaunchConfiguration("task2_pose_timeout_s"), value_type=float
                ),
                "task2_gps_receipt_pair_timeout_s": ParameterValue(
                    LaunchConfiguration("task2_gps_receipt_pair_timeout_s"),
                    value_type=float,
                ),
                "task2_use_dvl": ParameterValue(
                    LaunchConfiguration("task2_use_dvl"), value_type=bool
                ),
                "task2_gps_fix_timeout_s": ParameterValue(
                    LaunchConfiguration("task2_gps_fix_timeout_s"), value_type=float
                ),
                "task2_gps_surface_depth": ParameterValue(
                    LaunchConfiguration("task2_gps_surface_depth"),
                    value_type=float,
                ),
                "task2_gps_surface_tolerance": ParameterValue(
                    LaunchConfiguration("task2_gps_surface_tolerance"),
                    value_type=float,
                ),
                "task2_gps_max_fix_pose_skew_s": ParameterValue(
                    LaunchConfiguration("task2_gps_max_fix_pose_skew_s"),
                    value_type=float,
                ),
                "task2_pipeline_observation_topic": LaunchConfiguration(
                    "task2_detector_observation_topic"
                ),
                "task2_pipeline_ring_observation_topic": LaunchConfiguration(
                    "task2_detector_ring_observation_topic"
                ),
                "actions.send_mission_report": LaunchConfiguration(
                    "task2_report_action_name"
                ),
                "mission.max_retries": ParameterValue(
                    LaunchConfiguration("task2_report_max_retries"), value_type=int
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
            {"use_gui": LaunchConfiguration("debug")},
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
                "surface_depth": ParameterValue(
                    LaunchConfiguration("task2_gps_surface_depth"),
                    value_type=float,
                ),
                "surface_tolerance": ParameterValue(
                    LaunchConfiguration("task2_gps_surface_tolerance"),
                    value_type=float,
                ),
                "global_pose_max_skew_s": ParameterValue(
                    LaunchConfiguration("task2_gps_max_fix_pose_skew_s"),
                    value_type=float,
                ),
                "receipt_pair_timeout_s": ParameterValue(
                    LaunchConfiguration("task2_gps_receipt_pair_timeout_s"),
                    value_type=float,
                ),
                "task2_pose_timeout_s": ParameterValue(
                    LaunchConfiguration("task2_pose_timeout_s"), value_type=float
                ),
                "task2_gps_fix_timeout_s": ParameterValue(
                    LaunchConfiguration("task2_gps_fix_timeout_s"), value_type=float
                ),
                "start_buoy_map_x": ParameterValue(
                    LaunchConfiguration("task2_start_buoy_map_x"), value_type=float
                ),
                "start_buoy_map_y": ParameterValue(
                    LaunchConfiguration("task2_start_buoy_map_y"), value_type=float
                ),
            }
        ],
        output="screen",
        condition=IfCondition(
            task2_fixture_mode
        ),
    )

    task2_pipeline_detector = Node(
        package="banyu_detection",
        executable="task2_pipeline_detector",
        name="task2_pipeline_detector",
        parameters=[
            {
                "image_topic": LaunchConfiguration("task2_detector_image_topic"),
                "camera_info_topic": LaunchConfiguration(
                    "task2_detector_camera_info_topic"
                ),
                "observation_topic": LaunchConfiguration(
                    "task2_detector_observation_topic"
                ),
                "ring_observation_topic": LaunchConfiguration(
                    "task2_detector_ring_observation_topic"
                ),
            }
        ],
        output="screen",
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("mission_mode"), "task2")
        ),
    )

    task2_vehicle_status = Node(
        package="banyu_vehicle_interface",
        executable="vehicle_status_node",
        name="banyu_vehicle_status",
        parameters=[
            {
                "state_topic": LaunchConfiguration("task2_vehicle_state_topic"),
                "rc_topic": LaunchConfiguration("task2_vehicle_rc_topic"),
                "gps_topic": LaunchConfiguration("task2_vehicle_gps_topic"),
                "status_topic": LaunchConfiguration("task2_vehicle_status_topic"),
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("task2_vehicle_status_publish_rate_hz"),
                    value_type=float,
                ),
                "fcu_timeout_s": ParameterValue(
                    LaunchConfiguration("task2_vehicle_fcu_timeout_s"), value_type=float
                ),
                "rc_timeout_s": ParameterValue(
                    LaunchConfiguration("task2_vehicle_rc_timeout_s"), value_type=float
                ),
                "gps_timeout_s": ParameterValue(
                    LaunchConfiguration("task2_vehicle_gps_timeout_s"), value_type=float
                ),
                "rc_takeover_channel": ParameterValue(
                    LaunchConfiguration("task2_vehicle_rc_takeover_channel"),
                    value_type=int,
                ),
                "rc_takeover_threshold": ParameterValue(
                    LaunchConfiguration("task2_vehicle_rc_takeover_threshold"),
                    value_type=int,
                ),
                "autonomy_mode": LaunchConfiguration("task2_vehicle_autonomy_mode"),
            }
        ],
        output="screen",
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("mission_mode"), "task2")
        ),
    )

    task2_ring_tracker = Node(
        package="banyu_detection",
        executable="ring_tracker_node",
        name="ring_tracker",
        parameters=[
            {
                "observation_topic": LaunchConfiguration(
                    "task2_detector_ring_observation_topic"
                ),
                "pose_topic": LaunchConfiguration("task2_ring_pose_topic"),
                "state_topic": LaunchConfiguration("task2_ring_state_topic"),
                "accepted_ring_topic": LaunchConfiguration("task2_accepted_ring_topic"),
                "repair_service": LaunchConfiguration("task2_ring_repair_service"),
            }
        ],
        output="screen",
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("mission_mode"), "task2")
        ),
    )

    task2_pipeline_guidance = Node(
        package="banyu_guidance",
        executable="pipeline_guidance_node",
        name="pipeline_guidance",
        parameters=[
            {
                "input_topic": LaunchConfiguration("task2_detector_observation_topic"),
                "output_topic": LaunchConfiguration("task2_guidance_output_topic"),
                "intent_frame": LaunchConfiguration("task2_guidance_intent_frame"),
                "surge_gain_mps": ParameterValue(
                    LaunchConfiguration("task2_guidance_surge_gain_mps"), value_type=float
                ),
                "max_surge_mps": ParameterValue(
                    LaunchConfiguration("task2_guidance_max_surge_mps"), value_type=float
                ),
                "sway_gain_mps": ParameterValue(
                    LaunchConfiguration("task2_guidance_sway_gain_mps"), value_type=float
                ),
                "max_sway_mps": ParameterValue(
                    LaunchConfiguration("task2_guidance_max_sway_mps"), value_type=float
                ),
                "yaw_gain_rad_s": ParameterValue(
                    LaunchConfiguration("task2_guidance_yaw_gain_rad_s"), value_type=float
                ),
                "max_yaw_rate_rad_s": ParameterValue(
                    LaunchConfiguration("task2_guidance_max_yaw_rate_rad_s"),
                    value_type=float,
                ),
                "min_confidence": ParameterValue(
                    LaunchConfiguration("task2_guidance_min_confidence"), value_type=float
                ),
                "max_observation_age_s": ParameterValue(
                    LaunchConfiguration("task2_guidance_max_observation_age_s"),
                    value_type=float,
                ),
                "max_future_skew_s": ParameterValue(
                    LaunchConfiguration("task2_guidance_max_future_skew_s"),
                    value_type=float,
                ),
                "yaw_control_enabled": ParameterValue(
                    LaunchConfiguration("task2_guidance_yaw_control_enabled"),
                    value_type=bool,
                ),
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("task2_guidance_publish_rate_hz"), value_type=float
                ),
            }
        ],
        output="screen",
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("mission_mode"), "task2")
        ),
    )

    task2_uav_report = Node(
        package="banyu_communication",
        executable="uav_report_node",
        name="uav_report",
        parameters=[
            {
                "action_name": LaunchConfiguration("task2_report_action_name"),
                "report_topic": LaunchConfiguration("task2_report_topic"),
                "ack_topic": LaunchConfiguration("task2_report_ack_topic"),
                "max_attempts": ParameterValue(
                    task2_report_max_attempts, value_type=int
                ),
                "retry_period_ms": ParameterValue(
                    LaunchConfiguration("task2_report_retry_period_ms"), value_type=int
                ),
            }
        ],
        output="screen",
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("mission_mode"), "task2")
        ),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_sim_arg,
            debug_arg,
            mission_mode_arg,
            task2_follow_source_arg,
            ring_colors_arg,
            publish_rate_arg,
            pipeline_standoff_m_arg,
            task2_gps_surface_depth_arg,
            task2_gps_surface_tolerance_arg,
            task2_pose_timeout_arg,
            task2_gps_receipt_pair_timeout_arg,
            task2_use_dvl_arg,
            task2_gps_fix_timeout_arg,
            task2_start_buoy_map_x_arg,
            task2_start_buoy_map_y_arg,
            task2_gps_max_fix_pose_skew_arg,
            task2_detector_image_topic_arg,
            task2_detector_camera_info_topic_arg,
            task2_detector_observation_topic_arg,
            task2_detector_ring_observation_topic_arg,
            task2_vehicle_state_topic_arg,
            task2_vehicle_rc_topic_arg,
            task2_vehicle_gps_topic_arg,
            task2_vehicle_status_topic_arg,
            task2_vehicle_status_publish_rate_arg,
            task2_vehicle_fcu_timeout_arg,
            task2_vehicle_rc_timeout_arg,
            task2_vehicle_gps_timeout_arg,
            task2_vehicle_rc_takeover_channel_arg,
            task2_vehicle_rc_takeover_threshold_arg,
            task2_vehicle_autonomy_mode_arg,
            task2_ring_pose_topic_arg,
            task2_ring_state_topic_arg,
            task2_accepted_ring_topic_arg,
            task2_ring_repair_service_arg,
            task2_guidance_output_topic_arg,
            task2_guidance_intent_frame_arg,
            task2_guidance_publish_rate_arg,
            task2_guidance_surge_gain_arg,
            task2_guidance_max_surge_arg,
            task2_guidance_sway_gain_arg,
            task2_guidance_max_sway_arg,
            task2_guidance_yaw_gain_arg,
            task2_guidance_max_yaw_rate_arg,
            task2_guidance_min_confidence_arg,
            task2_guidance_max_observation_age_arg,
            task2_guidance_max_future_skew_arg,
            task2_guidance_yaw_control_arg,
            task2_report_action_name_arg,
            task2_report_topic_arg,
            task2_report_ack_topic_arg,
            task2_report_retry_period_arg,
            task2_report_max_retries_arg,
            set_use_sim_time,
            mission_manager,
            detection_node,
            task2_sim_provider,
            task2_pipeline_detector,
            task2_vehicle_status,
            task2_ring_tracker,
            task2_pipeline_guidance,
            task2_uav_report,
        ]
    )
