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
            set_use_sim_time,
            mission_manager,
            detection_node,
            task2_sim_provider,
            task2_pipeline_detector,
        ]
    )
