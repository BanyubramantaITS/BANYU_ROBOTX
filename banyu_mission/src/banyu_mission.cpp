#include "banyu_mission/banyu_mission.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <random>
#include <stdexcept>

#include "banyu_mission/bt_nodes/aborted_or_teleop.hpp"
#include "banyu_mission/bt_nodes/acquire_task2_pipeline.hpp"
#include "banyu_mission/bt_nodes/follow_task2_pipeline.hpp"
#include "banyu_mission/bt_nodes/follow_and_record_task2_pipeline.hpp"
#include "banyu_mission/bt_nodes/gate_align.hpp"
#include "banyu_mission/bt_nodes/gate_pass.hpp"
#include "banyu_mission/bt_nodes/home_to_start_buoy.hpp"
#include "banyu_mission/bt_nodes/marker_align.hpp"
#include "banyu_mission/bt_nodes/marker_orbit.hpp"
#include "banyu_mission/bt_nodes/mission_active.hpp"
#include "banyu_mission/bt_nodes/nav_to_gate.hpp"
#include "banyu_mission/bt_nodes/repair_red_ring.hpp"
#include "banyu_mission/bt_nodes/return_gate_pass.hpp"
#include "banyu_mission/bt_nodes/reverse_to_red_ring.hpp"
#include "banyu_mission/bt_nodes/search_gate.hpp"
#include "banyu_mission/bt_nodes/search_marker.hpp"
#include "banyu_mission/bt_nodes/surface.hpp"
#include "banyu_mission/bt_nodes/surface_acquire_gps_anchor.hpp"
#include "banyu_mission/bt_nodes/surface_at_red_target.hpp"
#include "banyu_mission/bt_nodes/task2_common.hpp"
#include "banyu_mission/bt_nodes/task2_camera_preflight.hpp"
#include "banyu_mission/bt_nodes/task2_preflight.hpp"
#include "banyu_mission/bt_nodes/task2_report.hpp"
#include "banyu_mission/bt_nodes/task2_source.hpp"
#include "banyu_mission/bt_nodes/turn_around.hpp"

namespace banyu_mission {

namespace {

bool finiteInRange(double value, double minimum, double maximum) {
    return std::isfinite(value) && value >= minimum && value <= maximum;
}

rclcpp::Time safeSourceTime(const builtin_interfaces::msg::Time& stamp) {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000U ||
        (stamp.sec == 0 && stamp.nanosec == 0)) {
        return rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    return rclcpp::Time(stamp, RCL_ROS_TIME);
}

bool validCameraPipelinePayload(const banyu_msgs::msg::PipelineObservation& observation,
                               const std::string& expected_frame_id) {
    if (observation.header.frame_id != expected_frame_id || observation.frame_sequence == 0 ||
        observation.header.stamp.sec < 0 ||
        (observation.header.stamp.sec == 0 && observation.header.stamp.nanosec == 0) ||
        observation.header.stamp.nanosec >= 1000000000U) {
        return false;
    }
    if (observation.status == banyu_msgs::msg::PipelineObservation::TRACKED) {
        return finiteInRange(observation.offset_x, -1.0, 1.0) &&
               finiteInRange(observation.offset_y, -1.0, 1.0) &&
               finiteInRange(observation.heading_error_rad, -M_PI / 2.0, M_PI / 2.0) &&
               finiteInRange(observation.confidence, 0.0, 1.0);
    }
    if (observation.status != banyu_msgs::msg::PipelineObservation::NO_PIPE &&
        observation.status != banyu_msgs::msg::PipelineObservation::INVALID) {
        return false;
    }
    return (std::isnan(observation.offset_x) && std::isnan(observation.offset_y) &&
            std::isnan(observation.heading_error_rad) &&
            finiteInRange(observation.confidence, 0.0, 1.0));
}

bool validCameraRingPayload(const banyu_msgs::msg::PipelineRingObservation& observation,
                            const std::string& expected_frame_id) {
    if (observation.header.frame_id != expected_frame_id || observation.frame_sequence == 0 ||
        observation.header.stamp.sec < 0 ||
        (observation.header.stamp.sec == 0 && observation.header.stamp.nanosec == 0) ||
        observation.header.stamp.nanosec >= 1000000000U ||
        !finiteInRange(observation.confidence, 0.0, 1.0)) {
        return false;
    }
    if (!observation.visible) {
        return observation.color == banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN &&
               std::isnan(observation.offset_x) && std::isnan(observation.offset_y) &&
               observation.confidence == 0.0F;
    }
    return (observation.color == banyu_msgs::msg::PipelineRingObservation::COLOR_RED ||
            observation.color == banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN) &&
           finiteInRange(observation.offset_x, -1.0, 1.0) &&
           finiteInRange(observation.offset_y, -1.0, 1.0) &&
           observation.confidence > 0.0;
}

unique_identifier_msgs::msg::UUID makeTask2RunId() {
    unique_identifier_msgs::msg::UUID id;
    std::random_device random;
    bool all_zero = true;
    for (auto& byte : id.uuid) {
        byte = static_cast<uint8_t>(random());
        all_zero = all_zero && byte == 0;
    }
    if (all_zero) {
        id.uuid[0] = 1;
    }
    return id;
}

}  // namespace

BanyuMission::BanyuMission(const rclcpp::NodeOptions& options)
    : Node("banyu_mission", options), ctx_(std::make_shared<MissionContext>()) {
    declareParameters();
    ctx_->setpoint_pub = setpoint_pub_;
    ctx_->robocommand_pub = robocommand_pub_;
    ctx_->task2_report_pub = task2_report_pub_;
}

void BanyuMission::initialize() {
    setupBehaviorTree();
    RCLCPP_INFO(this->get_logger(), "BanyuMission node has been initialized.");
}

BanyuMission::~BanyuMission() {
    if (ctx_->setpoint_pub) {
        ctx_->setpoint_pub->publish(bodyVelocityYawRateSetpoint(0.0, 0.0, 0.0, 0.0, this->now()));
    }
    shutting_down_.store(true);
    const bool claimed = cancelTask2Terminal(ctx_);
    if (mission_mode_ == "task2" && task2_follow_source_ == "camera") {
        if (claimed) {
            publishTask2Report(
                ctx_, this->now(), banyu_msgs::msg::Task2Report::ABORTED,
                "Task 2 camera mission destroyed", task2_camera_source_timeout_s_,
                task2_camera_future_tolerance_s_, true);
        }
    }
    handoff_active_.store(false);
    {
        std::lock_guard<std::mutex> lock(handoff_mutex_);
        handoff_request_in_flight_.store(false);
        ++handoff_request_generation_;
    }
    if (handoff_timer_) {
        handoff_timer_->cancel();
    }
    if (tick_thread_.joinable()) {
        tick_thread_.join();
    }
    if (tree_.rootNode() != nullptr) {
        tree_.haltTree();
    }
    if (ctx_->setpoint_pub) {
        ctx_->setpoint_pub->publish(
            bodyVelocityYawRateSetpoint(0.0, 0.0, 0.0, 0.0, this->now()));
    }
}

void BanyuMission::declareParameters() {
    // Offsets are NED: east/north/down.
    this->declare_parameter<double>("gate_offset_north", 0.0);
    this->declare_parameter<double>("gate_offset_east", 0.0);
    this->declare_parameter<double>("gate_offset_down", 1.9);
    this->declare_parameter<double>("arrival_tolerance", 0.5);
    this->declare_parameter<double>("nav_timeout", 60.0);
    this->declare_parameter<double>("nav_kp_h", 0.5);
    this->declare_parameter<double>("nav_kp_v", 0.5);
    this->declare_parameter<double>("nav_max_speed_h", 0.5);
    this->declare_parameter<double>("nav_max_speed_v", 0.3);
    this->declare_parameter<double>("return_timeout", 60.0);
    detection_timeout_s_ = this->declare_parameter<double>("detection_timeout_s", 0.5);
    hydrophone_heading_timeout_s_ =
        this->declare_parameter<double>("hydrophone_heading_timeout_s", 0.5);
    pipeline_ring_state_timeout_s_ =
        this->declare_parameter<double>("pipeline_ring_state_timeout_s", 0.5);
    active_buoy_pose_timeout_s_ =
        this->declare_parameter<double>("active_buoy_pose_timeout_s", 0.5);
    pipeline_route_timeout_s_ =
        this->declare_parameter<double>("pipeline_route_timeout_s", 0.5);

    // GateAlign
    this->declare_parameter<double>("gate_align_timeout", 60.0);
    this->declare_parameter<double>("gate_align_deadband_x", 0.08);
    this->declare_parameter<double>("gate_align_deadband_y", 0.08);
    this->declare_parameter<double>("gate_align_gain_x", 0.6);
    this->declare_parameter<double>("gate_align_gain_y", 0.4);
    this->declare_parameter<double>("gate_align_max_speed_y", 0.3);
    this->declare_parameter<double>("gate_align_max_speed_z", 0.2);
    this->declare_parameter<double>("gate_align_hold_time", 0.5);

    // GatePass (forward) and ReturnGatePass (return leg)
    this->declare_parameter<double>("gate_pass_speed", 0.4);
    this->declare_parameter<double>("gate_pass_distance", 3.0);
    this->declare_parameter<double>("gate_pass_min_distance", 1.5);
    this->declare_parameter<double>("gate_pass_timeout", 60.0);

    // SearchMarker / SearchGate
    this->declare_parameter<double>("search_speed", 0.4);
    this->declare_parameter<double>("search_approach_distance", 2.5);
    this->declare_parameter<double>("search_min_distance", 1.8);
    this->declare_parameter<double>("search_timeout", 60.0);

    // MarkerAlign
    this->declare_parameter<double>("marker_align_timeout", 60.0);
    this->declare_parameter<double>("marker_align_deadband_x", 0.08);
    this->declare_parameter<double>("marker_align_deadband_y", 0.08);
    this->declare_parameter<double>("marker_align_gain_x", 0.6);
    this->declare_parameter<double>("marker_align_gain_y", 0.4);
    this->declare_parameter<double>("marker_align_max_speed_y", 0.3);
    this->declare_parameter<double>("marker_align_max_speed_z", 0.2);
    this->declare_parameter<double>("marker_align_hold_time", 0.5);

    // MarkerOrbit (strafe orbit: body-y circles, body-x locks range, yaw rate
    // keeps the buoy centered; completes after a full revolution by yaw)
    this->declare_parameter<double>("orbit_speed", 0.3);
    this->declare_parameter<double>("orbit_radius_m", 3.0);
    this->declare_parameter<double>("orbit_kp_range", 0.5);
    this->declare_parameter<double>("orbit_max_range_speed", 0.3);
    this->declare_parameter<double>("orbit_kp_yaw", 0.6);
    this->declare_parameter<double>("orbit_max_yaw_rate", 0.6);
    this->declare_parameter<double>("orbit_kp_vertical", 0.4);
    this->declare_parameter<double>("orbit_max_vertical", 0.2);
    this->declare_parameter<double>("orbit_angle_rad", 6.28318530718);
    this->declare_parameter<double>("orbit_timeout", 120.0);

    // ReturnToStart leg: turn, search gate, align, pass through, surface
    this->declare_parameter<double>("turn_yaw_rate", 0.3);
    this->declare_parameter<double>("turn_capture_gain", 0.5);
    this->declare_parameter<double>("turn_capture_deadband", 0.2);
    this->declare_parameter<double>("turn_angle_rad", 3.14159265359);
    this->declare_parameter<double>("turn_timeout", 30.0);
    this->declare_parameter<double>("gate_pass_lost_distance", 4.0);
    this->declare_parameter<double>("gate_pass_clearance_m", 0.6);
    this->declare_parameter<double>("surface_depth", -0.4);
    this->declare_parameter<double>("surface_tolerance", 0.2);
    this->declare_parameter<double>("surface_timeout", 30.0);

    this->declare_parameter<std::string>("abort_mode", "STABILIZE");
    this->declare_parameter<int>("groot2_port", 1666);
    mission_mode_ = this->declare_parameter<std::string>("mission_mode", "por");
    handoff_request_timeout_s_ =
        this->declare_parameter<double>("handoff_request_timeout_s", 1.0);
    this->declare_parameter<bool>("task2_fixture_mode", false);
    this->declare_parameter<double>("task2_preflight_timeout_s", 30.0);
    task2_pose_timeout_s_ = this->declare_parameter<double>("task2_pose_timeout_s", 3.0);
    task2_state_timeout_s_ = this->declare_parameter<double>("task2_state_timeout_s", 3.0);
    task2_dvl_timeout_s_ = this->declare_parameter<double>("task2_dvl_timeout_s", 0.5);
    task2_use_dvl_ = this->declare_parameter<bool>("task2_use_dvl", false);
    this->declare_parameter<double>("task2_ring_confidence_min", 0.5);
    this->declare_parameter<double>("task2_waypoint_horizontal_tolerance", 0.4);
    this->declare_parameter<double>("task2_waypoint_vertical_tolerance", 0.3);
    this->declare_parameter<double>("task2_waypoint_hold_s", 0.5);
    this->declare_parameter<double>("task2_max_waypoint_spacing", 5.0);
    this->declare_parameter<double>("task2_route_min_z", -16.0);
    this->declare_parameter<double>("task2_route_max_z", -11.0);
    this->declare_parameter<double>("task2_min_segment_spacing", 0.3);
    this->declare_parameter<double>("task2_min_path_length", 4.0);
    this->declare_parameter<double>("task2_max_path_length", 8.0);
    this->declare_parameter<double>("task2_segment_timeout_s", 60.0);
    this->declare_parameter<double>("task2_total_timeout_s", 360.0);
    this->declare_parameter<double>("task2_gps_fix_timeout_s", 1.0);
    this->declare_parameter<double>("task2_gps_surface_depth", -0.4);
    this->declare_parameter<double>("task2_gps_surface_tolerance", 0.5);
    this->declare_parameter<double>("task2_gps_max_fix_pose_skew_s", 0.5);
    this->declare_parameter<double>("task2_gps_receipt_pair_timeout_s", 3.0);
    this->declare_parameter<double>("task2_gps_max_buoy_range_m", 100.0);
    this->declare_parameter<double>("task2_gps_max_horizontal_covariance_m2", 25.0);
    this->declare_parameter<double>("task2_gps_anchor_timeout_s", 60.0);
    this->declare_parameter<double>("task2_home_kp_h", 0.5);
    this->declare_parameter<double>("task2_home_kp_v", 0.5);
    this->declare_parameter<double>("task2_home_max_speed_h", 0.5);
    this->declare_parameter<double>("task2_home_max_speed_v", 0.3);
    this->declare_parameter<double>("task2_home_horizontal_tolerance", 0.5);
    this->declare_parameter<double>("task2_home_hold_s", 0.5);
    this->declare_parameter<double>("task2_home_timeout_s", 60.0);
    task2_follow_source_ =
        this->declare_parameter<std::string>("task2_follow_source", "fixture_route");
    task2_camera_expected_frame_id_ = this->declare_parameter<std::string>(
        "task2_camera_expected_frame_id", "front_camera_link");
    task2_camera_source_timeout_s_ =
        this->declare_parameter<double>("task2_camera_source_timeout_s", 0.5);
    task2_camera_receipt_timeout_s_ =
        this->declare_parameter<double>("task2_camera_receipt_timeout_s", 0.5);
    task2_camera_future_tolerance_s_ =
        this->declare_parameter<double>("task2_camera_future_tolerance_s", 0.1);
    task2_pose_history_max_samples_ = static_cast<std::size_t>(
        this->declare_parameter<int>("task2_pose_history_max_samples", 500));
    task2_pose_history_duration_s_ =
        this->declare_parameter<double>("task2_pose_history_duration_s", 10.0);
    task2_camera_ring_cache_max_frames_ = static_cast<std::size_t>(
        this->declare_parameter<int>("task2_camera_ring_cache_max_frames", 64));
    this->declare_parameter<std::string>(
        "task2_pipeline_observation_topic", "/banyu/pipeline/observation");
    this->declare_parameter<std::string>(
        "task2_pipeline_ring_observation_topic", "/banyu/pipeline/ring_observation");
    this->declare_parameter<std::string>("task2_report_topic", "/banyu/task2/report");
    this->declare_parameter<double>("task2_camera_preflight_timeout_s", 30.0);
    this->declare_parameter<double>("task2_camera_acquire_timeout_s", 45.0);
    this->declare_parameter<double>("task2_camera_follow_timeout_s", 360.0);
    this->declare_parameter<double>("task2_camera_operating_depth", -12.45);
    this->declare_parameter<double>("task2_camera_depth_tolerance_m", 0.3);
    this->declare_parameter<double>("task2_camera_depth_kp", 0.4);
    this->declare_parameter<double>("task2_camera_max_depth_speed", 0.3);
    this->declare_parameter<double>("task2_camera_depth_timeout_s", 60.0);
    this->declare_parameter<int>("task2_camera_acquire_required_frames", 3);
    this->declare_parameter<double>("task2_camera_acquire_min_confidence", 0.35);
    this->declare_parameter<double>("task2_camera_search_yaw_rate", 0.15);
    this->declare_parameter<int>("task2_camera_loss_required_frames", 3);
    this->declare_parameter<double>("task2_camera_loss_persistence_s", 0.3);
    this->declare_parameter<int>("task2_camera_required_ring_count", 5);
    this->declare_parameter<double>("task2_camera_min_path_length_m", 1.0);
    this->declare_parameter<double>("task2_camera_min_travel_after_last_ring_m", 1.0);
    this->declare_parameter<double>("task2_camera_path_sample_spacing_m", 0.2);
    this->declare_parameter<int>("task2_camera_max_path_samples", 2000);
    this->declare_parameter<double>("task2_camera_max_path_duration_s", 600.0);
    this->declare_parameter<double>("task2_camera_max_path_length_m", 100.0);
    this->declare_parameter<double>("task2_camera_max_segment_jump_m", 3.0);
    this->declare_parameter<double>("task2_camera_pose_association_skew_s", 0.25);
    this->declare_parameter<int>("task2_camera_ring_confirmation_frames", 2);
    this->declare_parameter<double>("task2_camera_ring_min_travel_spacing_m", 0.5);
    this->declare_parameter<double>("task2_camera_ring_min_confidence", 0.5);
    this->declare_parameter<int>("task2_camera_ring_invisible_required_frames", 2);
    this->declare_parameter<double>("task2_camera_ring_heartbeat_timeout_s", 0.5);
    this->declare_parameter<double>("task2_camera_ring_heartbeat_loss_tolerance_s", 0.5);
    this->declare_parameter<int>("task2_camera_max_ring_records", 16);
    this->declare_parameter<double>("task2_camera_forward_speed", 0.2);
    this->declare_parameter<double>("task2_camera_max_forward_speed", 0.3);
    this->declare_parameter<double>("task2_camera_lateral_kp", 0.35);
    this->declare_parameter<double>("task2_camera_heading_kp", 0.35);
    this->declare_parameter<double>("task2_camera_max_lateral_speed", 0.25);
    this->declare_parameter<double>("task2_camera_max_yaw_rate", 0.3);
    this->declare_parameter<double>("task2_camera_forward_lateral_limit", 0.35);
    this->declare_parameter<double>("task2_camera_forward_heading_limit", 0.35);
    this->declare_parameter<double>("task2_camera_reverse_timeout_s", 180.0);
    this->declare_parameter<double>("task2_camera_reverse_reacquire_timeout_s", 30.0);
    this->declare_parameter<double>("task2_camera_reverse_pair_timeout_s", 0.5);
    this->declare_parameter<double>("task2_camera_reverse_kp_h", 0.5);
    this->declare_parameter<double>("task2_camera_reverse_kp_v", 0.5);
    this->declare_parameter<double>("task2_camera_reverse_max_speed_h", 0.4);
    this->declare_parameter<double>("task2_camera_reverse_max_speed_v", 0.3);
    this->declare_parameter<double>("task2_camera_reverse_horizontal_tolerance_m", 0.35);
    this->declare_parameter<double>("task2_camera_reverse_vertical_tolerance_m", 0.3);
    this->declare_parameter<double>("task2_camera_reverse_max_pose_jump_m", 1.0);
    this->declare_parameter<double>("task2_camera_reverse_max_start_error_m", 1.5);
    this->declare_parameter<double>("task2_camera_reverse_max_path_jump_m", 3.0);
    this->declare_parameter<double>("task2_camera_red_reacquire_min_confidence", 0.5);
    this->declare_parameter<double>("task2_camera_red_reacquire_min_pipeline_confidence", 0.35);
    this->declare_parameter<double>("task2_camera_red_reacquire_offset_deadband", 0.25);
    this->declare_parameter<int>("task2_camera_red_reacquire_required_frames", 3);
    this->declare_parameter<double>("task2_camera_repair_duration_s", 30.0);
    this->declare_parameter<double>("task2_camera_repair_timeout_s", 60.0);
    this->declare_parameter<double>("task2_camera_repair_kp_h", 0.5);
    this->declare_parameter<double>("task2_camera_repair_kp_v", 0.5);
    this->declare_parameter<double>("task2_camera_repair_max_speed_h", 0.2);
    this->declare_parameter<double>("task2_camera_repair_max_speed_v", 0.2);
    this->declare_parameter<double>("task2_camera_repair_horizontal_tolerance_m", 0.25);
    this->declare_parameter<double>("task2_camera_repair_vertical_tolerance_m", 0.25);
    this->declare_parameter<double>("task2_camera_repair_max_pose_jump_m", 1.0);
    this->declare_parameter<double>("task2_camera_surface_depth", -0.4);
    this->declare_parameter<double>("task2_camera_surface_timeout_s", 90.0);
    this->declare_parameter<double>("task2_camera_surface_kp_h", 0.5);
    this->declare_parameter<double>("task2_camera_surface_kp_v", 0.5);
    this->declare_parameter<double>("task2_camera_surface_max_speed_h", 0.3);
    this->declare_parameter<double>("task2_camera_surface_max_speed_v", 0.25);
    this->declare_parameter<double>("task2_camera_surface_horizontal_tolerance_m", 0.4);
    this->declare_parameter<double>("task2_camera_surface_vertical_tolerance_m", 0.25);
    this->declare_parameter<double>("task2_camera_surface_max_pose_jump_m", 1.0);
    validateTask2GpsParameters();
    validateTask2FollowParameters();

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "mavros/local_position/pose", rclcpp::SensorDataQoS(),
        std::bind(&BanyuMission::onPose, this, std::placeholders::_1));
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 10, std::bind(&BanyuMission::onState, this, std::placeholders::_1));
    detection_sub_ = this->create_subscription<banyu_msgs::msg::Detection>(
        "banyu/detection", rclcpp::SensorDataQoS(), std::bind(&BanyuMission::onDetection, this, std::placeholders::_1));
    hydrophone_heading_sub_ = this->create_subscription<banyu_msgs::msg::HydrophoneHeading>(
        "/banyu/hydrophone/relative_heading", rclcpp::SensorDataQoS(),
        std::bind(&BanyuMission::onHydrophoneHeading, this, std::placeholders::_1));
    pipeline_observation_sub_ = this->create_subscription<banyu_msgs::msg::PipelineObservation>(
        this->get_parameter("task2_pipeline_observation_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&BanyuMission::onPipelineObservation, this, std::placeholders::_1));
    pipeline_ring_observation_sub_ =
        this->create_subscription<banyu_msgs::msg::PipelineRingObservation>(
            this->get_parameter("task2_pipeline_ring_observation_topic").as_string(),
            rclcpp::SensorDataQoS(),
            std::bind(&BanyuMission::onPipelineRingObservation, this, std::placeholders::_1));
    pipeline_ring_state_sub_ = this->create_subscription<banyu_msgs::msg::PipelineRingState>(
        "/banyu/pipeline/ring_state", rclcpp::SensorDataQoS(),
        std::bind(&BanyuMission::onPipelineRingState, this, std::placeholders::_1));
    active_buoy_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/banyu/active_buoy_pose", rclcpp::SensorDataQoS(),
        std::bind(&BanyuMission::onActiveBuoyPose, this, std::placeholders::_1));
    pipeline_route_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/banyu/pipeline/route", rclcpp::SensorDataQoS(),
        std::bind(&BanyuMission::onPipelineRoute, this, std::placeholders::_1));
    if (task2_use_dvl_) {
        dvl_live_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/banyu/dvl/live", 10,
            std::bind(&BanyuMission::onDvlLive, this, std::placeholders::_1));
    }
    auv_global_fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/mavros/global_position/global", rclcpp::SensorDataQoS(),
        std::bind(&BanyuMission::onAuvGlobalFix, this, std::placeholders::_1));
    start_buoy_fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/banyu/task2/start_buoy_fix",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        std::bind(&BanyuMission::onStartBuoyFix, this, std::placeholders::_1));

    detection_watchdog_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), [this]() {
        std::lock_guard<std::mutex> lock(ctx_->data_mutex);
        const rclcpp::Time now = this->now();
        if (ctx_->has_detection && ctx_->last_detection_received.nanoseconds() != 0 &&
            (now - ctx_->last_detection_received).seconds() > detection_timeout_s_) {
            ctx_->has_detection = false;
        }
        if (ctx_->has_pose && ctx_->last_pose_received.nanoseconds() != 0 &&
            (now - ctx_->last_pose_received).seconds() > task2_pose_timeout_s_) {
            ctx_->has_pose = false;
        }
        if (ctx_->has_state && ctx_->last_state_received.nanoseconds() != 0 &&
            (now - ctx_->last_state_received).seconds() > task2_state_timeout_s_) {
            ctx_->has_state = false;
        }
        if (ctx_->has_hydrophone_heading &&
            ctx_->last_hydrophone_heading_received.nanoseconds() != 0 &&
            (now - ctx_->last_hydrophone_heading_received).seconds() >
                hydrophone_heading_timeout_s_) {
            ctx_->has_hydrophone_heading = false;
        }
        if (!ctx_->pipeline_ring_states.empty()) {
            for (auto ring_it = ctx_->pipeline_ring_states.begin();
                 ring_it != ctx_->pipeline_ring_states.end();) {
                const auto receipt_it = ctx_->pipeline_ring_state_receipt_times.find(ring_it->first);
                if (receipt_it == ctx_->pipeline_ring_state_receipt_times.end() ||
                    receipt_it->second.nanoseconds() == 0 ||
                    (now - receipt_it->second).seconds() > pipeline_ring_state_timeout_s_) {
                    ctx_->pipeline_ring_state_receipt_times.erase(ring_it->first);
                    ring_it = ctx_->pipeline_ring_states.erase(ring_it);
                } else {
                    ++ring_it;
                }
            }
        }
        ctx_->has_pipeline_ring_state = !ctx_->pipeline_ring_states.empty();
        if (ctx_->has_active_buoy_pose && ctx_->last_active_buoy_pose_received.nanoseconds() != 0 &&
            (now - ctx_->last_active_buoy_pose_received).seconds() > active_buoy_pose_timeout_s_) {
            ctx_->has_active_buoy_pose = false;
        }
        if (ctx_->has_pipeline_route && ctx_->last_pipeline_route_received.nanoseconds() != 0 &&
            (now - ctx_->last_pipeline_route_received).seconds() > pipeline_route_timeout_s_) {
            ctx_->has_pipeline_route = false;
        }
        if (task2_use_dvl_ && ctx_->has_dvl_live &&
            ctx_->last_dvl_live_received.nanoseconds() != 0 &&
            (now - ctx_->last_dvl_live_received).seconds() > task2_dvl_timeout_s_) {
          ctx_->has_dvl_live = false;
        }
    });

    setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 10);
    robocommand_pub_ = this->create_publisher<std_msgs::msg::String>("/banyu/robocommand", 10);
    task2_report_pub_ = this->create_publisher<banyu_msgs::msg::Task2Report>(
        this->get_parameter("task2_report_topic").as_string(),
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    start_service_ = this->create_service<std_srvs::srv::SetBool>(
        "banyu/mission/start", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                      std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
            (void)req;
            std::string failure_message;
            res->success = startMission(failure_message);
            res->message = res->success ? "mission started" : failure_message;
        });

    abort_service_ = this->create_service<std_srvs::srv::SetBool>(
        "banyu/mission/abort", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                      std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
            (void)req;
            abortMission();
            res->success = true;
            res->message = "mission aborted";
        });

    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    handoff_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&BanyuMission::onHandoffTimer, this));
    handoff_timer_->cancel();
}

void BanyuMission::validateTask2GpsParameters() const {
    const auto value = [this](const char* name) { return this->get_parameter(name).as_double(); };
    const auto requireFinite = [value](const char* name) {
        if (!std::isfinite(value(name))) {
            throw std::invalid_argument(std::string(name) + " must be finite");
        }
    };
    const auto requirePositive = [value](const char* name) {
        if (!std::isfinite(value(name)) || value(name) <= 0.0) {
            throw std::invalid_argument(std::string(name) + " must be finite and positive");
        }
    };
    const auto requireNonnegative = [value](const char* name) {
        if (!std::isfinite(value(name)) || value(name) < 0.0) {
            throw std::invalid_argument(std::string(name) + " must be finite and nonnegative");
        }
    };

    requireFinite("task2_gps_surface_depth");
    for (const char* name : {"task2_gps_fix_timeout_s", "task2_gps_surface_tolerance",
                             "task2_gps_max_fix_pose_skew_s",
                             "task2_gps_receipt_pair_timeout_s",
                             "task2_gps_max_buoy_range_m",
                             "task2_gps_max_horizontal_covariance_m2", "task2_gps_anchor_timeout_s",
                             "task2_home_max_speed_h",
                             "task2_home_max_speed_v", "task2_home_horizontal_tolerance",
                             "task2_home_timeout_s"}) {
        requirePositive(name);
    }
    for (const char* name : {"task2_home_kp_h", "task2_home_kp_v", "task2_home_hold_s"}) {
        requireNonnegative(name);
    }
}

void BanyuMission::validateTask2FollowParameters() const {
    if (task2_follow_source_ != "fixture_route" && task2_follow_source_ != "camera") {
        throw std::invalid_argument(
            "task2_follow_source must be exactly fixture_route or camera");
    }
    if (task2_camera_expected_frame_id_.empty()) {
        throw std::invalid_argument("task2_camera_expected_frame_id must not be empty");
    }
    if (this->get_parameter("task2_report_topic").as_string().empty()) {
        throw std::invalid_argument("task2_report_topic must not be empty");
    }
    const auto positive = [this](const char* name) {
        const double value = this->get_parameter(name).as_double();
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument(std::string(name) + " must be finite and positive");
        }
    };
    const auto nonnegative = [this](const char* name) {
        const double value = this->get_parameter(name).as_double();
        if (!std::isfinite(value) || value < 0.0) {
            throw std::invalid_argument(std::string(name) + " must be finite and nonnegative");
        }
    };
    for (const char* name : {"task2_camera_source_timeout_s", "task2_camera_receipt_timeout_s",
                             "task2_camera_preflight_timeout_s", "task2_camera_acquire_timeout_s",
                             "task2_camera_follow_timeout_s", "task2_camera_max_path_duration_s",
                             "task2_camera_max_path_length_m", "task2_camera_max_segment_jump_m",
                             "task2_camera_pose_association_skew_s", "task2_camera_ring_min_travel_spacing_m",
                             "task2_camera_depth_tolerance_m", "task2_camera_max_depth_speed",
                             "task2_camera_depth_timeout_s", "task2_camera_ring_heartbeat_timeout_s",
                             "task2_camera_ring_heartbeat_loss_tolerance_s",
                             "task2_camera_max_forward_speed", "task2_camera_max_lateral_speed",
                             "task2_camera_max_yaw_rate"}) {
        positive(name);
    }
    for (const char* name : {"task2_camera_future_tolerance_s", "task2_camera_loss_persistence_s",
                             "task2_camera_min_path_length_m",
                             "task2_camera_min_travel_after_last_ring_m",
                             "task2_camera_path_sample_spacing_m", "task2_camera_forward_speed",
                             "task2_camera_lateral_kp", "task2_camera_heading_kp",
                             "task2_camera_depth_kp"}) {
        nonnegative(name);
    }
    if (this->get_parameter("task2_pose_history_max_samples").as_int() <= 0 ||
        this->get_parameter("task2_camera_ring_cache_max_frames").as_int() <= 0 ||
        this->get_parameter("task2_camera_max_path_samples").as_int() <= 0 ||
        this->get_parameter("task2_camera_max_ring_records").as_int() <= 0 ||
        this->get_parameter("task2_camera_acquire_required_frames").as_int() <= 0 ||
        this->get_parameter("task2_camera_loss_required_frames").as_int() <= 0 ||
        this->get_parameter("task2_camera_required_ring_count").as_int() <= 0 ||
        this->get_parameter("task2_camera_ring_confirmation_frames").as_int() <= 0 ||
        this->get_parameter("task2_camera_ring_invisible_required_frames").as_int() <= 0 ||
        this->get_parameter("task2_camera_required_ring_count").as_int() >
            this->get_parameter("task2_camera_max_ring_records").as_int()) {
        throw std::invalid_argument(
            "Task 2 camera count limits must be positive and required rings must fit the record bound");
    }
    for (const char* name : {"task2_camera_reverse_timeout_s",
                             "task2_camera_reverse_reacquire_timeout_s",
                             "task2_camera_reverse_pair_timeout_s",
                             "task2_camera_reverse_max_speed_h",
                             "task2_camera_reverse_max_speed_v",
                             "task2_camera_reverse_horizontal_tolerance_m",
                             "task2_camera_reverse_vertical_tolerance_m",
                             "task2_camera_reverse_max_pose_jump_m",
                             "task2_camera_reverse_max_start_error_m",
                             "task2_camera_reverse_max_path_jump_m"}) {
        positive(name);
    }
    for (const char* name : {"task2_camera_reverse_kp_h", "task2_camera_reverse_kp_v"}) {
        nonnegative(name);
    }
    for (const char* name : {"task2_camera_repair_duration_s", "task2_camera_repair_timeout_s",
                             "task2_camera_repair_max_speed_h", "task2_camera_repair_max_speed_v",
                             "task2_camera_repair_horizontal_tolerance_m",
                             "task2_camera_repair_vertical_tolerance_m",
                             "task2_camera_repair_max_pose_jump_m",
                             "task2_camera_surface_timeout_s", "task2_camera_surface_max_speed_h",
                             "task2_camera_surface_max_speed_v",
                             "task2_camera_surface_horizontal_tolerance_m",
                             "task2_camera_surface_vertical_tolerance_m",
                             "task2_camera_surface_max_pose_jump_m"}) {
        positive(name);
    }
    for (const char* name : {"task2_camera_repair_kp_h", "task2_camera_repair_kp_v",
                             "task2_camera_surface_kp_h", "task2_camera_surface_kp_v"}) {
        nonnegative(name);
    }
    if (this->get_parameter("task2_camera_red_reacquire_required_frames").as_int() <= 0) {
        throw std::invalid_argument("task2_camera_red_reacquire_required_frames must be positive");
    }
    if (this->get_parameter("task2_camera_repair_duration_s").as_double() >
        this->get_parameter("task2_camera_repair_timeout_s").as_double()) {
        throw std::invalid_argument("repair duration must not exceed repair timeout");
    }
    const double surface_depth = this->get_parameter("task2_camera_surface_depth").as_double();
    if (!std::isfinite(surface_depth) || surface_depth > 0.0) {
        throw std::invalid_argument("task2_camera_surface_depth must be finite and at or below surface");
    }
    const double operating_depth = this->get_parameter("task2_camera_operating_depth").as_double();
    if (!std::isfinite(operating_depth) || operating_depth >= 0.0) {
        throw std::invalid_argument("task2_camera_operating_depth must be finite and below surface");
    }
    if (!std::isfinite(task2_pose_history_duration_s_) || task2_pose_history_duration_s_ <= 0.0) {
        throw std::invalid_argument("task2_pose_history_duration_s must be finite and positive");
    }
    const auto bounded = [this](const char* name, double min_value, double max_value) {
        const double value = this->get_parameter(name).as_double();
        if (!std::isfinite(value) || value < min_value || value > max_value) {
            throw std::invalid_argument(std::string(name) + " is outside its valid range");
        }
    };
    bounded("task2_camera_acquire_min_confidence", 0.0, 1.0);
    bounded("task2_camera_ring_min_confidence", 0.0, 1.0);
    bounded("task2_camera_forward_lateral_limit", 0.0, 1.0);
    bounded("task2_camera_forward_heading_limit", 0.0, M_PI / 2.0);
    bounded("task2_camera_search_yaw_rate", -1.0, 1.0);
    bounded("task2_camera_red_reacquire_min_confidence", 0.0, 1.0);
    bounded("task2_camera_red_reacquire_min_pipeline_confidence", 0.0, 1.0);
    bounded("task2_camera_red_reacquire_offset_deadband", 0.0, 1.0);
}

void BanyuMission::clearTask2GpsAnchor() {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->task2_anchor_buoy_fix = sensor_msgs::msg::NavSatFix();
    ctx_->task2_anchor_auv_fix = sensor_msgs::msg::NavSatFix();
    ctx_->task2_anchor_map_pose = geometry_msgs::msg::PoseStamped();
    ctx_->task2_target_map_pose = geometry_msgs::msg::Point();
    ctx_->has_task2_surface_anchor = false;
}

void BanyuMission::setupBehaviorTree() {
    factory_.registerNodeType<AbortedOrTeleop>("AbortedOrTeleop", ctx_);
    factory_.registerNodeType<MissionActive>("MissionActive", ctx_);
    factory_.registerNodeType<NavToGate>("NavToGate", this->shared_from_this(), ctx_);
    factory_.registerNodeType<GateAlign>("GateAlign", this->shared_from_this(), ctx_);
    factory_.registerNodeType<GatePass>("GatePass", this->shared_from_this(), ctx_);
    factory_.registerNodeType<SearchMarker>("SearchMarker", this->shared_from_this(), ctx_);
    factory_.registerNodeType<MarkerAlign>("MarkerAlign", this->shared_from_this(), ctx_);
    factory_.registerNodeType<MarkerOrbit>("MarkerOrbit", this->shared_from_this(), ctx_);
    factory_.registerNodeType<TurnAround>("TurnAround", this->shared_from_this(), ctx_);
    factory_.registerNodeType<SearchGate>("SearchGate", this->shared_from_this(), ctx_);
    factory_.registerNodeType<ReturnGatePass>("ReturnGatePass", this->shared_from_this(), ctx_);
    factory_.registerNodeType<Surface>("Surface", this->shared_from_this(), ctx_);
    factory_.registerNodeType<Task2Preflight>("Task2Preflight", this->shared_from_this(), ctx_);
    factory_.registerNodeType<FollowTask2Pipeline>("FollowTask2Pipeline", this->shared_from_this(), ctx_);
    factory_.registerNodeType<Task2CameraSourceSelected>(
        "Task2CameraSourceSelected", ctx_);
    factory_.registerNodeType<Task2CameraPreflight>(
        "Task2CameraPreflight", this->shared_from_this(), ctx_);
    factory_.registerNodeType<AcquireTask2Pipeline>(
        "AcquireTask2Pipeline", this->shared_from_this(), ctx_);
    factory_.registerNodeType<FollowAndRecordTask2Pipeline>(
        "FollowAndRecordTask2Pipeline", this->shared_from_this(), ctx_);
    factory_.registerNodeType<ReverseToRedRing>(
        "ReverseToRedRing", this->shared_from_this(), ctx_);
    factory_.registerNodeType<RepairRedRing>(
        "RepairRedRing", this->shared_from_this(), ctx_);
    factory_.registerNodeType<SurfaceAtRedTarget>(
        "SurfaceAtRedTarget", this->shared_from_this(), ctx_);
    factory_.registerNodeType<Task2ReportNode>(
        "Task2Report", this->shared_from_this(), ctx_);
    factory_.registerNodeType<SurfaceAcquireGpsAnchor>(
        "SurfaceAcquireGpsAnchor", this->shared_from_this(), ctx_);
    factory_.registerNodeType<HomeToStartBuoy>(
        "HomeToStartBuoy", this->shared_from_this(), ctx_);

    const std::filesystem::path share_dir = ament_index_cpp::get_package_share_directory("banyu_mission");
    const std::filesystem::path tree_name =
        mission_mode_ == "task2" ? "mission_tree_task2.xml" : "mission_tree.xml";
    const std::filesystem::path tree_file = share_dir / "bt" / tree_name;

    factory_.registerBehaviorTreeFromFile(tree_file);
    tree_ = factory_.createTree("BanyuMission");
    tree_.initialize();

    auto blackboard = tree_.rootBlackboard();
    blackboard->set("mission_active", false);
    blackboard->set("aborted", false);
    blackboard->set("gate_offset_north", this->get_parameter("gate_offset_north").as_double());
    blackboard->set("gate_offset_east", this->get_parameter("gate_offset_east").as_double());
    blackboard->set("gate_offset_down", this->get_parameter("gate_offset_down").as_double());

    const int groot2_port = this->get_parameter("groot2_port").as_int();
    groot2_publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, groot2_port);
    RCLCPP_INFO(this->get_logger(), "BehaviorTree 'BanyuMission' loaded, Groot2 on port %d", groot2_port);
}

bool BanyuMission::startMission(std::string& failure_message) {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (shutting_down_.load()) {
        failure_message = "node is shutting down";
        return false;
    }
    if (handoff_active_.load()) {
        failure_message = "handoff in progress";
        return false;
    }
    if (ctx_->mission_active.load()) {
        failure_message = "mission already active";
        return false;
    }
    if (mission_mode_ == "task2") {
        const std::string source = this->get_parameter("task2_follow_source").as_string();
        if (source != "fixture_route" && source != "camera") {
            failure_message = "task2_follow_source must be exactly fixture_route or camera";
            return false;
        }
        if (source == "fixture_route" && !this->get_parameter("task2_fixture_mode").as_bool()) {
            failure_message = "task2 fixture mode is disabled";
            return false;
        }
        task2_follow_source_ = source;
    }

    if (tick_thread_.joinable()) {
        tick_thread_.join();
    }

    {
        std::lock_guard<std::mutex> terminal_lock(ctx_->task2_terminal_mutex);
        ctx_->mission_active.store(false);
        ctx_->aborted.store(false);
        ctx_->task2_terminal_claim.store(banyu_msgs::msg::Task2Report::STATUS_UNKNOWN);
    }
    clearTask2GpsAnchor();
    {
        std::lock_guard<std::mutex> lock(ctx_->data_mutex);
        ctx_->mission_origin.reset();
        ctx_->gate_target.reset();
        ctx_->task2_camera_run = MissionContext::Task2CameraRunState{};
        ctx_->task2_run_id = makeTask2RunId();
        ctx_->task2_run_id_valid = mission_mode_ == "task2";
        ctx_->task2_report_published = false;
        ctx_->task2_repair_status = banyu_msgs::msg::Task2Report::NOT_ATTEMPTED;
        ctx_->task2_repair_hold_duration_s = 0.0;
        ctx_->task2_repair_started = false;
        ctx_->has_task2_final_surface_pose = false;
        ctx_->task2_camera_source_selected =
            mission_mode_ == "task2" && task2_follow_source_ == "camera";
        ctx_->task2_camera_run.active = ctx_->task2_camera_source_selected;
    }

    armAndSetGuided();

    ctx_->mission_active.store(true);
    tick_thread_ = std::thread([this]() {
        const BT::NodeStatus status = tree_.tickWhileRunning(std::chrono::milliseconds(20));
        if (status == BT::NodeStatus::FAILURE) {
            {
                std::lock_guard<std::mutex> terminal_lock(ctx_->task2_terminal_mutex);
                ctx_->aborted.store(true);
                ctx_->mission_active.store(false);
            }
            if (mission_mode_ == "task2" && task2_follow_source_ == "camera") {
                std::string detail = "Task 2 camera mission failed";
                {
                    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
                    if (!ctx_->task2_camera_run.failure_reason.empty()) {
                        detail = ctx_->task2_camera_run.failure_reason;
                    }
                    if (ctx_->task2_repair_started &&
                        ctx_->task2_repair_status !=
                            banyu_msgs::msg::Task2Report::HOLD_COMPLETED_UNVERIFIED) {
                        ctx_->task2_repair_status = banyu_msgs::msg::Task2Report::REPAIR_FAILED;
                    }
                }
                publishTask2Report(ctx_, this->now(), banyu_msgs::msg::Task2Report::FAILED, detail,
                                   task2_camera_source_timeout_s_,
                                   task2_camera_future_tolerance_s_);
            }
            beginHandoff();
        }
        if (status == BT::NodeStatus::SUCCESS && mission_mode_ == "task2") {
            if (task2_follow_source_ == "camera" && !shutting_down_.load() &&
                ctx_->mission_active.load() && !ctx_->aborted.load()) {
                const bool claimed = claimTask2Terminal(
                    ctx_, banyu_msgs::msg::Task2Report::COMPLETED);
                if (claimed) {
                    publishTask2Report(
                        ctx_, this->now(), banyu_msgs::msg::Task2Report::COMPLETED,
                        "Task 2 camera mission completed", task2_camera_source_timeout_s_,
                        task2_camera_future_tolerance_s_, true);
                }
            }
            {
                std::lock_guard<std::mutex> terminal_lock(ctx_->task2_terminal_mutex);
                ctx_->mission_active.store(false);
                if (ctx_->task2_terminal_claim.load() !=
                    banyu_msgs::msg::Task2Report::ABORTED) {
                    ctx_->aborted.store(false);
                }
            }
            beginHandoff();
        }
        {
            std::lock_guard<std::mutex> terminal_lock(ctx_->task2_terminal_mutex);
            ctx_->mission_active.store(false);
            if (ctx_->task2_terminal_claim.load() != banyu_msgs::msg::Task2Report::ABORTED) {
                ctx_->aborted.store(false);
            }
        }
        {
            std::lock_guard<std::mutex> lock(ctx_->data_mutex);
            ctx_->task2_camera_run.active = false;
        }
        if (status == BT::NodeStatus::SUCCESS) {
            if (mission_mode_ == "task2") {
                RCLCPP_INFO(this->get_logger(),
                            "Task 2 mission finished (SUCCESS); handoff command published.");
            } else {
                RCLCPP_INFO(this->get_logger(), "Mission tree finished (SUCCESS). Handing control back.");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Mission tree finished (FAILURE). Handing control back.");
        }
    });
    return true;
}

void BanyuMission::abortMission() {
    if (ctx_->setpoint_pub) {
        ctx_->setpoint_pub->publish(
            bodyVelocityYawRateSetpoint(0.0, 0.0, 0.0, 0.0, this->now()));
    }
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (!ctx_->mission_active.load() && !handoff_active_.load()) {
        return;
    }

    if (ctx_->setpoint_pub) {
        ctx_->setpoint_pub->publish(
            bodyVelocityYawRateSetpoint(0.0, 0.0, 0.0, 0.0, this->now()));
    }
    bool terminal_claimed = false;
    if (mission_mode_ == "task2" && task2_follow_source_ == "camera") {
        terminal_claimed = cancelTask2Terminal(ctx_);
        {
            std::lock_guard<std::mutex> lock(ctx_->data_mutex);
            if (ctx_->task2_repair_started &&
                ctx_->task2_repair_status !=
                    banyu_msgs::msg::Task2Report::HOLD_COMPLETED_UNVERIFIED) {
                ctx_->task2_repair_status = banyu_msgs::msg::Task2Report::REPAIR_FAILED;
            }
        }
        if (terminal_claimed) {
            publishTask2Report(
                ctx_, this->now(), banyu_msgs::msg::Task2Report::ABORTED,
                "Task 2 camera mission aborted", task2_camera_source_timeout_s_,
                task2_camera_future_tolerance_s_, true);
        }
    } else {
        cancelTask2Terminal(ctx_);
    }
    {
        std::lock_guard<std::mutex> lock(ctx_->data_mutex);
        ctx_->task2_camera_run.active = false;
    }
    RCLCPP_WARN(this->get_logger(), "Mission aborted by request.");

    beginHandoff();

    if (tick_thread_.joinable()) {
        tick_thread_.join();
    }
}

void BanyuMission::beginHandoff() {
    if (shutting_down_.load()) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(handoff_mutex_);
        handoff_active_.store(true);
        handoff_request_in_flight_.store(false);
        ++handoff_request_generation_;
    }

    if (ctx_->setpoint_pub) {
        ctx_->setpoint_pub->publish(velocitySetpoint(0.0, 0.0, 0.0, this->now()));
    }
    if (handoff_timer_) {
        handoff_timer_->reset();
    }
}

void BanyuMission::onHandoffTimer() {
    if (shutting_down_.load() || !handoff_active_.load()) {
        return;
    }

    const rclcpp::Time now = this->now();
    std::string current_mode;
    {
        std::lock_guard<std::mutex> lock(ctx_->data_mutex);
        current_mode = ctx_->state.mode;
    }

    if (ctx_->setpoint_pub) {
        ctx_->setpoint_pub->publish(velocitySetpoint(0.0, 0.0, 0.0, now));
    }

    const std::string target_mode = this->get_parameter("abort_mode").as_string();
    if (current_mode == target_mode) {
        {
            std::lock_guard<std::mutex> lock(handoff_mutex_);
            handoff_active_.store(false);
            handoff_request_in_flight_.store(false);
            ++handoff_request_generation_;
        }
        handoff_timer_->cancel();
        return;
    }

    const auto wall_now = std::chrono::steady_clock::now();
    uint64_t request_generation = 0;
    {
        std::lock_guard<std::mutex> lock(handoff_mutex_);
        if (!handoff_active_.load()) {
            return;
        }
        if (handoff_request_in_flight_.load() && wall_now < handoff_request_deadline_) {
            return;
        }
        if (handoff_request_in_flight_.load()) {
            handoff_request_in_flight_.store(false);
            ++handoff_request_generation_;
        }
        if (!set_mode_client_->service_is_ready()) {
            return;
        }

        request_generation = ++handoff_request_generation_;
        handoff_request_in_flight_.store(true);
        const auto timeout = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(std::max(0.05, handoff_request_timeout_s_)));
        handoff_request_deadline_ = wall_now + timeout;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = target_mode;
    set_mode_client_->async_send_request(
        request, [this, request_generation](
                     rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
            (void)future;
            if (shutting_down_.load()) {
                return;
            }
            std::lock_guard<std::mutex> lock(handoff_mutex_);
            if (request_generation == handoff_request_generation_) {
                handoff_request_in_flight_.store(false);
            }
        });
}

void BanyuMission::armAndSetGuided() {
    auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_req->value = true;
    if (arming_client_->wait_for_service(std::chrono::seconds(1))) {
        arming_client_->async_send_request(
            arm_req, [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                const auto res = future.get();
                if (res->success) {
                    RCLCPP_INFO(this->get_logger(), "Vehicle armed");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Arm command failed (result=%d)", res->result);
                }
            });
    } else {
        RCLCPP_WARN(this->get_logger(), "Arming service not available");
    }

    auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    mode_req->custom_mode = "GUIDED";
    if (set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
        set_mode_client_->async_send_request(mode_req,
                                             [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                                                 const auto res = future.get();
                                                 if (res->mode_sent) {
                                                     RCLCPP_INFO(this->get_logger(), "Mode set to GUIDED");
                                                 } else {
                                                     RCLCPP_ERROR(this->get_logger(), "Failed to set GUIDED mode");
                                                 }
                                             });
    } else {
        RCLCPP_WARN(this->get_logger(), "SetMode service not available");
    }
}

void BanyuMission::onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!msg) {
        return;
    }
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->current_pose = *msg;
    ctx_->has_pose = true;
    const rclcpp::Time receipt_time = this->now();
    ctx_->last_pose_received = receipt_time;
    if (msg->header.frame_id == "map" && bt_nodes::task2_common::finitePose(*msg) &&
        msg->header.stamp.sec >= 0 &&
        !(msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)) {
        ctx_->map_pose_history.push_back({*msg, receipt_time});
        while (ctx_->map_pose_history.size() > task2_pose_history_max_samples_) {
            ctx_->map_pose_history.pop_front();
        }
        while (!ctx_->map_pose_history.empty() &&
               (receipt_time - ctx_->map_pose_history.front().receipt_time).seconds() >
                   task2_pose_history_duration_s_) {
            ctx_->map_pose_history.pop_front();
        }
    }
}

void BanyuMission::onState(const mavros_msgs::msg::State::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->state = *msg;
    ctx_->has_state = true;
    ctx_->last_state_received = this->now();
}

void BanyuMission::onDetection(const banyu_msgs::msg::Detection::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->latest_detection = *msg;
    ctx_->has_detection = true;
    ctx_->last_detection_received = this->now();
}

void BanyuMission::onHydrophoneHeading(const banyu_msgs::msg::HydrophoneHeading::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->latest_hydrophone_heading = *msg;
    ctx_->has_hydrophone_heading = true;
    ctx_->last_hydrophone_heading_received = this->now();
}

void BanyuMission::onPipelineObservation(
    const banyu_msgs::msg::PipelineObservation::SharedPtr msg) {
    if (!msg) {
        return;
    }
    const rclcpp::Time receipt_time = this->now();
    const rclcpp::Time source_stamp = safeSourceTime(msg->header.stamp);
    const bool contract_valid =
        validCameraPipelinePayload(*msg, task2_camera_expected_frame_id_);
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    if (ctx_->task2_camera_run.active) {
        if ((ctx_->pipeline_observation_sequence_seen &&
             msg->frame_sequence <= ctx_->last_pipeline_observation_sequence) ||
            (ctx_->pipeline_observation_sequence_seen &&
             source_stamp.nanoseconds() <= ctx_->last_pipeline_observation_source_stamp.nanoseconds())) {
            ctx_->task2_camera_run.input_failure = true;
            ctx_->task2_camera_run.failure_reason = "pipeline sequence or source stamp regressed";
        }
        if (!contract_valid) {
            ctx_->task2_camera_run.input_failure = true;
            ctx_->task2_camera_run.failure_reason = "invalid pipeline observation contract";
        }
    }
    ctx_->latest_pipeline_observation = *msg;
    ctx_->has_pipeline_observation = true;
    ctx_->last_pipeline_observation_received = receipt_time;
    ctx_->pipeline_observation_contract_valid = contract_valid;
    ctx_->last_pipeline_observation_sequence = msg->frame_sequence;
    ctx_->last_pipeline_observation_source_stamp = source_stamp;
    ctx_->pipeline_observation_sequence_seen = true;
}

void BanyuMission::onPipelineRingObservation(
    const banyu_msgs::msg::PipelineRingObservation::SharedPtr msg) {
    if (!msg) {
        return;
    }
    const rclcpp::Time receipt_time = this->now();
    const rclcpp::Time source_stamp = safeSourceTime(msg->header.stamp);
    const bool contract_valid = validCameraRingPayload(*msg, task2_camera_expected_frame_id_);
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    if (ctx_->task2_camera_run.active) {
        if ((ctx_->pipeline_ring_observation_sequence_seen &&
             msg->frame_sequence <= ctx_->last_pipeline_ring_observation_sequence) ||
            (ctx_->pipeline_ring_observation_sequence_seen &&
             source_stamp.nanoseconds() <=
                 ctx_->last_pipeline_ring_observation_source_stamp.nanoseconds())) {
            ctx_->task2_camera_run.input_failure = true;
            ctx_->task2_camera_run.failure_reason = "ring sequence or source stamp regressed";
        }
        if (!contract_valid) {
            ctx_->task2_camera_run.input_failure = true;
            ctx_->task2_camera_run.failure_reason = "invalid ring observation contract";
        }
    }
    ctx_->pipeline_ring_observation_cache[msg->frame_sequence] =
        MissionContext::CameraRingFrame{*msg, receipt_time, contract_valid};
    while (ctx_->pipeline_ring_observation_cache.size() > task2_camera_ring_cache_max_frames_) {
        ctx_->pipeline_ring_observation_cache.erase(
            ctx_->pipeline_ring_observation_cache.begin());
    }
    ctx_->latest_pipeline_ring_observation = *msg;
    ctx_->has_pipeline_ring_observation = !ctx_->pipeline_ring_observation_cache.empty();
    ctx_->pipeline_ring_observation_contract_valid = contract_valid;
    ctx_->last_pipeline_ring_observation_received = receipt_time;
    ctx_->last_pipeline_ring_observation_sequence = msg->frame_sequence;
    ctx_->last_pipeline_ring_observation_source_stamp = source_stamp;
    ctx_->pipeline_ring_observation_sequence_seen = true;
}

void BanyuMission::onPipelineRingState(const banyu_msgs::msg::PipelineRingState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    const rclcpp::Time receipt_time = this->now();
    ctx_->latest_pipeline_ring_state = *msg;
    ctx_->has_pipeline_ring_state = true;
    ctx_->last_pipeline_ring_state_received = receipt_time;
    ctx_->pipeline_ring_states[msg->index] = *msg;
    ctx_->pipeline_ring_state_receipt_times[msg->index] = receipt_time;
}

void BanyuMission::onActiveBuoyPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->active_buoy_pose = *msg;
    ctx_->has_active_buoy_pose = true;
    ctx_->last_active_buoy_pose_received = this->now();
}

void BanyuMission::onPipelineRoute(const nav_msgs::msg::Path::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->latest_pipeline_route = *msg;
    ctx_->has_pipeline_route = true;
    ctx_->last_pipeline_route_received = this->now();
}

void BanyuMission::onDvlLive(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->dvl_live = msg->data;
    ctx_->has_dvl_live = true;
    ctx_->last_dvl_live_received = this->now();
}

void BanyuMission::onAuvGlobalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->latest_task2_auv_fix = *msg;
    ctx_->last_task2_auv_fix_received = this->now();
}

void BanyuMission::onStartBuoyFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->latest_task2_buoy_fix = *msg;
    ctx_->last_task2_buoy_fix_received = this->now();
}

}  // namespace banyu_mission

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<banyu_mission::BanyuMission>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
