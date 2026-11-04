#include "banyu_detection/ring_tracker_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

namespace banyu_detection
{

RingTrackerNode::RingTrackerNode(const rclcpp::NodeOptions & options)
: Node("ring_tracker", options)
{
  declareParameters();

  state_pub_ = create_publisher<banyu_msgs::msg::PipelineRingState>(state_topic_, ringStateQoS());
  accepted_ring_pub_ = create_publisher<banyu_msgs::msg::Task2RingRecord>(
    accepted_ring_topic_, acceptedRingQoS());
  rings_complete_pub_ = create_publisher<banyu_msgs::msg::PipelineEndCandidate>(
    rings_complete_topic_, rclcpp::QoS(1).reliable().transient_local());
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, rclcpp::SensorDataQoS(),
    std::bind(&RingTrackerNode::poseCallback, this, std::placeholders::_1));
  if (dead_reckoning_ || buoy_line_axis_) {
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      declare_parameter<std::string>("gps_fix_topic", "/mavros/global_position/global"), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr m) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        fix_valid_ = m->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
        std::isfinite(m->latitude) && std::isfinite(m->longitude);
        fix_time_ = now();
      });
    command_sub_ = create_subscription<mavros_msgs::msg::PositionTarget>(
      declare_parameter<std::string>("command_topic", "/banyu/execution/preview_target"), rclcpp::QoS(10),
      [this](const mavros_msgs::msg::PositionTarget::SharedPtr m) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        reckoner_.setCommand(m->velocity.x, m->velocity.y, rclcpp::Time(m->header.stamp).seconds());
      });
    buoy_sub_ = create_subscription<banyu_msgs::msg::Task2BuoyTarget>(
      declare_parameter<std::string>("buoy_target_topic", "/banyu/task2/buoy_target"),
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const banyu_msgs::msg::Task2BuoyTarget::SharedPtr m) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (m->heading_valid && std::isfinite(m->heading_enu_rad)) {
          buoy_heading_valid_ = true;
          buoy_heading_enu_ = m->heading_enu_rad;
        }
      });
  }
  if (pipe_height_enabled_) {
    pipe_sub_ = create_subscription<banyu_msgs::msg::PipelineObservation>(
      declare_parameter<std::string>("pipeline_observation_topic", "/banyu/pipeline/observation"),
      rclcpp::SensorDataQoS(), [this](const banyu_msgs::msg::PipelineObservation::SharedPtr m) {
        if (m->status != banyu_msgs::msg::PipelineObservation::TRACKED || !std::isfinite(m->width) ||
        m->width <= 0.0F) {return;}
        const double range = pipe_diameter_m_ / (m->width * 2.0 * std::tan(pipe_hfov_rad_ / 2.0));
        if (range < 0.3 || range > 4.0) {return;}
        std::lock_guard<std::mutex> lock(pose_mutex_);
        pipe_heights_.emplace_back(stampSeconds(m->header.stamp), range * std::cos(camera_tilt_rad_));
        while (pipe_heights_.size() > 9) {pipe_heights_.pop_front();}
      });
  }
  observation_sub_ = create_subscription<banyu_msgs::msg::PipelineRingObservation>(
    observation_topic_, rclcpp::SensorDataQoS(),
    std::bind(&RingTrackerNode::observationCallback, this, std::placeholders::_1));
  repair_service_server_ = create_service<AcknowledgeRingRepair>(
    repair_service_,
    std::bind(
      &RingTrackerNode::acknowledgeCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  reset_service_server_ = create_service<ResetTask2>(
    reset_service_,
    std::bind(&RingTrackerNode::resetCallback, this, std::placeholders::_1,
    std::placeholders::_2));
}

void RingTrackerNode::declareParameters()
{
  observation_topic_ = declare_parameter<std::string>(
    "observation_topic", "/banyu/pipeline/ring_observation");
  pose_topic_ = declare_parameter<std::string>(
    "pose_topic", "/mavros/local_position/pose");
  state_topic_ = declare_parameter<std::string>(
    "state_topic", "/banyu/pipeline/ring_state");
  accepted_ring_topic_ = declare_parameter<std::string>(
    "accepted_ring_topic", "/banyu/pipeline/accepted_ring");
  rings_complete_topic_ = declare_parameter<std::string>(
    "rings_complete_topic", "/banyu/pipeline/rings_complete");
  repair_service_ = declare_parameter<std::string>(
    "repair_service", "/banyu/pipeline/acknowledge_ring_repair");
  reset_service_ = declare_parameter<std::string>(
    "reset_service", "/banyu/pipeline/reset_tracker");
  require_run_context_ = declare_parameter<bool>("require_run_context", false);
  pose_timestamp_tolerance_s_ = declare_parameter<double>(
    "pose_timestamp_tolerance_s", 0.05);
  pose_history_size_ = static_cast<std::size_t>(std::max<int64_t>(
      2, declare_parameter<int64_t>("pose_history_size", 100)));

  RingTrackerConfig config;
  config.entry_confirm_frames = static_cast<std::size_t>(std::max<int64_t>(
      1, declare_parameter<int64_t>("entry_confirm_frames", 3)));
  config.exit_confirm_frames = static_cast<std::size_t>(std::max<int64_t>(
      1, declare_parameter<int64_t>("exit_confirm_frames", 2)));
  config.debounce_frames = static_cast<std::size_t>(std::max<int64_t>(
      1, declare_parameter<int64_t>("debounce_frames", 2)));
  config.minimum_confidence = declare_parameter<double>("minimum_confidence", 0.2);
  config.minimum_rearm_distance_m = declare_parameter<double>(
    "minimum_rearm_distance_m", 0.25);
  config.pose_timestamp_tolerance_s = pose_timestamp_tolerance_s_;
  config.maximum_sequence_gap = static_cast<uint64_t>(std::max<int64_t>(
      1, declare_parameter<int64_t>("maximum_sequence_gap", 1)));
  config.reject_duplicate_red = declare_parameter<bool>("reject_duplicate_red", true);
  config.camera_height_m = declare_parameter<double>("camera_height_m", 0.0);
  config.camera_forward_m = declare_parameter<double>("camera_forward_m", 0.0);
  config.lid_diameter_m = declare_parameter<double>("lid_diameter_m", 0.0);
  config.focal_ratio = declare_parameter<double>("focal_ratio", 0.0);
  config.camera_tilt_from_vertical_rad =
    declare_parameter<double>("camera_tilt_from_vertical_rad", 0.0);
  config.camera_half_vfov_rad = declare_parameter<double>("camera_half_vfov_rad", 0.0);
  config.node_association_distance_m = declare_parameter<double>(
    "node_association_distance_m", 0.8);
  config.min_node_spacing_m = declare_parameter<double>("min_node_spacing_m", 1.0);
  adapter_ = RingTrackerAdapter(config);

  pipe_height_enabled_ = declare_parameter<bool>("pipe_height.enabled", true);
  pipe_diameter_m_ = declare_parameter<double>("pipe_height.pipe_diameter_m", 0.0889);
  pipe_hfov_rad_ = declare_parameter<double>("pipe_height.hfov_rad", 1.047);
  camera_tilt_rad_ = config.camera_tilt_from_vertical_rad;
  const auto pose_source = declare_parameter<std::string>("pose_source", "ekf");
  const auto progress_axis = declare_parameter<std::string>("progress_axis", "centerline");
  if ((pose_source != "ekf" && pose_source != "dead_reckoning") ||
    (progress_axis != "centerline" && progress_axis != "buoy_line"))
  {
    throw std::invalid_argument("pose_source must be ekf or dead_reckoning, progress_axis centerline or buoy_line");
  }
  dead_reckoning_ = pose_source == "dead_reckoning";
  buoy_line_axis_ = progress_axis == "buoy_line";
  buoy_line_half_length_m_ = declare_parameter<double>("buoy_line_half_length_m", 30.0);
  reckoner_ = DeadReckoner(declare_parameter<double>("dead_reckoning.velocity_gain", 1.0));
  const auto centerline_points = declare_parameter<std::vector<double>>(
    "centerline_points", std::vector<double>{});
  // buoy_line: the pipe map (centerline) is not known; the axis is set at the dive from the buoys' bearing.
  if (!buoy_line_axis_ && centerline_points.size() >= 4 && centerline_points.size() % 2 == 0) {
    std::vector<std::pair<double, double>> waypoints;
    for (std::size_t i = 0; i < centerline_points.size(); i += 2) {
      waypoints.emplace_back(centerline_points[i], centerline_points[i + 1]);
    }
    adapter_.setCenterline(waypoints);
  }
}

double RingTrackerNode::pipeHeight(double t) const
{
  constexpr double kMaxAgeS = 1.0;
  std::vector<double> recent;
  for (const auto & sample : pipe_heights_) {
    if (std::abs(t - sample.first) <= kMaxAgeS) {recent.push_back(sample.second);}
  }
  if (recent.size() < 3) {return 0.0;}
  std::nth_element(recent.begin(), recent.begin() + recent.size() / 2, recent.end());
  return recent[recent.size() / 2];
}

double RingTrackerNode::stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

void RingTrackerNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (!dead_reckoning_ && !buoy_line_axis_) {
    pose_history_.push_back(*msg);
  } else {
    const double t = stampSeconds(msg->header.stamp);
    const bool gps_fresh = fix_valid_ && (now() - fix_time_).seconds() <= gps_fresh_s_;
    auto pose = *msg;
    if (dead_reckoning_) {
      if (gps_fresh) {
        reckoner_.reset(msg->pose.position.x, msg->pose.position.y, t);
      } else {
        reckoner_.advance(t);
      }
      if (reckoner_.valid()) {
        pose.pose.position.x = reckoner_.x();
        pose.pose.position.y = reckoner_.y();
      }
    }
    if (buoy_line_axis_ && !gps_fresh && !axis_set_ && buoy_heading_valid_) {
      // The dive point (the start buoy) is the origin; progress grows toward the other buoy.
      const double c = std::cos(buoy_heading_enu_), sn = std::sin(buoy_heading_enu_);
      const double x0 = pose.pose.position.x, y0 = pose.pose.position.y, l = buoy_line_half_length_m_;
      pending_axis_ = {{x0 - l * c, y0 - l * sn}, {x0 + l * c, y0 + l * sn}};
      axis_set_ = true;
      RCLCPP_INFO(get_logger(), "Ring progress axis from the buoys: origin (%.2f, %.2f), heading %.3f rad",
        x0, y0, buoy_heading_enu_);
    }
    pose_history_.push_back(pose);
  }
  while (pose_history_.size() > pose_history_size_) {
    pose_history_.pop_front();
  }
}

bool RingTrackerNode::nearestPose(
  const builtin_interfaces::msg::Time & stamp,
  geometry_msgs::msg::PoseStamped & pose) const
{
  const double source_seconds = stampSeconds(stamp);
  if (!std::isfinite(source_seconds) || (stamp.sec == 0 && stamp.nanosec == 0)) {
    return false;
  }

  double best_skew = std::numeric_limits<double>::max();
  bool found = false;
  std::lock_guard<std::mutex> lock(pose_mutex_);
  for (const auto & candidate : pose_history_) {
    const double skew = std::abs(stampSeconds(candidate.header.stamp) - source_seconds);
    if (candidate.header.frame_id == "map" && std::isfinite(skew) &&
      skew <= pose_timestamp_tolerance_s_ && skew < best_skew)
    {
      best_skew = skew;
      pose = candidate;
      found = true;
    }
  }
  return found;
}

void RingTrackerNode::observationCallback(
  const banyu_msgs::msg::PipelineRingObservation::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(adapter_mutex_);
  if (!run_context_.acceptsStamp(rclcpp::Time(msg->header.stamp).nanoseconds(),
    require_run_context_))
  {
    return;
  }
  {
    std::lock_guard<std::mutex> pose_lock(pose_mutex_);
    if (!pending_axis_.empty()) {
      adapter_.setCenterline(pending_axis_);
      pending_axis_.clear();
    }
  }
  RingTrackerInput input;
  input.run_id = run_context_.runId();
  input.epoch = run_context_.epoch();
  input.observation = *msg;
  input.has_vehicle_pose = nearestPose(msg->header.stamp, input.vehicle_pose_map);
  {
    std::lock_guard<std::mutex> pose_lock(pose_mutex_);
    input.camera_height_m = pipeHeight(stampSeconds(msg->header.stamp));
  }
  if (input.has_vehicle_pose && adapter_.hasCenterline()) {
    input.path_distance_m = adapter_.projectOntoCenterline(
      input.vehicle_pose_map.pose.position.x,
      input.vehicle_pose_map.pose.position.y);
    input.has_path_distance = true;
  }
  RingTrackerUpdate update = adapter_.update(input);
  if (!update.accepted) {
    return;
  }
  auto state = adapter_.tracker().snapshot().last_state;
  state_pub_->publish(state);
  if (update.merged) {
    RCLCPP_WARN(get_logger(), "Five distinct rings for four nodes: merged record %u into %u, slot %u reused",
      static_cast<unsigned>(update.record_index), update.merged_record.node_id,
      static_cast<unsigned>(update.record_index));
    accepted_ring_pub_->publish(update.merged_record);
  }
  accepted_ring_pub_->publish(update.record);
  publishRingsComplete();
}

void RingTrackerNode::publishRingsComplete()
{
  const auto & snapshot = adapter_.tracker().snapshot();
  banyu_msgs::msg::PipelineEndCandidate msg;
  msg.header.stamp = now();
  msg.run_id = run_context_.runId();
  msg.epoch = run_context_.epoch();
  msg.candidate = snapshot.records.size() >= 4 && !snapshot.red_pending;
  rings_complete_pub_->publish(msg);
}

void RingTrackerNode::resetCallback(
  const std::shared_ptr<ResetTask2::Request> request,
  std::shared_ptr<ResetTask2::Response> response)
{
  std::lock_guard<std::mutex> lock(adapter_mutex_);
  const auto result = run_context_.accept(request->run_id, request->epoch, now().nanoseconds());
  response->success = result.success;
  response->message = result.message;
  if (!result.changed) {
    return;
  }
  adapter_.reset();
  {
    std::lock_guard<std::mutex> pose_lock(pose_mutex_);
    pose_history_.clear();
    axis_set_ = false;
  }
  banyu_msgs::msg::PipelineRingState empty_state;
  empty_state.header.stamp = now();
  empty_state.run_id = request->run_id;
  empty_state.epoch = request->epoch;
  empty_state.color = banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN;
  state_pub_->publish(empty_state);
  publishRingsComplete();
}

void RingTrackerNode::acknowledgeCallback(
  const std::shared_ptr<AcknowledgeRingRepair::Request> request,
  std::shared_ptr<AcknowledgeRingRepair::Response> response)
{
  std::lock_guard<std::mutex> lock(adapter_mutex_);
  if (require_run_context_ && (!run_context_.initialized() ||
    request->run_id != run_context_.runId() || request->epoch != run_context_.epoch()))
  {
    response->success = false;
    response->reason = "repair acknowledgement has no matching run context";
    response->accepted_ring_index = kInvalidRingIndex;
    return;
  }
  const RepairAcknowledgementResult result =
    adapter_.acknowledgeRingRepair(request->ring_index);
  response->success = result.success;
  response->reason = result.reason;
  response->accepted_ring_index = result.accepted_ring_index;
  publishRingsComplete();
}

}  // namespace banyu_detection
