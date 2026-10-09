#include "banyu_detection/ring_tracker_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace banyu_detection
{

RingTrackerNode::RingTrackerNode(const rclcpp::NodeOptions & options)
: Node("ring_tracker", options)
{
  declareParameters();

  state_pub_ = create_publisher<banyu_msgs::msg::PipelineRingState>(state_topic_, ringStateQoS());
  accepted_ring_pub_ = create_publisher<banyu_msgs::msg::Task2RingRecord>(
    accepted_ring_topic_, acceptedRingQoS());
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, rclcpp::SensorDataQoS(),
    std::bind(&RingTrackerNode::poseCallback, this, std::placeholders::_1));
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
  adapter_ = RingTrackerAdapter(config);
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
  pose_history_.push_back(*msg);
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
  RingTrackerInput input;
  input.run_id = run_context_.runId();
  input.epoch = run_context_.epoch();
  input.observation = *msg;
  input.has_vehicle_pose = nearestPose(msg->header.stamp, input.vehicle_pose_map);
  RingTrackerUpdate update = adapter_.update(input);
  if (!update.accepted) {
    return;
  }
  auto state = adapter_.tracker().snapshot().last_state;
  state_pub_->publish(state);
  accepted_ring_pub_->publish(update.record);
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
  }
  banyu_msgs::msg::PipelineRingState empty_state;
  empty_state.header.stamp = now();
  empty_state.run_id = request->run_id;
  empty_state.epoch = request->epoch;
  empty_state.color = banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN;
  state_pub_->publish(empty_state);
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
}

}  // namespace banyu_detection
