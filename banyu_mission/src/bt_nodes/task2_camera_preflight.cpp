#include "banyu_mission/bt_nodes/task2_camera_preflight.hpp"

#include "banyu_mission/bt_nodes/common.hpp"
#include "banyu_mission/bt_nodes/task2_camera_common.hpp"

namespace banyu_mission {

Task2CameraPreflight::Task2CameraPreflight(const std::string& name,
                                           const BT::NodeConfig& config,
                                           const rclcpp::Node::SharedPtr& node,
                                           std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus Task2CameraPreflight::onStart() {
  start_time_ = node_->now();
  bt_nodes::task2_camera::publishZeroBody(ctx_, start_time_);

  std::lock_guard<std::mutex> lock(ctx_->data_mutex);
  if (!ctx_->task2_camera_source_selected || !ctx_->task2_camera_run.active) {
    RCLCPP_ERROR(node_->get_logger(), "Task2CameraPreflight: camera source was not selected");
    return BT::NodeStatus::FAILURE;
  }
  ctx_->task2_camera_run.baseline_set = false;
  ctx_->task2_camera_run.baseline_sequence = 0;
  ctx_->task2_camera_run.baseline_source_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  ctx_->task2_camera_run.ring_baseline_set = false;
  ctx_->task2_camera_run.ring_baseline_sequence = 0;
  ctx_->task2_camera_run.ring_baseline_source_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  ctx_->task2_camera_run.ring_heartbeat_seen = false;
  ctx_->task2_camera_run.last_heartbeat_sequence = 0;
  ctx_->task2_camera_run.last_heartbeat_source_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  ctx_->task2_camera_run.consumed_pipeline_sequence = 0;
  ctx_->task2_camera_run.consumed_pipeline_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  if (ctx_->has_pipeline_observation && ctx_->latest_pipeline_observation.frame_sequence != 0) {
    ctx_->task2_camera_run.baseline_set = true;
    ctx_->task2_camera_run.baseline_sequence = ctx_->last_pipeline_observation_sequence;
    ctx_->task2_camera_run.baseline_source_stamp = ctx_->last_pipeline_observation_source_stamp;
    ctx_->task2_camera_run.consumed_pipeline_sequence = ctx_->last_pipeline_observation_sequence;
    ctx_->task2_camera_run.consumed_pipeline_stamp = ctx_->last_pipeline_observation_source_stamp;
  }
  if (ctx_->has_pipeline_ring_observation &&
      ctx_->latest_pipeline_ring_observation.frame_sequence != 0) {
    ctx_->task2_camera_run.ring_baseline_set = true;
    ctx_->task2_camera_run.ring_baseline_sequence =
        ctx_->last_pipeline_ring_observation_sequence;
    ctx_->task2_camera_run.ring_baseline_source_stamp =
        ctx_->last_pipeline_ring_observation_source_stamp;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Task2CameraPreflight::onRunning() {
  const rclcpp::Time now = node_->now();
  bt_nodes::task2_camera::publishZeroBody(ctx_, now);

  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    return BT::NodeStatus::SUCCESS;
  }
  if ((now - start_time_).seconds() >=
      node_->get_parameter("task2_camera_preflight_timeout_s").as_double()) {
    RCLCPP_ERROR(node_->get_logger(), "Task2CameraPreflight: timeout waiting for a new frame");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped pose;
  mavros_msgs::msg::State state;
  bool ready = false;
  bt_nodes::task2_camera::PipelineFrame frame;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    if (ctx_->task2_camera_run.input_failure) {
      RCLCPP_ERROR(node_->get_logger(), "Task2CameraPreflight: %s",
                   ctx_->task2_camera_run.failure_reason.c_str());
      return BT::NodeStatus::FAILURE;
    }
    ready = bt_nodes::task2_common::dvlReadyLocked(
                *ctx_, now, node_->get_parameter("task2_use_dvl").as_bool(),
                node_->get_parameter("task2_dvl_timeout_s").as_double()) &&
            bt_nodes::task2_common::vehicleReady(
                *ctx_, now, node_->get_parameter("task2_pose_timeout_s").as_double(),
                node_->get_parameter("task2_state_timeout_s").as_double(), pose, state);
    const bool fresh = bt_nodes::task2_camera::freshPipelineLocked(
        *ctx_, now, node_->get_parameter("task2_camera_source_timeout_s").as_double(),
        node_->get_parameter("task2_camera_receipt_timeout_s").as_double(),
        node_->get_parameter("task2_camera_future_tolerance_s").as_double(), frame);
    if (!ready) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                            "Task2CameraPreflight: vehicle readiness lost");
      return BT::NodeStatus::FAILURE;
    }
    const bool ring_fresh = bt_nodes::task2_camera::freshRingHeartbeatLocked(
        *ctx_, now, node_->get_parameter("task2_camera_source_timeout_s").as_double(),
        node_->get_parameter("task2_camera_ring_heartbeat_timeout_s").as_double(),
        node_->get_parameter("task2_camera_future_tolerance_s").as_double());
    if (!fresh || !ring_fresh) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                            "Task2CameraPreflight: camera or ring heartbeat is missing or stale");
      return BT::NodeStatus::FAILURE;
    }
    if (!bt_nodes::task2_camera::sourceNewerThanBaselineLocked(*ctx_, frame) ||
        !bt_nodes::task2_camera::ringHeartbeatNewerThanBaselineLocked(*ctx_)) {
      return BT::NodeStatus::RUNNING;
    }
    ctx_->task2_camera_run.ring_heartbeat_seen = true;
    ctx_->task2_camera_run.last_heartbeat_sequence =
        ctx_->last_pipeline_ring_observation_sequence;
    ctx_->task2_camera_run.last_heartbeat_source_stamp =
        ctx_->last_pipeline_ring_observation_source_stamp;
  }

  RCLCPP_INFO(node_->get_logger(), "Task2CameraPreflight: accepted post-start camera baseline");
  return BT::NodeStatus::SUCCESS;
}

void Task2CameraPreflight::onHalted() {
  bt_nodes::task2_camera::publishZeroBody(ctx_, node_->now());
}

}  // namespace banyu_mission
