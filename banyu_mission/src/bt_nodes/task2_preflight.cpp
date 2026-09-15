#include "banyu_mission/bt_nodes/task2_preflight.hpp"

#include <algorithm>
#include <cmath>

#include "banyu_mission/bt_nodes/common.hpp"
#include "banyu_mission/bt_nodes/task2_common.hpp"

namespace banyu_mission {

Task2Preflight::Task2Preflight(const std::string& name, const BT::NodeConfig& config,
                               const rclcpp::Node::SharedPtr& node,
                               std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus Task2Preflight::onStart() {
  start_time_ = node_->now();
  bt_nodes::common::publishZeroVelocity(ctx_, start_time_);
  if (!node_->get_parameter("task2_fixture_mode").as_bool()) {
    RCLCPP_ERROR(node_->get_logger(), "Task2Preflight: fixture mode is disabled");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Task2Preflight::onRunning() {
  const rclcpp::Time now = node_->now();
  bt_nodes::common::publishZeroVelocity(ctx_, now);

  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    return BT::NodeStatus::SUCCESS;
  }

  if (!node_->get_parameter("task2_fixture_mode").as_bool()) {
    RCLCPP_ERROR(node_->get_logger(), "Task2Preflight: fixture mode is disabled");
    return BT::NodeStatus::FAILURE;
  }

  if ((now - start_time_).seconds() >= node_->get_parameter("task2_preflight_timeout_s").as_double()) {
    RCLCPP_ERROR(node_->get_logger(), "Task2Preflight: timeout");
    return BT::NodeStatus::FAILURE;
  }

  std::array<geometry_msgs::msg::PoseStamped, bt_nodes::task2_common::kWaypointCount> route;
  std::array<banyu_msgs::msg::PipelineRingState, bt_nodes::task2_common::kRingCount> rings;
  geometry_msgs::msg::PoseStamped pose;
  mavros_msgs::msg::State state;
  const bool dvl_required_ready = bt_nodes::task2_common::dvlReady(
      *ctx_, now, node_->get_parameter("task2_use_dvl").as_bool(),
      node_->get_parameter("task2_dvl_timeout_s").as_double());
  bool ready = false;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ready = dvl_required_ready && bt_nodes::task2_common::vehicleReady(*ctx_, now,
                              node_->get_parameter("task2_pose_timeout_s").as_double(),
                              node_->get_parameter("task2_state_timeout_s").as_double(), pose,
                              state) &&
            bt_nodes::task2_common::snapshotInputsLocked(
                *ctx_, now, node_->get_parameter("pipeline_route_timeout_s").as_double(),
                node_->get_parameter("pipeline_ring_state_timeout_s").as_double(),
                node_->get_parameter("task2_ring_confidence_min").as_double(),
                node_->get_parameter("task2_route_min_z").as_double(),
                node_->get_parameter("task2_route_max_z").as_double(),
                node_->get_parameter("task2_min_segment_spacing").as_double(),
                node_->get_parameter("task2_max_waypoint_spacing").as_double(),
                node_->get_parameter("task2_min_path_length").as_double(),
                node_->get_parameter("task2_max_path_length").as_double(), route, rings);
  }

  if (!ready) {
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(node_->get_logger(), "Task2Preflight: ready");
  return BT::NodeStatus::SUCCESS;
}

void Task2Preflight::onHalted() {
  bt_nodes::common::publishZeroVelocity(ctx_, node_->now());
}

}  // namespace banyu_mission
