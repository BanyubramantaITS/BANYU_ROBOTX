#include "banyu_mission/bt_nodes/search_marker.hpp"

#include <algorithm>
#include <utility>

#include "banyu_mission/bt_nodes/common.hpp"

namespace banyu_mission {

SearchMarker::SearchMarker(const std::string& name, const BT::NodeConfig& config,
                           const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus SearchMarker::onStart() {
  start_time_ = node_->now();
  seen_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SearchMarker::onRunning() {
  const rclcpp::Time now = node_->now();

  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }

  const double detection_timeout = node_->get_parameter("detection_timeout_s").as_double();
  const double approach_distance = node_->get_parameter("search_approach_distance").as_double();
  const double min_distance = node_->get_parameter("search_min_distance").as_double();

  if (detectionIsFresh(*ctx_, now, detection_timeout, 1)) {
    seen_ = true;

    banyu_msgs::msg::Detection detection;
    {
      std::lock_guard<std::mutex> lock(ctx_->data_mutex);
      detection = ctx_->latest_detection;
    }

    const double distance = static_cast<double>(detection.distance);
    if (distance > 0.0f) {
      if (distance < min_distance) {
        bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
        RCLCPP_ERROR(node_->get_logger(), "SearchMarker: too close (%.2f m), aborting", distance);
        return BT::NodeStatus::FAILURE;
      }
      if (distance <= approach_distance) {
        bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
        RCLCPP_INFO(node_->get_logger(), "SearchMarker: approach complete at %.2f m", distance);
        return BT::NodeStatus::SUCCESS;
      }
    }

    // Approach while keeping the buoy centered: forward + lateral/vertical
    // corrections from the image offsets (same convention as MarkerAlign).
    const double lateral = std::clamp(
        node_->get_parameter("marker_align_gain_x").as_double() *
            static_cast<double>(detection.offset_x),
        -node_->get_parameter("marker_align_max_speed_y").as_double(),
        node_->get_parameter("marker_align_max_speed_y").as_double());
    const double vertical = std::clamp(
        node_->get_parameter("marker_align_gain_y").as_double() *
            static_cast<double>(detection.offset_y),
        -node_->get_parameter("marker_align_max_speed_z").as_double(),
        node_->get_parameter("marker_align_max_speed_z").as_double());

    ctx_->setpoint_pub->publish(bodyVelocitySetpoint(
        node_->get_parameter("search_speed").as_double(), lateral, vertical, now));
  } else if (seen_) {
    // Lost the buoy mid-approach: hold position and wait for re-acquisition
    // rather than driving forward blindly.
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "SearchMarker: waiting for re-acquisition");
  } else {
    // Initial sweep: slow forward drive while scanning for the yellow buoy.
    ctx_->setpoint_pub->publish(
        bodyVelocitySetpoint(node_->get_parameter("search_speed").as_double(), 0.0, 0.0, now));
  }

  if ((now - start_time_).seconds() > node_->get_parameter("search_timeout").as_double()) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "SearchMarker: timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void SearchMarker::onHalted() {
  bt_nodes::common::publishZeroBodyVelocity(ctx_, node_->now());
}

}  // namespace banyu_mission
