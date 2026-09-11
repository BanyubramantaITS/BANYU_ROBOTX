#include "banyu_mission/bt_nodes/marker_orbit.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "banyu_mission/bt_nodes/common.hpp"

namespace banyu_mission {

MarkerOrbit::MarkerOrbit(const std::string& name, const BT::NodeConfig& config,
                         const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
    : BT::StatefulActionNode(name, config), node_(node), ctx_(std::move(ctx)) {}

BT::NodeStatus MarkerOrbit::onStart() {
  start_time_ = node_->now();
  accumulated_yaw_ = 0.0;
  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    pose = ctx_->current_pose;
  }
  start_yaw_ = bt_nodes::common::yawFromQuaternion(pose.pose.orientation);
  last_yaw_ = start_yaw_;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MarkerOrbit::onRunning() {
  const rclcpp::Time now = node_->now();

  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    return BT::NodeStatus::SUCCESS;
  }

  // Track the integrated yaw rotation: the orbit completes after a full
  // revolution, regardless of radius or speed.
  geometry_msgs::msg::PoseStamped pose;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    pose = ctx_->current_pose;
  }
  const double yaw = bt_nodes::common::yawFromQuaternion(pose.pose.orientation);
  accumulated_yaw_ += bt_nodes::common::wrapToPi(yaw - last_yaw_);
  last_yaw_ = yaw;

  if (std::fabs(accumulated_yaw_) >= node_->get_parameter("orbit_angle_rad").as_double()) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    RCLCPP_INFO(node_->get_logger(), "MarkerOrbit: full revolution (%.1f deg)",
                accumulated_yaw_ * 180.0 / M_PI);
    return BT::NodeStatus::SUCCESS;
  }

  if ((now - start_time_).seconds() > node_->get_parameter("orbit_timeout").as_double()) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    RCLCPP_ERROR(node_->get_logger(), "MarkerOrbit: timeout (rotated %.1f deg)",
                 accumulated_yaw_ * 180.0 / M_PI);
    return BT::NodeStatus::FAILURE;
  }

  banyu_msgs::msg::Detection detection;
  bool have_detection = false;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    if (ctx_->has_detection && ctx_->latest_detection.object_id == 1) {
      const double age = (now - ctx_->last_detection_received).seconds();
      if (age >= 0.0 && age <= node_->get_parameter("detection_timeout_s").as_double()) {
        detection = ctx_->latest_detection;
        have_detection = true;
      }
    }
  }

  if (!have_detection) {
    bt_nodes::common::publishZeroBodyVelocity(ctx_, now);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "MarkerOrbit: waiting for re-acquisition");
    return BT::NodeStatus::RUNNING;
  }

  // Strafe orbit: body-y is the circle speed, body-x locks the standoff
  // range, and yaw rate keeps the buoy centered in the camera while circling.
  const double distance = static_cast<double>(detection.distance);
  const double orbit_radius = node_->get_parameter("orbit_radius_m").as_double();
  const double forward = distance > 0.0f
                             ? std::clamp(node_->get_parameter("orbit_kp_range").as_double() *
                                              (distance - orbit_radius),
                                          -node_->get_parameter("orbit_max_range_speed").as_double(),
                                          node_->get_parameter("orbit_max_range_speed").as_double())
                             : 0.0;
  const double lateral = node_->get_parameter("orbit_speed").as_double();
  const double yaw_rate = std::clamp(
      node_->get_parameter("orbit_kp_yaw").as_double() * static_cast<double>(detection.offset_x),
      -node_->get_parameter("orbit_max_yaw_rate").as_double(),
      node_->get_parameter("orbit_max_yaw_rate").as_double());
  const double vertical = std::clamp(
      node_->get_parameter("orbit_kp_vertical").as_double() * static_cast<double>(detection.offset_y),
      -node_->get_parameter("orbit_max_vertical").as_double(),
      node_->get_parameter("orbit_max_vertical").as_double());

  ctx_->setpoint_pub->publish(bodyVelocityYawRateSetpoint(forward, lateral, vertical, yaw_rate, now));
  return BT::NodeStatus::RUNNING;
}

void MarkerOrbit::onHalted() {
  bt_nodes::common::publishZeroBodyVelocity(ctx_, node_->now());
}

}  // namespace banyu_mission
