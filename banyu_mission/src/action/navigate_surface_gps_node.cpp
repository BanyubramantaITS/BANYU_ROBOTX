#include "banyu_mission/action/navigate_surface_gps_node.hpp"

#include <cmath>
#include <utility>

namespace banyu_mission
{

BT::PortsList NavigateSurfaceGpsNode::providedPorts()
{
  return {
    BT::InputPort<double>("target_latitude"), BT::InputPort<double>("target_longitude"),
    BT::InputPort<double>("start_latitude"), BT::InputPort<double>("start_longitude"),
    BT::InputPort<double>("tolerance"),
    BT::InputPort<double>("retarget_threshold_m", 1.0, "Target move that re-sends the goal")};
}

NavigateSurfaceGpsNode::NavigateSurfaceGpsNode(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node, const std::string & action)
: ActionClientNode(name, config, std::move(node), action)
{
}

NavigateSurfaceGpsNode::Goal NavigateSurfaceGpsNode::makeGoal()
{
  Goal goal;
  goal.target_latitude_deg = input<double>("target_latitude");
  goal.target_longitude_deg = input<double>("target_longitude");
  goal.start_latitude_deg = input<double>("start_latitude");
  goal.start_longitude_deg = input<double>("start_longitude");
  goal.tolerance_m = input<double>("tolerance");
  goal_latitude_ = goal.target_latitude_deg;
  goal_longitude_ = goal.target_longitude_deg;
  return goal;
}

BT::NodeStatus NavigateSurfaceGpsNode::onRunning()
{
  const auto latitude = getInput<double>("target_latitude");
  const auto longitude = getInput<double>("target_longitude");
  const auto threshold = getInput<double>("retarget_threshold_m");
  if (latitude && longitude && threshold && std::isfinite(latitude.value()) &&
    std::isfinite(longitude.value()))
  {
    constexpr double kMetresPerDegree = 111320.0;
    const double north = (latitude.value() - goal_latitude_) * kMetresPerDegree;
    const double east = (longitude.value() - goal_longitude_) * kMetresPerDegree *
      std::cos(goal_latitude_ * 3.14159265358979323846 / 180.0);
    if (std::hypot(north, east) > threshold.value()) {
      RCLCPP_INFO(
        node_->get_logger(), "%s target moved %.1f m; re-sending the goal",
        name().c_str(), std::hypot(north, east));
      onHalted();
      return onStart();
    }
  }
  return ActionClientNode::onRunning();
}

}  // namespace banyu_mission
