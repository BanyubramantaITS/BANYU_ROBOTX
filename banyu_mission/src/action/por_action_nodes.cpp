#include "banyu_mission/action/por_action_nodes.hpp"

#include <utility>

namespace banyu_mission
{

#define BANYU_POR_CONSTRUCTOR(ClassName, ActionType) \
  ClassName::ClassName( \
    const std::string & name, const BT::NodeConfig & config, \
    rclcpp::Node::SharedPtr node, const std::string & action) \
    : ActionClientNode<ActionType>(name, config, std::move(node), action) {}

#define BANYU_POR_PORTS(ClassName, ...) \
  BT::PortsList ClassName::providedPorts() {return {__VA_ARGS__};}

BANYU_POR_CONSTRUCTOR(AcquireGateNode, banyu_msgs::action::AcquireGate)
BANYU_POR_PORTS(
  AcquireGateNode,
  BT::InputPort<double>("target_depth_m"),
  BT::InputPort<double>("depth_tolerance_m"),
  BT::InputPort<double>("timeout_s"))
banyu_msgs::action::AcquireGate::Goal AcquireGateNode::makeGoal()
{
  Goal goal;
  goal.target_depth_m = input<double>("target_depth_m");
  goal.depth_tolerance_m = input<double>("depth_tolerance_m");
  goal.timeout_s = input<double>("timeout_s");
  return goal;
}

BANYU_POR_CONSTRUCTOR(SearchMarkerNode, banyu_msgs::action::SearchMarker)
BANYU_POR_PORTS(
  SearchMarkerNode,
  BT::InputPort<double>("search_speed_mps"),
  BT::InputPort<double>("timeout_s"))
banyu_msgs::action::SearchMarker::Goal SearchMarkerNode::makeGoal()
{
  Goal goal;
  goal.search_speed_mps = input<double>("search_speed_mps");
  goal.timeout_s = input<double>("timeout_s");
  return goal;
}

BANYU_POR_CONSTRUCTOR(SearchGateNode, banyu_msgs::action::SearchGate)
BANYU_POR_PORTS(
  SearchGateNode,
  BT::InputPort<double>("search_speed_mps"),
  BT::InputPort<double>("timeout_s"))
banyu_msgs::action::SearchGate::Goal SearchGateNode::makeGoal()
{
  Goal goal;
  goal.search_speed_mps = input<double>("search_speed_mps");
  goal.timeout_s = input<double>("timeout_s");
  return goal;
}

BANYU_POR_CONSTRUCTOR(AlignGateNode, banyu_msgs::action::AlignGate)
BANYU_POR_PORTS(
  AlignGateNode,
  BT::InputPort<double>("deadband_x"), BT::InputPort<double>("deadband_y"),
  BT::InputPort<double>("gain_x"), BT::InputPort<double>("gain_y"),
  BT::InputPort<double>("max_speed_y"), BT::InputPort<double>("max_speed_z"),
  BT::InputPort<double>("hold_time_s"), BT::InputPort<double>("timeout_s"))
banyu_msgs::action::AlignGate::Goal AlignGateNode::makeGoal()
{
  Goal goal;
  goal.deadband_x = input<double>("deadband_x");
  goal.deadband_y = input<double>("deadband_y");
  goal.gain_x = input<double>("gain_x");
  goal.gain_y = input<double>("gain_y");
  goal.max_speed_y = input<double>("max_speed_y");
  goal.max_speed_z = input<double>("max_speed_z");
  goal.hold_time_s = input<double>("hold_time_s");
  goal.timeout_s = input<double>("timeout_s");
  return goal;
}

BANYU_POR_CONSTRUCTOR(AlignMarkerNode, banyu_msgs::action::AlignMarker)
BANYU_POR_PORTS(
  AlignMarkerNode,
  BT::InputPort<double>("deadband_x"), BT::InputPort<double>("deadband_y"),
  BT::InputPort<double>("gain_x"), BT::InputPort<double>("gain_y"),
  BT::InputPort<double>("max_speed_y"), BT::InputPort<double>("max_speed_z"),
  BT::InputPort<double>("minimum_distance_m"), BT::InputPort<double>("standoff_min_m"),
  BT::InputPort<double>("standoff_max_m"),
  BT::InputPort<double>("hold_time_s"),
  BT::InputPort<double>("timeout_s"))
banyu_msgs::action::AlignMarker::Goal AlignMarkerNode::makeGoal()
{
  Goal goal;
  goal.deadband_x = input<double>("deadband_x");
  goal.deadband_y = input<double>("deadband_y");
  goal.gain_x = input<double>("gain_x");
  goal.gain_y = input<double>("gain_y");
  goal.max_speed_y = input<double>("max_speed_y");
  goal.max_speed_z = input<double>("max_speed_z");
  goal.minimum_distance_m = input<double>("minimum_distance_m");
  goal.standoff_min_m = input<double>("standoff_min_m");
  goal.standoff_max_m = input<double>("standoff_max_m");
  goal.hold_time_s = input<double>("hold_time_s");
  goal.timeout_s = input<double>("timeout_s");
  return goal;
}

BANYU_POR_CONSTRUCTOR(PassGateNode, banyu_msgs::action::PassGate)
BANYU_POR_PORTS(
  PassGateNode,
  BT::InputPort<bool>("return_leg"), BT::InputPort<double>("speed_mps"),
  BT::InputPort<double>("target_distance_m"), BT::InputPort<double>("minimum_distance_m"),
  BT::InputPort<double>("blind_duration_s"),
  BT::InputPort<double>("timeout_s"))
banyu_msgs::action::PassGate::Goal PassGateNode::makeGoal()
{
  Goal goal;
  goal.return_leg = input<bool>("return_leg");
  goal.speed_mps = input<double>("speed_mps");
  goal.target_distance_m = input<double>("target_distance_m");
  goal.minimum_distance_m = input<double>("minimum_distance_m");
  goal.blind_duration_s = input<double>("blind_duration_s");
  goal.timeout_s = input<double>("timeout_s");
  return goal;
}

BANYU_POR_CONSTRUCTOR(OrbitMarkerNode, banyu_msgs::action::OrbitMarker)
BANYU_POR_PORTS(
  OrbitMarkerNode,
  BT::InputPort<double>("orbit_speed_mps"), BT::InputPort<double>("radius_m"),
  BT::InputPort<double>("range_gain"), BT::InputPort<double>("max_range_speed_mps"),
  BT::InputPort<double>("yaw_gain"), BT::InputPort<double>("max_yaw_rate_rad_s"),
  BT::InputPort<double>("vertical_gain"), BT::InputPort<double>("max_vertical_speed_mps"),
  BT::InputPort<double>("angle_rad"), BT::InputPort<double>("timeout_s"))
banyu_msgs::action::OrbitMarker::Goal OrbitMarkerNode::makeGoal()
{
  Goal goal;
  goal.orbit_speed_mps = input<double>("orbit_speed_mps");
  goal.radius_m = input<double>("radius_m");
  goal.range_gain = input<double>("range_gain");
  goal.max_range_speed_mps = input<double>("max_range_speed_mps");
  goal.yaw_gain = input<double>("yaw_gain");
  goal.max_yaw_rate_rad_s = input<double>("max_yaw_rate_rad_s");
  goal.vertical_gain = input<double>("vertical_gain");
  goal.max_vertical_speed_mps = input<double>("max_vertical_speed_mps");
  goal.angle_rad = input<double>("angle_rad");
  goal.timeout_s = input<double>("timeout_s");
  return goal;
}

BANYU_POR_CONSTRUCTOR(TurnAroundNode, banyu_msgs::action::TurnAround)
BANYU_POR_PORTS(
  TurnAroundNode,
  BT::InputPort<double>("turn_yaw_rate_rad_s"), BT::InputPort<double>("capture_gain"),
  BT::InputPort<double>("capture_deadband"), BT::InputPort<double>("max_angle_rad"),
  BT::InputPort<double>("timeout_s"))
banyu_msgs::action::TurnAround::Goal TurnAroundNode::makeGoal()
{
  Goal goal;
  goal.turn_yaw_rate_rad_s = input<double>("turn_yaw_rate_rad_s");
  goal.capture_gain = input<double>("capture_gain");
  goal.capture_deadband = input<double>("capture_deadband");
  goal.max_angle_rad = input<double>("max_angle_rad");
  goal.timeout_s = input<double>("timeout_s");
  return goal;
}

#undef BANYU_POR_PORTS
#undef BANYU_POR_CONSTRUCTOR

}  // namespace banyu_mission
