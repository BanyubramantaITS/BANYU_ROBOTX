#ifndef BANYU_MISSION_BT_NODES_SURFACE_ACQUIRE_GPS_ANCHOR_HPP_
#define BANYU_MISSION_BT_NODES_SURFACE_ACQUIRE_GPS_ANCHOR_HPP_

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include "banyu_mission/mission_context.hpp"

namespace banyu_mission {

class SurfaceAcquireGpsAnchor : public BT::StatefulActionNode {
 public:
  SurfaceAcquireGpsAnchor(const std::string& name, const BT::NodeConfig& config,
                          const rclcpp::Node::SharedPtr& node,
                          std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  rclcpp::Time start_time_;
  bool readiness_achieved_{false};
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION_BT_NODES_SURFACE_ACQUIRE_GPS_ANCHOR_HPP_
