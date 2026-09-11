#ifndef BANYU_MISSION_BT_NODES_MISSION_ACTIVE_HPP_
#define BANYU_MISSION_BT_NODES_MISSION_ACTIVE_HPP_

#include <memory>
#include <string>

#include <behaviortree_cpp/condition_node.h>

#include "banyu_mission/mission_context.hpp"

namespace banyu_mission {

class MissionActive : public BT::ConditionNode {
 public:
  MissionActive(const std::string& name, const BT::NodeConfig& config,
                std::shared_ptr<MissionContext> ctx);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }

 private:
  std::shared_ptr<MissionContext> ctx_;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION_BT_NODES_MISSION_ACTIVE_HPP_
