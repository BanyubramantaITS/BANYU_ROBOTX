#ifndef BANYU_MISSION_BT_NODES_TASK2_SOURCE_HPP_
#define BANYU_MISSION_BT_NODES_TASK2_SOURCE_HPP_

#include <memory>

#include <behaviortree_cpp/condition_node.h>

#include "banyu_mission/mission_context.hpp"

namespace banyu_mission {

class Task2CameraSourceSelected : public BT::ConditionNode {
 public:
  Task2CameraSourceSelected(const std::string& name, const BT::NodeConfig& config,
                            std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  std::shared_ptr<MissionContext> ctx_;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION_BT_NODES_TASK2_SOURCE_HPP_
