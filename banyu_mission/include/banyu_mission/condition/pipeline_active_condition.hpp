#ifndef BANYU_MISSION__CONDITION__PIPELINE_ACTIVE_CONDITION_HPP_
#define BANYU_MISSION__CONDITION__PIPELINE_ACTIVE_CONDITION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "banyu_mission/mission_state.hpp"

namespace banyu_mission
{

class PipelineActiveCondition : public BT::ConditionNode
{
public:
  PipelineActiveCondition(
    const std::string & name,
    const BT::NodeConfig & config,
    std::shared_ptr<MissionState> state);

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<MissionState> state_;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__CONDITION__PIPELINE_ACTIVE_CONDITION_HPP_
