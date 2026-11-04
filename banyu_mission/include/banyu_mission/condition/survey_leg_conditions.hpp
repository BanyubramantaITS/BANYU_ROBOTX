#ifndef BANYU_MISSION__CONDITION__SURVEY_LEG_CONDITIONS_HPP_
#define BANYU_MISSION__CONDITION__SURVEY_LEG_CONDITIONS_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "banyu_mission/mission_state.hpp"

namespace banyu_mission
{

// Four rings recorded and none waiting for its repair: the survey has everything it needs.
class RingsCompleteCondition : public BT::ConditionNode
{
public:
  RingsCompleteCondition(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<MissionState> state);
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<MissionState> state_;
};

// A survey leg reached a pipe end: the pipe was out of view `lost_s` in total since the last new ring.
class PipelineLegEndedCondition : public BT::ConditionNode
{
public:
  PipelineLegEndedCondition(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<MissionState> state);
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("lost_s", 6.0, "total time out of view since the last new ring")};
  }
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<MissionState> state_;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__CONDITION__SURVEY_LEG_CONDITIONS_HPP_
