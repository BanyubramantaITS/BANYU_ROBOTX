#include "banyu_mission/condition/survey_leg_conditions.hpp"

#include <utility>

namespace banyu_mission
{

RingsCompleteCondition::RingsCompleteCondition(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<MissionState> state)
: BT::ConditionNode(name, config), state_(std::move(state))
{
}

BT::NodeStatus RingsCompleteCondition::tick()
{
  return state_->ringsComplete() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

PipelineLegEndedCondition::PipelineLegEndedCondition(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<MissionState> state)
: BT::ConditionNode(name, config), state_(std::move(state))
{
}

BT::NodeStatus PipelineLegEndedCondition::tick()
{
  const double lost_s = getInput<double>("lost_s").value_or(6.0);
  return state_->legEnded(lost_s) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace banyu_mission
