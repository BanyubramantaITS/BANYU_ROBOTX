#include "banyu_mission/condition/red_ring_pending_condition.hpp"

#include <utility>

namespace banyu_mission
{

RedRingPendingCondition::RedRingPendingCondition(
  const std::string & name,
  const BT::NodeConfig & config,
  std::shared_ptr<MissionState> state)
: BT::ConditionNode(name, config), state_(std::move(state))
{
}

BT::NodeStatus RedRingPendingCondition::tick()
{
  return state_->redPending() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace banyu_mission
