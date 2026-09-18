#include "banyu_mission/action/handle_abort_or_teleop_node.hpp"

#include <utility>

namespace banyu_mission
{

HandleAbortOrTeleopNode::HandleAbortOrTeleopNode(
  const std::string & name,
  const BT::NodeConfig & config,
  std::shared_ptr<MissionState> state)
: BT::SyncActionNode(name, config), state_(std::move(state))
{
}

BT::NodeStatus HandleAbortOrTeleopNode::tick()
{
  state_->requestAbort();
  return BT::NodeStatus::SUCCESS;
}

}  // namespace banyu_mission
