#include "banyu_mission/action/handle_abort_or_teleop_node.hpp"

#include <utility>

namespace banyu_mission
{

HandleAbortOrTeleopNode::HandleAbortOrTeleopNode(
  const std::string & name,
  const BT::NodeConfig & config,
  std::shared_ptr<MissionState> state,
  std::function<void()> on_abort)
: BT::SyncActionNode(name, config), state_(std::move(state)), on_abort_(std::move(on_abort))
{
}

BT::NodeStatus HandleAbortOrTeleopNode::tick()
{
  state_->requestAbort();
  if (on_abort_) {
    on_abort_();
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace banyu_mission
