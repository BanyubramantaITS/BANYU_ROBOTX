#include "banyu_mission/bt_nodes/aborted_or_teleop.hpp"

#include <utility>

namespace banyu_mission {

AbortedOrTeleop::AbortedOrTeleop(const std::string& name, const BT::NodeConfig& config,
                                 std::shared_ptr<MissionContext> ctx)
    : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

BT::NodeStatus AbortedOrTeleop::tick() {
  if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace banyu_mission
