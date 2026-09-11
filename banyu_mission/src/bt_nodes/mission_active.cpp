#include "banyu_mission/bt_nodes/mission_active.hpp"

#include <utility>

namespace banyu_mission {

MissionActive::MissionActive(const std::string& name, const BT::NodeConfig& config,
                             std::shared_ptr<MissionContext> ctx)
    : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

BT::NodeStatus MissionActive::tick() {
  return ctx_->mission_active.load() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace banyu_mission
