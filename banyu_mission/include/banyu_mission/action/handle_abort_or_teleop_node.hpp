#ifndef BANYU_MISSION__ACTION__HANDLE_ABORT_OR_TELEOP_NODE_HPP_
#define BANYU_MISSION__ACTION__HANDLE_ABORT_OR_TELEOP_NODE_HPP_

#include <functional>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "banyu_mission/mission_state.hpp"

namespace banyu_mission
{

class HandleAbortOrTeleopNode : public BT::SyncActionNode
{
public:
  HandleAbortOrTeleopNode(
    const std::string & name,
    const BT::NodeConfig & config,
    std::shared_ptr<MissionState> state,
    std::function<void()> on_abort = nullptr);

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<MissionState> state_;
  std::function<void()> on_abort_;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__ACTION__HANDLE_ABORT_OR_TELEOP_NODE_HPP_
