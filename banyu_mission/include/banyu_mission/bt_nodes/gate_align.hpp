#ifndef BANYU_MISSION_BT_NODES_GATE_ALIGN_HPP_
#define BANYU_MISSION_BT_NODES_GATE_ALIGN_HPP_

#include <memory>

#include "banyu_mission/bt_nodes/visual_align.hpp"

namespace banyu_mission {

class GateAlign : public VisualAlign {
 public:
  GateAlign(const std::string& name, const BT::NodeConfig& config,
            const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx);
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION_BT_NODES_GATE_ALIGN_HPP_
