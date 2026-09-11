#include "banyu_mission/bt_nodes/gate_align.hpp"

#include <utility>

namespace banyu_mission {

GateAlign::GateAlign(const std::string& name, const BT::NodeConfig& config,
                     const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
    : VisualAlign(name, config, node, std::move(ctx), 0, "gate_align_") {}

}  // namespace banyu_mission
