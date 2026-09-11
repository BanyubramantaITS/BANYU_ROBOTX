#include "banyu_mission/bt_nodes/marker_align.hpp"

#include <utility>

namespace banyu_mission {

MarkerAlign::MarkerAlign(const std::string& name, const BT::NodeConfig& config,
                         const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
    : VisualAlign(name, config, node, std::move(ctx), 1, "marker_align_") {}

}  // namespace banyu_mission
