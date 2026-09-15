#include "banyu_mission/bt_nodes/task2_source.hpp"

namespace banyu_mission {

Task2CameraSourceSelected::Task2CameraSourceSelected(
    const std::string& name, const BT::NodeConfig& config, std::shared_ptr<MissionContext> ctx)
    : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

BT::NodeStatus Task2CameraSourceSelected::tick() {
  std::lock_guard<std::mutex> lock(ctx_->data_mutex);
  return ctx_->task2_camera_source_selected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace banyu_mission
