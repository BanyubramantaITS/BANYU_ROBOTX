#include "banyu_mission/task2/set_survey_depth.hpp"

#include "banyu_mission/task2/vehicle_io.hpp"

namespace banyu_mission::task2 {

BT::NodeStatus SetSurveyDepth::tick() {
  const Snapshot s = snapshot(*ctx_);
  const double search = node_->get_parameter("task2_search_altitude").as_double();
  const double target = s.depth + s.altitude.value_or(search) - node_->get_parameter("task2_pipe_height").as_double() -
                        node_->get_parameter("task2_survey_altitude").as_double();
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->depth_target = target;
  }
  RCLCPP_INFO(node_->get_logger(), "pipe in view at depth %.2f m, survey depth target %.2f m", s.depth, target);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace banyu_mission::task2
