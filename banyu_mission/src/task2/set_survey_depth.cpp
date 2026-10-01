#include "banyu_mission/task2/set_survey_depth.hpp"

#include "banyu_mission/task2/vehicle_io.hpp"

namespace banyu_mission::task2 {

BT::NodeStatus SetSurveyDepth::tick() {
  const Snapshot s = snapshot(*ctx_);
  const double search = node_->get_parameter("task2_search_altitude").as_double();
  double target;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    // Not s.altitude: with the pipe under the vehicle that is the range to the pipe top, and the
    // target would come out pipe_height too shallow, above where the camera can see the pipe.
    target = ctx_->seabed_depth.value_or(s.depth + s.altitude.value_or(search)) -
             node_->get_parameter("task2_pipe_height").as_double() -
             node_->get_parameter("task2_survey_altitude").as_double();
    ctx_->depth_target = target;
  }
  RCLCPP_INFO(node_->get_logger(), "pipe in view at depth %.2f m, survey depth target %.2f m", s.depth, target);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace banyu_mission::task2
