// Base for the Task 2 stateful BT nodes: node + context handles, a phase clock, and the
// abort check every phase needs.
#ifndef BANYU_MISSION_TASK2_TASK2_ACTION_HPP_
#define BANYU_MISSION_TASK2_TASK2_ACTION_HPP_

#include <algorithm>
#include <memory>
#include <optional>
#include <string>

#include <behaviortree_cpp/action_node.h>

#include "banyu_mission/mission_context.hpp"
#include "banyu_mission/task2/vehicle_io.hpp"

namespace banyu_mission::task2 {

class Task2Action : public BT::StatefulActionNode {
 public:
  Task2Action(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node,
              std::shared_ptr<MissionContext> ctx)
      : BT::StatefulActionNode(name, config), node_(std::move(node)), ctx_(std::move(ctx)) {}

  BT::NodeStatus onStart() final {
    start_ = node_->now();
    last_seq_ = snapshot(*ctx_).pipe_seq;
    return begin();
  }
  BT::NodeStatus onRunning() final {
    if (!ctx_->mission_active.load() || ctx_->aborted.load()) {
      sendSticks(*ctx_, 0, 0, 0);
      return BT::NodeStatus::FAILURE;
    }
    return step(snapshot(*ctx_));
  }
  void onHalted() override { sendSticks(*ctx_, 0, 0, 0); }

 protected:
  virtual BT::NodeStatus begin() { return BT::NodeStatus::RUNNING; }
  virtual BT::NodeStatus step(const Snapshot& s) = 0;

  double elapsed() const { return (node_->now() - start_).seconds(); }
  // True once per new bottom-camera observation, for logic that counts frames.
  bool newFrame(const Snapshot& s) {
    if (s.pipe_seq == last_seq_) return false;
    last_seq_ = s.pipe_seq;
    return true;
  }
  const rclcpp::Logger logger() const { return node_->get_logger(); }
  double param(const std::string& name) const { return node_->get_parameter(name).as_double(); }

  // The light node nearest under the repair probe tip (see lightUnderProbe). The camera's height
  // above the switches comes from pressure depth and the pipe depth FollowPipe has been holding
  // survey altitude over: the echosounder often ranges past the pipe to the seabed.
  std::optional<ProbeFix> lightUnderProbe(const Snapshot& s, double gate_m) const {
    const double drop = param("task2_probe_drop");
    // depth_target + survey_altitude = pipe top under the echosounder (0.11 m below the pressure
    // sensor's datum); the camera is 0.05 m below it, a switch 0.04 m above the pipe.
    const double z = s.depth_target + param("task2_survey_altitude") + 0.11 - 0.04 - (s.depth + 0.05);
    return task2::lightUnderProbe(s.pipe, std::max(drop, z), param("task2_probe_forward"), gate_m);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;

 private:
  rclcpp::Time start_;
  uint64_t last_seq_{0};
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_TASK2_ACTION_HPP_
