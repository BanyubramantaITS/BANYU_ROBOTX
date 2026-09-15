#ifndef BANYU_MISSION_BT_NODES_FOLLOW_AND_RECORD_TASK2_PIPELINE_HPP_
#define BANYU_MISSION_BT_NODES_FOLLOW_AND_RECORD_TASK2_PIPELINE_HPP_

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include "banyu_mission/bt_nodes/task2_camera_common.hpp"
#include "banyu_mission/mission_context.hpp"

namespace banyu_mission {

class FollowAndRecordTask2Pipeline : public BT::StatefulActionNode {
 public:
  FollowAndRecordTask2Pipeline(const std::string& name, const BT::NodeConfig& config,
                               const rclcpp::Node::SharedPtr& node,
                               std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  bool recordPathLocked(const rclcpp::Time& source_stamp,
                        const geometry_msgs::msg::PoseStamped& associated_pose);
  bool storeRequiredPathSampleLocked(const geometry_msgs::msg::PoseStamped& pose);
  bool storeFinalObservedPathSampleLocked();
  bool processRingLocked(const bt_nodes::task2_camera::PipelineFrame& frame,
                         const geometry_msgs::msg::PoseStamped& associated_pose,
                         const rclcpp::Time& now);
  bool endConditionsMetLocked(const rclcpp::Time& source_stamp) const;
  void failLocked(const std::string& reason);
  void publishFollowCommand(const banyu_msgs::msg::PipelineObservation& observation,
                            const rclcpp::Time& stamp);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  rclcpp::Time start_time_;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION_BT_NODES_FOLLOW_AND_RECORD_TASK2_PIPELINE_HPP_
