#ifndef BANYU_DETECTION_RING_TRACKER_NODE_HPP_
#define BANYU_DETECTION_RING_TRACKER_NODE_HPP_

#include <deque>
#include <mutex>
#include <string>

#include <banyu_msgs/msg/pipeline_ring_observation.hpp>
#include <banyu_msgs/msg/pipeline_ring_state.hpp>
#include <banyu_msgs/msg/task2_ring_record.hpp>
#include <banyu_msgs/srv/acknowledge_ring_repair.hpp>
#include <banyu_msgs/srv/reset_task2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "banyu_detection/ring_tracker_adapter.hpp"
#include "banyu_detection/run_context_gate.hpp"

namespace banyu_detection
{

inline rclcpp::QoS ringStateQoS()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

inline rclcpp::QoS acceptedRingQoS()
{
  return rclcpp::QoS(rclcpp::KeepLast(32)).reliable().transient_local();
}

class RingTrackerNode : public rclcpp::Node
{
public:
  explicit RingTrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using AcknowledgeRingRepair = banyu_msgs::srv::AcknowledgeRingRepair;
  using ResetTask2 = banyu_msgs::srv::ResetTask2;

  void declareParameters();
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void observationCallback(
    const banyu_msgs::msg::PipelineRingObservation::SharedPtr msg);
  void acknowledgeCallback(
    const std::shared_ptr<AcknowledgeRingRepair::Request> request,
    std::shared_ptr<AcknowledgeRingRepair::Response> response);
  void resetCallback(
    const std::shared_ptr<ResetTask2::Request> request,
    std::shared_ptr<ResetTask2::Response> response);
  bool nearestPose(
    const builtin_interfaces::msg::Time & stamp,
    geometry_msgs::msg::PoseStamped & pose) const;

  static double stampSeconds(const builtin_interfaces::msg::Time & stamp);

  std::string observation_topic_;
  std::string pose_topic_;
  std::string state_topic_;
  std::string accepted_ring_topic_;
  std::string repair_service_;
  std::string reset_service_;
  bool require_run_context_{false};
  double pose_timestamp_tolerance_s_{0.05};
  std::size_t pose_history_size_{100};

  RingTrackerAdapter adapter_;
  RunContextGate run_context_;
  mutable std::mutex adapter_mutex_;
  mutable std::mutex pose_mutex_;
  std::deque<geometry_msgs::msg::PoseStamped> pose_history_;

  rclcpp::Subscription<banyu_msgs::msg::PipelineRingObservation>::SharedPtr observation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<banyu_msgs::msg::PipelineRingState>::SharedPtr state_pub_;
  rclcpp::Publisher<banyu_msgs::msg::Task2RingRecord>::SharedPtr accepted_ring_pub_;
  rclcpp::Service<AcknowledgeRingRepair>::SharedPtr repair_service_server_;
  rclcpp::Service<ResetTask2>::SharedPtr reset_service_server_;
};

}  // namespace banyu_detection

#endif  // BANYU_DETECTION_RING_TRACKER_NODE_HPP_
