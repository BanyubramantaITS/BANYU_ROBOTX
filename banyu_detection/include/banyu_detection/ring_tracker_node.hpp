#ifndef BANYU_DETECTION_RING_TRACKER_NODE_HPP_
#define BANYU_DETECTION_RING_TRACKER_NODE_HPP_

#include <deque>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <banyu_msgs/msg/pipeline_end_candidate.hpp>
#include <banyu_msgs/msg/pipeline_observation.hpp>
#include <banyu_msgs/msg/pipeline_ring_observation.hpp>
#include <banyu_msgs/msg/pipeline_ring_state.hpp>
#include <banyu_msgs/msg/task2_ring_record.hpp>
#include <banyu_msgs/srv/acknowledge_ring_repair.hpp>
#include <banyu_msgs/srv/reset_task2.hpp>
#include <banyu_msgs/msg/task2_buoy_target.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <rclcpp/rclcpp.hpp>

#include "banyu_detection/dead_reckoner.hpp"
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
  // Four rings recorded and no RED waiting for its repair: nothing is left to survey (pipeline_end_node ends the
  // traverse once the pipe is also out of view).
  void publishRingsComplete();
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
  std::string rings_complete_topic_;
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

  // Dead-reckoned pose under water and ring progress along the buoy-to-buoy line (no pipe map, no underwater position).
  double buoy_line_half_length_m_{30.0};
  double gps_fresh_s_{1.0};
  DeadReckoner reckoner_;
  bool fix_valid_{false};
  rclcpp::Time fix_time_{0, 0, RCL_ROS_TIME};
  bool buoy_heading_valid_{false};
  double buoy_heading_enu_{0.0};
  bool axis_set_{false};
  std::vector<std::pair<double, double>> pending_axis_;

  // Camera height above the pipe from its apparent width (median of the recent frames).
  bool pipe_height_enabled_{true};
  double pipe_diameter_m_{0.0889};
  double pipe_hfov_rad_{1.047};
  double camera_tilt_rad_{0.0};
  std::deque<std::pair<double, double>> pipe_heights_;   // (stamp s, height m)
  double pipeHeight(double t) const;
  rclcpp::Subscription<banyu_msgs::msg::PipelineObservation>::SharedPtr pipe_sub_;
  // The mission switches ring recording off while diving and backing up to the pipe's start (rings seen then are placed
  // badly or out of order); on for the survey pass. On until the mission says otherwise.
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recording_sub_;
  bool recording_{true};
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr command_sub_;
  rclcpp::Subscription<banyu_msgs::msg::Task2BuoyTarget>::SharedPtr buoy_sub_;

  rclcpp::Subscription<banyu_msgs::msg::PipelineRingObservation>::SharedPtr observation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<banyu_msgs::msg::PipelineRingState>::SharedPtr state_pub_;
  rclcpp::Publisher<banyu_msgs::msg::Task2RingRecord>::SharedPtr accepted_ring_pub_;
  rclcpp::Publisher<banyu_msgs::msg::PipelineEndCandidate>::SharedPtr rings_complete_pub_;
  rclcpp::Service<AcknowledgeRingRepair>::SharedPtr repair_service_server_;
  rclcpp::Service<ResetTask2>::SharedPtr reset_service_server_;
};

}  // namespace banyu_detection

#endif  // BANYU_DETECTION_RING_TRACKER_NODE_HPP_
