#ifndef BANYU_DETECTION_PIPELINE_END_NODE_HPP_
#define BANYU_DETECTION_PIPELINE_END_NODE_HPP_

#include <mutex>
#include <string>

#include <banyu_msgs/msg/pipeline_observation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "banyu_detection/pipeline_end_adapter.hpp"

namespace banyu_detection
{

class PipelineEndNode : public rclcpp::Node
{
public:
  explicit PipelineEndNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  const PipelineEndAdapter & adapter() const noexcept {return adapter_;}
  void reset();

private:
  void declareParameters();
  void observationCallback(const banyu_msgs::msg::PipelineObservation::SharedPtr msg);
  void candidateCallback(const std_msgs::msg::Bool::SharedPtr msg);

  std::string observation_topic_;
  std::string candidate_topic_;
  std::string end_topic_;
  bool publish_confirmed_only_{true};
  bool default_candidate_{false};
  double candidate_timeout_s_{1.0};

  PipelineEndAdapter adapter_;
  mutable std::mutex mutex_;
  bool candidate_active_{false};
  bool candidate_received_{false};
  rclcpp::Time candidate_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<banyu_msgs::msg::PipelineObservation>::SharedPtr observation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr candidate_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr end_pub_;
};

}  // namespace banyu_detection

#endif  // BANYU_DETECTION_PIPELINE_END_NODE_HPP_
