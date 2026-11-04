#ifndef BANYU_DETECTION_PIPELINE_END_NODE_HPP_
#define BANYU_DETECTION_PIPELINE_END_NODE_HPP_

#include <mutex>
#include <string>

#include <banyu_msgs/msg/pipeline_observation.hpp>
#include <banyu_msgs/msg/pipeline_end_candidate.hpp>
#include <banyu_msgs/msg/pipeline_end_state.hpp>
#include <banyu_msgs/srv/reset_task2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "banyu_detection/pipeline_end_adapter.hpp"
#include "banyu_detection/run_context_gate.hpp"

namespace banyu_detection
{

class PipelineEndNode : public rclcpp::Node
{
public:
  explicit PipelineEndNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  const PipelineEndAdapter & adapter() const noexcept {return adapter_;}
  void reset();

private:
  using ResetTask2 = banyu_msgs::srv::ResetTask2;

  void declareParameters();
  void observationCallback(const banyu_msgs::msg::PipelineObservation::SharedPtr msg);
  void candidateCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void candidateStateCallback(const banyu_msgs::msg::PipelineEndCandidate::SharedPtr msg);
  void ringsCompleteCallback(const banyu_msgs::msg::PipelineEndCandidate::SharedPtr msg);
  void resetCallback(
    const std::shared_ptr<ResetTask2::Request> request,
    std::shared_ptr<ResetTask2::Response> response);
  void publishEnd(bool ended);

  std::string observation_topic_;
  std::string candidate_topic_;
  std::string end_topic_;
  std::string end_state_topic_;
  std::string candidate_state_topic_;
  std::string reset_service_;
  std::string rings_complete_topic_;
  // > 0: four rings recorded (none waiting for a repair) and the pipe out of view this long in total since then ends the
  // traverse. The only end signal on the real vehicle (the end candidates come from the sim fixture). In total, not in one
  // go: at the end the search finds the pipe's last stretch again within a second, every time.
  double rings_complete_lost_s_{3.0};
  // The confirmed end candidate counts only once the four rings are complete (a two-leg survey from the midpoint
  // passes one end of the pipe before it has seen every ring).
  bool require_rings_complete_{false};
  bool require_run_context_{false};
  bool publish_confirmed_only_{true};
  bool default_candidate_{false};
  double candidate_timeout_s_{1.0};

  PipelineEndAdapter adapter_;
  RunContextGate run_context_;
  mutable std::mutex mutex_;
  bool candidate_active_{false};
  bool candidate_received_{false};
  rclcpp::Time candidate_stamp_{0, 0, RCL_ROS_TIME};
  bool rings_complete_{false};
  bool has_last_stamp_{false};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  double lost_after_rings_s_{0.0};
  bool ended_by_rings_{false};

  rclcpp::Subscription<banyu_msgs::msg::PipelineObservation>::SharedPtr observation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr candidate_sub_;
  rclcpp::Subscription<banyu_msgs::msg::PipelineEndCandidate>::SharedPtr candidate_state_sub_;
  rclcpp::Subscription<banyu_msgs::msg::PipelineEndCandidate>::SharedPtr rings_complete_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr end_pub_;
  rclcpp::Publisher<banyu_msgs::msg::PipelineEndState>::SharedPtr end_state_pub_;
  rclcpp::Service<ResetTask2>::SharedPtr reset_service_server_;
};

}  // namespace banyu_detection

#endif  // BANYU_DETECTION_PIPELINE_END_NODE_HPP_
