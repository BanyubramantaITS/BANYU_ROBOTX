#ifndef BANYU_DETECTION__RESOURCE_CODE_DECODER_NODE_HPP_
#define BANYU_DETECTION__RESOURCE_CODE_DECODER_NODE_HPP_

#include <mutex>
#include <string>

#include "banyu_detection/resource_code_decoder.hpp"
#include "banyu_msgs/msg/pipeline_ring_observation.hpp"
#include "banyu_msgs/msg/resource_code.hpp"
#include "banyu_msgs/msg/run_lifecycle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace banyu_detection
{

// Collects ring colours while a decode window is open (SetBool true) and publishes the code read so far.
class ResourceCodeDecoderNode : public rclcpp::Node
{
public:
  explicit ResourceCodeDecoderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  bool windowOpen() const;

private:
  void handleRing(const banyu_msgs::msg::PipelineRingObservation::SharedPtr msg);
  void handleLifecycle(const banyu_msgs::msg::RunLifecycle::SharedPtr msg);
  void handleWindow(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);
  void publishResult(bool final_result);

  mutable std::mutex mutex_;
  ResourceCodeConfig config_;
  ResourceCodeDecoder decoder_;
  uint32_t run_id_{0};
  uint64_t epoch_{0};
  bool open_{false};
  double max_window_s_{40.0};
  double window_start_s_{0.0};

  rclcpp::Subscription<banyu_msgs::msg::PipelineRingObservation>::SharedPtr ring_sub_;
  rclcpp::Subscription<banyu_msgs::msg::RunLifecycle>::SharedPtr lifecycle_sub_;
  rclcpp::Publisher<banyu_msgs::msg::ResourceCode>::SharedPtr result_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr window_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace banyu_detection

#endif  // BANYU_DETECTION__RESOURCE_CODE_DECODER_NODE_HPP_
