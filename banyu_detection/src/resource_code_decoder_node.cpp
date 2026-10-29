#include "banyu_detection/resource_code_decoder_node.hpp"

#include <algorithm>
#include <chrono>
#include <memory>

namespace banyu_detection
{

ResourceCodeDecoderNode::ResourceCodeDecoderNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("resource_code_decoder", options)
{
  const auto ring_topic = declare_parameter<std::string>(
    "topics.ring_observation", "/banyu/pipeline/ring_observation");
  const auto lifecycle_topic = declare_parameter<std::string>(
    "topics.run_lifecycle", "/banyu/mission/run_lifecycle");
  const auto result_topic = declare_parameter<std::string>(
    "topics.resource_code", "/banyu/pipeline/resource_code");
  const auto window_service = declare_parameter<std::string>(
    "services.decode_window", "/banyu/pipeline/decode_resource_code");

  // code_length follows the tier: Advanced reads the circle only, Disruptive tin and circle.
  const int tier = static_cast<int>(declare_parameter<int64_t>("tier", 4));
  config_.code_length = tier >= 4 ? 2 : 1;
  config_.min_on_s = declare_parameter<double>("min_on_s", config_.min_on_s);
  config_.max_on_s = declare_parameter<double>("max_on_s", config_.max_on_s);
  config_.steady_on_s = declare_parameter<double>("steady_on_s", config_.steady_on_s);
  config_.long_gap_s = declare_parameter<double>("long_gap_s", config_.long_gap_s);
  config_.merge_gap_s = declare_parameter<double>("merge_gap_s", config_.merge_gap_s);
  config_.min_cycles = static_cast<int>(declare_parameter<int64_t>("min_cycles", config_.min_cycles));
  min_confidence_ = declare_parameter<double>("min_confidence", 0.85);
  config_.require_repair_verified =
    declare_parameter<bool>("require_repair_verified", config_.require_repair_verified);
  max_window_s_ = declare_parameter<double>("max_window_s", max_window_s_);
  const double publish_period_s = declare_parameter<double>("publish_period_s", 1.0);
  decoder_ = ResourceCodeDecoder(config_);

  result_pub_ = create_publisher<banyu_msgs::msg::ResourceCode>(
    result_topic, rclcpp::QoS(1).reliable().transient_local());
  ring_sub_ = create_subscription<banyu_msgs::msg::PipelineRingObservation>(
    ring_topic, rclcpp::SensorDataQoS(),
    std::bind(&ResourceCodeDecoderNode::handleRing, this, std::placeholders::_1));
  lifecycle_sub_ = create_subscription<banyu_msgs::msg::RunLifecycle>(
    lifecycle_topic, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&ResourceCodeDecoderNode::handleLifecycle, this, std::placeholders::_1));
  window_srv_ = create_service<std_srvs::srv::SetBool>(
    window_service,
    std::bind(
      &ResourceCodeDecoderNode::handleWindow, this, std::placeholders::_1, std::placeholders::_2));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(std::max(0.1, publish_period_s))),
    [this]() {
      bool open = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        open = open_;
        if (open_ && now().seconds() - window_start_s_ > max_window_s_) {open_ = false;}
      }
      if (open) {publishResult(false);}
    });
  RCLCPP_INFO(
    get_logger(), "ResourceCodeDecoder: code_length=%d, window service '%s'",
    config_.code_length, window_service.c_str());
}

bool ResourceCodeDecoderNode::windowOpen() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return open_;
}

void ResourceCodeDecoderNode::handleLifecycle(const banyu_msgs::msg::RunLifecycle::SharedPtr msg)
{
  if (!msg) {return;}
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->run_id != run_id_ || msg->epoch != epoch_) {
    run_id_ = msg->run_id;
    epoch_ = msg->epoch;
    open_ = false;
    decoder_.reset();
  }
}

void ResourceCodeDecoderNode::handleRing(
  const banyu_msgs::msg::PipelineRingObservation::SharedPtr msg)
{
  if (!msg) {return;}
  std::lock_guard<std::mutex> lock(mutex_);
  if (!open_) {return;}
  const double t = rclcpp::Time(msg->header.stamp).seconds();
  // Only a lit ring counts: a weak colour match in the dark half of a flash is not a flash.
  decoder_.add(t, (msg->visible && msg->confidence >= min_confidence_) ? static_cast<int>(msg->color) : 0);
}

void ResourceCodeDecoderNode::handleWindow(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  bool publish_final = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request->data) {
      decoder_.reset();
      open_ = true;
      window_start_s_ = now().seconds();
      response->message = "decode window open";
    } else {
      publish_final = open_;
      open_ = false;
      response->message = "decode window closed";
    }
    response->success = true;
  }
  if (publish_final) {publishResult(true);}
}

void ResourceCodeDecoderNode::publishResult(bool final_result)
{
  banyu_msgs::msg::ResourceCode out;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto r = decoder_.decode();
    out.run_id = run_id_;
    out.epoch = epoch_;
    out.complete = r.complete;
    out.repair_verified = r.repair_verified;
    out.resource_color = static_cast<uint8_t>(r.complete ? r.tin : 0);
    out.delivery_circle_color = static_cast<uint8_t>(r.complete ? r.circle : 0);
    out.cycles = static_cast<uint8_t>(std::min(r.cycles, 255));
    out.cycles_seen = static_cast<uint8_t>(std::min(r.cycles_seen, 255));
    out.confidence = r.confidence;
  }
  out.header.stamp = now();
  out.header.frame_id = final_result ? "final" : "running";
  result_pub_->publish(out);
}

}  // namespace banyu_detection

#ifndef BANYU_DETECTION_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<banyu_detection::ResourceCodeDecoderNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
