#include "banyu_detection/pipeline_end_node.hpp"

#include <algorithm>
#include <memory>

namespace banyu_detection
{

PipelineEndNode::PipelineEndNode(const rclcpp::NodeOptions & options)
: Node("pipeline_end_node", options)
{
  declareParameters();

  end_pub_ = create_publisher<std_msgs::msg::Bool>(end_topic_, rclcpp::QoS(5));
  observation_sub_ = create_subscription<banyu_msgs::msg::PipelineObservation>(
    observation_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PipelineEndNode::observationCallback, this, std::placeholders::_1));
  candidate_sub_ = create_subscription<std_msgs::msg::Bool>(
    candidate_topic_, rclcpp::QoS(10),
    std::bind(&PipelineEndNode::candidateCallback, this, std::placeholders::_1));
}

void PipelineEndNode::declareParameters()
{
  observation_topic_ = declare_parameter<std::string>(
    "observation_topic", "/banyu/pipeline/observation");
  candidate_topic_ = declare_parameter<std::string>(
    "candidate_topic", "/banyu/pipeline/end_candidate");
  end_topic_ = declare_parameter<std::string>(
    "end_topic", "/banyu/pipeline/end");
  publish_confirmed_only_ = declare_parameter<bool>(
    "publish_confirmed_only", true);
  default_candidate_ = declare_parameter<bool>(
    "default_candidate", false);
  candidate_timeout_s_ = declare_parameter<double>(
    "candidate_timeout_s", 1.0);

  PipelineEndConfirmerConfig config;
  config.required_frames = static_cast<std::size_t>(std::max<int64_t>(
      2, declare_parameter<int64_t>("required_frames", 3)));
  config.maximum_sequence_gap = static_cast<uint64_t>(std::max<int64_t>(
      1, declare_parameter<int64_t>("maximum_sequence_gap", 1)));
  config.minimum_confidence = declare_parameter<double>(
    "minimum_confidence", 0.5);
  config.maximum_offset_delta = declare_parameter<double>(
    "maximum_offset_delta", 0.15);
  config.maximum_heading_delta_rad = declare_parameter<double>(
    "maximum_heading_delta_rad", 0.20);

  adapter_ = PipelineEndAdapter(config);
}

void PipelineEndNode::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  adapter_.reset();
  candidate_active_ = false;
  candidate_received_ = false;
}

void PipelineEndNode::candidateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  candidate_active_ = msg->data;
  candidate_received_ = true;
  candidate_stamp_ = now();
}

void PipelineEndNode::observationCallback(
  const banyu_msgs::msg::PipelineObservation::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  bool is_candidate = default_candidate_;
  if (candidate_received_) {
    const double age = (now() - candidate_stamp_).seconds();
    if (age >= 0.0 && age <= candidate_timeout_s_) {
      is_candidate = candidate_active_;
    } else {
      candidate_received_ = false;
      candidate_active_ = false;
    }
  }

  const auto update = adapter_.update(*msg, is_candidate);
  if (update.confirmed) {
    std_msgs::msg::Bool end_msg;
    end_msg.data = true;
    end_pub_->publish(end_msg);
  } else if (!publish_confirmed_only_) {
    std_msgs::msg::Bool end_msg;
    end_msg.data = false;
    end_pub_->publish(end_msg);
  }
}

}  // namespace banyu_detection
