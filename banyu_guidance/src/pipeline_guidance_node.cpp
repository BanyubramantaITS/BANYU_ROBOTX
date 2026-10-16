#include "banyu_guidance/pipeline_follower.hpp"

#include <banyu_guidance/msg/velocity_intent.hpp>
#include <banyu_msgs/msg/pipeline_observation.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace banyu_guidance
{

class PipelineGuidanceNode final : public rclcpp::Node
{
public:
  PipelineGuidanceNode()
  : Node("pipeline_guidance")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/banyu/pipeline/observation");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/banyu/guidance/velocity_intent");
    intent_frame_ = declare_parameter<std::string>("intent_frame", "base_link");

    GuidanceConfig config;
    config.surge_gain_mps = declare_parameter<double>("surge_gain_mps", config.surge_gain_mps);
    config.max_surge_mps = declare_parameter<double>("max_surge_mps", config.max_surge_mps);
    config.sway_gain_mps = declare_parameter<double>("sway_gain_mps", config.sway_gain_mps);
    config.max_sway_mps = declare_parameter<double>("max_sway_mps", config.max_sway_mps);
    config.yaw_gain_rad_s = declare_parameter<double>("yaw_gain_rad_s", config.yaw_gain_rad_s);
    config.max_yaw_rate_rad_s = declare_parameter<double>(
      "max_yaw_rate_rad_s", config.max_yaw_rate_rad_s);
    config.min_confidence = declare_parameter<double>(
      "min_confidence", config.min_confidence);
    config.max_observation_age_s = declare_parameter<double>(
      "max_observation_age_s", config.max_observation_age_s);
    config.max_future_skew_s = declare_parameter<double>(
      "max_future_skew_s", config.max_future_skew_s);
    config.yaw_control_enabled = declare_parameter<bool>(
      "yaw_control_enabled", config.yaw_control_enabled);
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 20.0);

    follower_ = std::make_unique<PipelineFollower>(config);
    intent_pub_ = create_publisher<banyu_guidance::msg::VelocityIntent>(output_topic, 10);
    observation_sub_ = create_subscription<banyu_msgs::msg::PipelineObservation>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&PipelineGuidanceNode::observationCallback, this, std::placeholders::_1));

    const double safe_rate_hz = std::isfinite(publish_rate_hz) && publish_rate_hz > 0.0 ?
      publish_rate_hz : 20.0;
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / safe_rate_hz)),
      std::bind(&PipelineGuidanceNode::publishIntent, this));
  }

private:
  void observationCallback(const banyu_msgs::msg::PipelineObservation::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    PipelineObservation observation;
    observation.frame_sequence = msg->frame_sequence;
    observation.status = static_cast<PipelineStatus>(msg->status);
    observation.stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    observation.offset_x = msg->offset_x;
    observation.offset_y = msg->offset_y;
    observation.heading_error_rad = msg->heading_error_rad;
    observation.confidence = msg->confidence;

    std::lock_guard<std::mutex> lock(observation_mutex_);
    if (have_sequence_ && observation.frame_sequence <= last_sequence_) {
      observation.status = PipelineStatus::Invalid;
    } else {
      last_sequence_ = observation.frame_sequence;
      have_sequence_ = true;
    }
    latest_observation_ = observation;
  }

  void publishIntent()
  {
    const auto now = get_clock()->now();
    std::optional<PipelineObservation> observation;
    {
      std::lock_guard<std::mutex> lock(observation_mutex_);
      observation = latest_observation_;
    }

    VelocityIntent intent;
    if (observation.has_value()) {
      intent = follower_->compute(*observation, now.nanoseconds());
    } else {
      intent.status = IntentStatus::Stale;
    }

    banyu_guidance::msg::VelocityIntent message;
    message.header.stamp = now;
    message.header.frame_id = intent_frame_;
    message.status = static_cast<std::uint8_t>(intent.status);
    message.surge_mps = static_cast<float>(intent.surge_mps);
    message.sway_mps = static_cast<float>(intent.sway_mps);
    message.yaw_rate_rad_s = static_cast<float>(intent.yaw_rate_rad_s);
    message.yaw_control_active = intent.yaw_control_active;
    intent_pub_->publish(message);
  }

  std::unique_ptr<PipelineFollower> follower_;
  std::string intent_frame_;
  rclcpp::Publisher<banyu_guidance::msg::VelocityIntent>::SharedPtr intent_pub_;
  rclcpp::Subscription<banyu_msgs::msg::PipelineObservation>::SharedPtr observation_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex observation_mutex_;
  std::optional<PipelineObservation> latest_observation_;
  std::uint64_t last_sequence_{0};
  bool have_sequence_{false};
};

}  // namespace banyu_guidance

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<banyu_guidance::PipelineGuidanceNode>());
  rclcpp::shutdown();
  return 0;
}
