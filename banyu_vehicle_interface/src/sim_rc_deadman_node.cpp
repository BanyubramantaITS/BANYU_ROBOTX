#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "mavros_msgs/msg/rc_in.hpp"
#include "rclcpp/rclcpp.hpp"

namespace banyu_vehicle_interface
{

class SimRcDeadmanNode : public rclcpp::Node
{
public:
  SimRcDeadmanNode()
  : Node("sim_rc_deadman_node")
  {
    const auto rc_topic = declare_parameter<std::string>("rc_topic", "/sim/supervisor_rc");
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 10.0);
    const int num_channels = declare_parameter<int>("num_channels", 8);
    const int channel_value = declare_parameter<int>("channel_value", 1500);

    publisher_ = create_publisher<mavros_msgs::msg::RCIn>(rc_topic, rclcpp::QoS(10));

    const double safe_rate_hz = std::max(0.1, publish_rate_hz);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / safe_rate_hz));

    const size_t safe_num_channels = static_cast<size_t>(std::max(0, num_channels));
    const uint16_t safe_channel_val = static_cast<uint16_t>(
      std::clamp(channel_value, 0, 65535));

    timer_ = create_wall_timer(
      period,
      [this, safe_num_channels, safe_channel_val]() {
        mavros_msgs::msg::RCIn msg;
        msg.header.stamp = now();
        msg.channels.assign(safe_num_channels, safe_channel_val);
        publisher_->publish(msg);
      });
  }

private:
  rclcpp::Publisher<mavros_msgs::msg::RCIn>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace banyu_vehicle_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<banyu_vehicle_interface::SimRcDeadmanNode>());
  rclcpp::shutdown();
  return 0;
}
