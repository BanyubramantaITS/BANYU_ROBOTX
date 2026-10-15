#ifndef BANYU_VEHICLE_INTERFACE__ESP32_BRIDGE_NODE_HPP_
#define BANYU_VEHICLE_INTERFACE__ESP32_BRIDGE_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace banyu_vehicle_interface
{

class Esp32BridgeNode : public rclcpp::Node
{
public:
  explicit Esp32BridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Esp32BridgeNode() override;

private:
  bool openSerialPort();
  void closeSerialPort();
  void heartbeatCallback();
  void readerLoop();

  std::string port_;
  int baud_;
  double heartbeat_rate_hz_;
  double serial_timeout_s_;

  int serial_fd_{-1};
  std::atomic<bool> is_running_{false};

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr kill_switch_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  std::thread reader_thread_;
};

}  // namespace banyu_vehicle_interface

#endif  // BANYU_VEHICLE_INTERFACE__ESP32_BRIDGE_NODE_HPP_
