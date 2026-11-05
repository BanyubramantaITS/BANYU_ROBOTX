#ifndef BANYU_VEHICLE_INTERFACE__ESP32_TRIGGER_NODE_HPP_
#define BANYU_VEHICLE_INTERFACE__ESP32_TRIGGER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <banyu_msgs/srv/trigger_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace banyu_vehicle_interface
{

class Esp32TriggerNode : public rclcpp::Node
{
public:
  explicit Esp32TriggerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Esp32TriggerNode() override;

  bool isPortOpen() const;
  bool sendTrigger(const std::string & cmd);
  bool isHardwareKillLatched() const {return hw_kill_latched_;}
  void processIncomingData();

  void handleTrigger(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handleSetTrigger(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void handleSendCommand(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<banyu_msgs::srv::TriggerCommand::Request> request,
    std::shared_ptr<banyu_msgs::srv::TriggerCommand::Response> response);

  void handleCommandTopic(const std_msgs::msg::String::SharedPtr msg);

private:
  bool openSerialPort();
  void closeSerialPort();
  void closeSerialPortLocked();
  void timerCallback();
  void recordCommandedMode(const std::string & cmd);
  void publishCommandedMode();
  void handleBoardReport(const std::string & line);

  std::string port_{"/dev/esp32_kill"};
  int baud_{115200};
  double timer_period_s_{0.1};
  bool append_newline_{false};

  std::string default_trigger_cmd_{"2"};
  std::string trigger_on_cmd_{"2"};
  std::string trigger_off_cmd_{"0"};

  std::string trigger_service_name_{"/banyu/trigger/fire"};
  std::string set_trigger_service_name_{"/banyu/trigger/set"};
  std::string command_service_name_{"/banyu/trigger/command"};
  std::string command_topic_name_{"/banyu/trigger/cmd"};
  std::string monitor_topic_name_{"/banyu/trigger/monitor"};
  std::string kill_switch_topic_name_{"/banyu/hardware/kill_switch"};
  bool publish_kill_switch_{true};
  bool use_hardware_kill_{true};
  double kill_report_grace_s_{0.3};
  double resend_period_s_{1.0};

  // Last '0'/'1'/'2' commanded, republished as the operational mode (0 while a hardware kill
  // reported by the board is latched; a '0' command acknowledges it).
  bool has_commanded_mode_{false};
  uint8_t commanded_mode_{0};
  bool hw_kill_latched_{false};
  bool hw_kill_pending_write_{false};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time commanded_at_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_mode_write_{0, 0, RCL_STEADY_TIME};

  int serial_fd_{-1};
  mutable std::mutex serial_mutex_;
  std::string rx_buffer_;
  int reconnect_counter_{0};

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_trigger_service_;
  rclcpp::Service<banyu_msgs::srv::TriggerCommand>::SharedPtr command_service_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr monitor_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr kill_switch_publisher_;
};

}  // namespace banyu_vehicle_interface

#endif  // BANYU_VEHICLE_INTERFACE__ESP32_TRIGGER_NODE_HPP_
