#include "banyu_teleop/banyu_teleop.hpp"

#include <algorithm>
#include <chrono>

namespace banyu_teleop
{

BanyuTeleop::BanyuTeleop(const rclcpp::NodeOptions & options)
: Node("banyu_teleop", options)
{
  declareParameters();

  _rcinSubscriber = this->create_subscription<mavros_msgs::msg::RCIn>(
    _rcTopic, 10, std::bind(&BanyuTeleop::rcinCallback, this, std::placeholders::_1));
  _killSwitchPublisher = this->create_publisher<std_msgs::msg::UInt8>(_killSwitchTopic, 10);

  _armingClient = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  _setModeClient = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

  openSerial();

  RCLCPP_INFO(this->get_logger(), "BanyuTeleop node has been initialized.");
}

void BanyuTeleop::declareParameters()
{
  this->declare_parameter<std::string>("rc_topic", "/mavros/rc/in");
  this->declare_parameter<std::string>("kill_switch_topic", "/banyu/hardware/kill_switch");

  this->declare_parameter<int>("arm_channel", 6);
  this->declare_parameter<int>("arm_threshold", 1500);

  this->declare_parameter<int>("mode_channel", 7);
  this->declare_parameter<int>("kill_channel", 5);
  this->declare_parameter<double>("switch_low_threshold", 1200.0);
  this->declare_parameter<double>("switch_high_threshold", 1750.0);

  this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("serial_baud", 115200);

  this->get_parameter("rc_topic", _rcTopic);
  this->get_parameter("kill_switch_topic", _killSwitchTopic);

  this->get_parameter("arm_channel", _armChannel);
  this->get_parameter("arm_threshold", _armThreshold);

  this->get_parameter("mode_channel", _modeChannel);
  this->get_parameter("kill_channel", _killChannel);
  this->get_parameter("switch_low_threshold", _switchLowThreshold);
  this->get_parameter("switch_high_threshold", _switchHighThreshold);

  this->get_parameter("serial_port", _serialDevice);
  this->get_parameter("serial_baud", _serialBaud);

  RCLCPP_INFO(this->get_logger(), "RC input: %s", _rcTopic.c_str());
  RCLCPP_INFO(
    this->get_logger(), "Switches - arm:%d mode:%d kill:%d (low<%.0f, high>%.0f)", _armChannel,
    _modeChannel, _killChannel, _switchLowThreshold, _switchHighThreshold);
}

void BanyuTeleop::handleArmSwitch(const mavros_msgs::msg::RCIn::SharedPtr & msg)
{
  if (static_cast<size_t>(_armChannel) >= msg->channels.size()) {
    return;
  }
  const auto threshold = static_cast<uint16_t>(std::clamp(_armThreshold, 0, 65535));
  const bool arm_request = msg->channels[_armChannel] >= threshold;
  if (_armStateInitialized && arm_request != _lastArmRequest) {
    armVehicle(arm_request);
  }
  _lastArmRequest = arm_request;
  _armStateInitialized = true;
}

void BanyuTeleop::handleModeSwitch(const mavros_msgs::msg::RCIn::SharedPtr & msg)
{
  if (static_cast<size_t>(_modeChannel) >= msg->channels.size()) {
    return;
  }
  const uint16_t pwm = msg->channels[_modeChannel];

  std::string mode;
  if (pwm < _switchLowThreshold) {
    mode = "STABILIZE";
  } else if (pwm > _switchHighThreshold) {
    mode = "ALT_HOLD";
  } else {
    mode = "MANUAL";
  }

  if (!_modeStateInitialized) {
    RCLCPP_INFO(this->get_logger(), "Mode switch initial position: %s", mode.c_str());
  } else if (mode != _lastMode) {
    setFlightMode(mode);
  }
  _lastMode = mode;
  _modeStateInitialized = true;
}

void BanyuTeleop::handleKillSwitch(const mavros_msgs::msg::RCIn::SharedPtr & msg)
{
  if (static_cast<size_t>(_killChannel) >= msg->channels.size()) {
    return;
  }
  const uint16_t pwm = msg->channels[_killChannel];

  int command;
  uint8_t operational_mode;
  if (pwm < _switchLowThreshold) {
    command = 0;            // ESP32: '0' = autonomous
    operational_mode = 2;   // VehicleStatus: 2 = AUTONOMOUS
  } else if (pwm > _switchHighThreshold) {
    command = 2;            // ESP32: '2' = kill
    operational_mode = 0;   // VehicleStatus: 0 = DISARMED / KILL
  } else {
    command = 1;            // ESP32: '1' = remote
    operational_mode = 1;   // VehicleStatus: 1 = MANUAL
  }

  if (_killSwitchPublisher) {
    std_msgs::msg::UInt8 status_msg;
    status_msg.data = operational_mode;
    _killSwitchPublisher->publish(status_msg);
  }

  if (!_killStateInitialized) {
    RCLCPP_INFO(this->get_logger(), "Kill switch initial position: %d", command);
    writeKillCommand(command);
  } else if (command != _lastKillCommand) {
    writeKillCommand(command);
  }
  _lastKillCommand = command;
  _killStateInitialized = true;
}

void BanyuTeleop::rcinCallback(const mavros_msgs::msg::RCIn::SharedPtr msg)
{
  handleArmSwitch(msg);
  handleModeSwitch(msg);
  handleKillSwitch(msg);
}

void BanyuTeleop::armVehicle(bool arm)
{
  if (!_armingClient->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Arming service not available");
    return;
  }

  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = arm;

  RCLCPP_INFO(this->get_logger(), "Sending %s command...", arm ? "Arming" : "Disarming");

  auto result = _armingClient->async_send_request(
    request, [this, arm](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
      try {
        auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(this->get_logger(), "Vehicle successfully %s", arm ? "ARMED" : "DISARMED");
        } else {
          RCLCPP_ERROR(
            this->get_logger(), "Failed to %s vehicle (result: %d)", arm ? "arm" : "disarm",
            response->result);
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Arming service call threw an exception: %s", e.what());
      }
    });
}

void BanyuTeleop::setFlightMode(const std::string & mode)
{
  if (!_setModeClient->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "SetMode service not available");
    return;
  }

  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = mode;

  RCLCPP_INFO(this->get_logger(), "Requesting mode switch to: %s", mode.c_str());

  auto result = _setModeClient->async_send_request(
    request, [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      try {
        auto response = future.get();
        if (response->mode_sent) {
          RCLCPP_INFO(this->get_logger(), "Flight mode command sent for: %s", mode.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to set flight mode to %s", mode.c_str());
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "SetMode service call threw an exception: %s", e.what());
      }
    });
}

void BanyuTeleop::openSerial()
{
  try {
    _serialPort = std::make_unique<boost::asio::serial_port>(_io, _serialDevice);
    _serialPort->set_option(boost::asio::serial_port_base::baud_rate(_serialBaud));
    _serialPort->set_option(boost::asio::serial_port_base::character_size(8u));
    _serialPort->set_option(
      boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));
    _serialPort->set_option(boost::asio::serial_port_base::parity(
      boost::asio::serial_port_base::parity::none));
    _serialPort->set_option(boost::asio::serial_port_base::stop_bits(
      boost::asio::serial_port_base::stop_bits::one));
    _serialConnected = true;
    RCLCPP_INFO(
      this->get_logger(), "Kill switch serial connected on %s @ %d baud", _serialDevice.c_str(),
      _serialBaud);
  } catch (const boost::system::system_error & e) {
    _serialPort.reset();
    _serialConnected = false;
    RCLCPP_ERROR(
      this->get_logger(), "Failed to open kill switch serial %s: %s", _serialDevice.c_str(),
      e.what());
  }
}

void BanyuTeleop::writeKillCommand(int command)
{
  if (!_serialConnected || !_serialPort) {
    openSerial();
  }
  if (!_serialConnected || !_serialPort) {
    RCLCPP_WARN(
      this->get_logger(), "Kill switch command %d not sent: serial port unavailable", command);
    return;
  }

  const std::string payload = std::to_string(command);
  try {
    boost::asio::write(*_serialPort, boost::asio::buffer(payload));
    RCLCPP_INFO(this->get_logger(), "Kill switch command sent: %d", command);
  } catch (const boost::system::system_error & e) {
    _serialPort.reset();
    _serialConnected = false;
    RCLCPP_ERROR(
      this->get_logger(), "Failed to send kill switch command %d: %s", command, e.what());
  }
}

}   // namespace banyu_teleop

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<banyu_teleop::BanyuTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
