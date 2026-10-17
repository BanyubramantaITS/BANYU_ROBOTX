#pragma once

#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/asio.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace banyu_teleop
{

class BanyuTeleop : public rclcpp::Node
{
public:
  BanyuTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void rcinCallback(const mavros_msgs::msg::RCIn::SharedPtr msg);
  void handleArmSwitch(const mavros_msgs::msg::RCIn::SharedPtr & msg);
  void handleModeSwitch(const mavros_msgs::msg::RCIn::SharedPtr & msg);
  void handleKillSwitch(const mavros_msgs::msg::RCIn::SharedPtr & msg);

  void armVehicle(bool arm);
  void setFlightMode(const std::string & mode);
  void declareParameters();

  void openSerial();
  void writeKillCommand(int command);

  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr _rcinSubscriber;

  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr _armingClient;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr _setModeClient;

  std::string _rcTopic;

  int _armChannel;
  int _armThreshold;

  int _modeChannel;
  int _killChannel;
  double _switchLowThreshold;
  double _switchHighThreshold;

  std::string _serialDevice;
  int _serialBaud;

  bool _armStateInitialized = false;
  bool _lastArmRequest = false;

  std::string _lastMode;
  bool _modeStateInitialized = false;

  int _lastKillCommand = -1;
  bool _killStateInitialized = false;

  boost::asio::io_context _io;
  std::unique_ptr<boost::asio::serial_port> _serialPort;
  bool _serialConnected = false;
};

}   // namespace banyu_teleop
