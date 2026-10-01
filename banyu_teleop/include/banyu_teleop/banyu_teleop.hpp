#pragma once

#include <mavros_msgs/msg/manual_control.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>
#include <vector>

namespace banyu_teleop
{

class BanyuTeleop : public rclcpp::Node
{
public:
  BanyuTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void armVehicle(bool arm);
  void setFlightMode(const std::string & mode);
  void declareParameters();

  // Subscribers and Publishers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joySubscriber;
  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr _manualControlPub;

  // Service Clients
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr _armingClient;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr _setModeClient;

  // Cache of last button states to detect rising edge (button press)
  std::vector<int> _lastButtonsState;

  // Parameters
  int _axisX;
  int _axisY;
  int _axisZ;
  int _axisR;
  int _axisS;
  int _axisT;

  double _scaleX;
  double _scaleY;
  double _scaleZ;
  double _scaleR;
  double _scaleS;
  double _scaleT;

  int _btnArm;
  int _btnDisarm;
  int _btnShift;
  int _btnModeManual;
  int _btnModeAltHold;
  int _btnModeStabilize;

  int _lastDpadV = 0;
  int _lastDpadH = 0;
};

}   // namespace banyu_teleop
