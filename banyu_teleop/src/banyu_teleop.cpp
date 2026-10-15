#include "banyu_teleop/banyu_teleop.hpp"

namespace banyu_teleop
{

BanyuTeleop::BanyuTeleop(const rclcpp::NodeOptions & options)
: Node("banyu_teleop", options)
{
  declareParameters();

  _joySubscriber = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&BanyuTeleop::joyCallback, this, std::placeholders::_1));

  _manualControlPub = this->create_publisher<mavros_msgs::msg::ManualControl>(
    "mavros/manual_control/send", 10);

  _armingClient = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
  _setModeClient = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

  RCLCPP_INFO(this->get_logger(), "BanyuTeleop node has been initialized.");
}

void BanyuTeleop::declareParameters()
{
  this->declare_parameter<int>("axis_x", 1);
  this->declare_parameter<int>("axis_y", 0);
  this->declare_parameter<int>("axis_z", 4);
  this->declare_parameter<int>("axis_r", 3);
  this->declare_parameter<int>("axis_s", 7);
  this->declare_parameter<int>("axis_t", 6);

  this->declare_parameter<double>("scale_x", 1000);
  this->declare_parameter<double>("scale_y", 1000);
  this->declare_parameter<double>("scale_z", 1000);
  this->declare_parameter<double>("scale_r", 1000);

  this->declare_parameter<int>("button_arm", 9);
  this->declare_parameter<int>("button_disarm", 8);
  this->declare_parameter<int>("button_shift", 0);
  this->declare_parameter<int>("button_mode_manual", 1);
  this->declare_parameter<int>("button_mode_althold", 3);
  this->declare_parameter<int>("button_mode_stabilize", 2);

  this->get_parameter("axis_x", _axisX);
  this->get_parameter("axis_y", _axisY);
  this->get_parameter("axis_z", _axisZ);
  this->get_parameter("axis_r", _axisR);
  this->get_parameter("axis_s", _axisS);
  this->get_parameter("axis_t", _axisT);

  this->get_parameter("scale_x", _scaleX);
  this->get_parameter("scale_y", _scaleY);
  this->get_parameter("scale_z", _scaleZ);
  this->get_parameter("scale_r", _scaleR);

  this->get_parameter("button_arm", _btnArm);
  this->get_parameter("button_disarm", _btnDisarm);
  this->get_parameter("button_shift", _btnShift);
  this->get_parameter("button_mode_manual", _btnModeManual);
  this->get_parameter("button_mode_althold", _btnModeAltHold);
  this->get_parameter("button_mode_stabilize", _btnModeStabilize);

  RCLCPP_INFO(this->get_logger(), "Parameters loaded successfully:");
  RCLCPP_INFO(this->get_logger(), "  Surge: axis %d, scale %.2f", _axisX, _scaleX);
  RCLCPP_INFO(this->get_logger(), "  Sway: axis %d, scale %.2f", _axisY, _scaleY);
  RCLCPP_INFO(this->get_logger(), "  Heave: axis %d, scale %.2f", _axisZ, _scaleZ);
  RCLCPP_INFO(this->get_logger(), "  Yaw: axis %d, scale %.2f", _axisR, _scaleR);
  RCLCPP_INFO(
    this->get_logger(), "  Gain/Trim: D-pad axis %d (gain), axis %d (trim pitch/roll w/ shift)",
      _axisS,
    _axisT);
  RCLCPP_INFO(this->get_logger(), "  Shift: btn %d", _btnShift);
  RCLCPP_INFO(this->get_logger(), "  Arm: btn %d, Disarm: btn %d", _btnArm, _btnDisarm);
  RCLCPP_INFO(
    this->get_logger(), "  Manual Mode: btn %d, AltHold Mode: btn %d, Stabilize Mode: btn %d",
    _btnModeManual, _btnModeAltHold, _btnModeStabilize);
}

void BanyuTeleop::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (_lastButtonsState.empty()) {
    _lastButtonsState.resize(msg->buttons.size(), 0);
  }

  size_t num_axes = msg->axes.size();
  size_t num_buttons = msg->buttons.size();

  if (static_cast<size_t>(_btnArm) < num_buttons) {
    if (msg->buttons[_btnArm] == 1 && _lastButtonsState[_btnArm] == 0) {
      armVehicle(true);
    }
  }
  if (static_cast<size_t>(_btnDisarm) < num_buttons) {
    if (msg->buttons[_btnDisarm] == 1 && _lastButtonsState[_btnDisarm] == 0) {
      armVehicle(false);
    }
  }

  if (static_cast<size_t>(_btnModeManual) < num_buttons) {
    if (msg->buttons[_btnModeManual] == 1 && _lastButtonsState[_btnModeManual] == 0) {
      setFlightMode("MANUAL");
    }
  }
  if (static_cast<size_t>(_btnModeAltHold) < num_buttons) {
    if (msg->buttons[_btnModeAltHold] == 1 && _lastButtonsState[_btnModeAltHold] == 0) {
      setFlightMode("ALT_HOLD");
    }
  }
  if (static_cast<size_t>(_btnModeStabilize) < num_buttons) {
    if (msg->buttons[_btnModeStabilize] == 1 && _lastButtonsState[_btnModeStabilize] == 0) {
      setFlightMode("STABILIZE");
    }
  }

  uint16_t manual_control_buttons = 0;

  bool shift_held = static_cast<size_t>(_btnShift) < num_buttons && msg->buttons[_btnShift] == 1;
  if (shift_held) {
    manual_control_buttons |= (1 << 5);
  }

  if (static_cast<size_t>(_axisS) < num_axes) {
    // trim pitch decrease
    if (_lastDpadV == 0 && msg->axes[_axisS] == 1) {
      manual_control_buttons |= (1 << 11);
    }
    // Trim Pitch Increase
    if (_lastDpadV == 0 && msg->axes[_axisS] == -1) {
      manual_control_buttons |= (1 << 12);
    }
    _lastDpadV = msg->axes[_axisS];
  }

  if (shift_held && static_cast<size_t>(_axisT) < num_axes) {
    // Trim Roll Increase
    if (_lastDpadH == 0 && msg->axes[_axisT] == -1) {
      manual_control_buttons |= (1 << 14);
    }
    // Trim Roll Decrease
    if (_lastDpadH == 0 && msg->axes[_axisT] == 1) {
      manual_control_buttons |= (1 << 13);
    }
    _lastDpadH = msg->axes[_axisT];
  }

  _lastButtonsState = msg->buttons;

  mavros_msgs::msg::ManualControl manual_control_msg;
  manual_control_msg.header.stamp = this->now();
  manual_control_msg.header.frame_id = "base_link";

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double r = 0.0;

  if (static_cast<size_t>(_axisX) < num_axes) {
    x = msg->axes[_axisX] * _scaleX;
  }
  if (static_cast<size_t>(_axisY) < num_axes) {
    y = msg->axes[_axisY] * _scaleY * -1;
  }
  if (static_cast<size_t>(_axisZ) < num_axes) {
    z = (msg->axes[_axisZ] * _scaleZ) + _scaleZ;
  }
  if (static_cast<size_t>(_axisR) < num_axes) {
    r = msg->axes[_axisR] * _scaleR * -1;
  }

  manual_control_msg.x = x;
  manual_control_msg.y = y;
  manual_control_msg.z = z;
  manual_control_msg.r = r;
  manual_control_msg.buttons = manual_control_buttons;

  _manualControlPub->publish(manual_control_msg);
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

}  // namespace banyu_teleop

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<banyu_teleop::BanyuTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
