#include "banyu_teleop/banyu_teleop.hpp"

#include <algorithm>
#include <chrono>

namespace banyu_teleop
{

BanyuTeleop::BanyuTeleop(const rclcpp::NodeOptions & options)
: Node("banyu_teleop", options)
{
  declareParameters();
  initCommunication();
  openSerial();

  RCLCPP_INFO(this->get_logger(), "BanyuTeleop node initialized (mock auto mode: %s).",
    _enableMockAuto ? "ENABLED" : "DISABLED");
}

BanyuTeleop::~BanyuTeleop()
{
  if (_mockAutoActive) {
    stopMockAutoMode(false);
  }
  if (_serialConnected && _serialPort && _serialPort->is_open()) {
    try {
      _serialPort->close();
    } catch (...) {
    }
  }
}

void BanyuTeleop::declareParameters()
{
  this->declare_parameter<std::string>("rc_topic", "/mavros/rc/in");
  this->declare_parameter<std::string>("kill_switch_topic", "/banyu/hardware/kill_switch");
  this->declare_parameter<std::string>("override_topic", "/mavros/rc/override");
  this->declare_parameter<std::string>("setpoint_topic", "/mavros/setpoint_raw/local");
  this->declare_parameter<std::string>("state_topic", "/mavros/state");

  this->declare_parameter<int>("arm_channel", 6);
  this->declare_parameter<int>("arm_threshold", 1500);
  this->declare_parameter<int>("mode_channel", 7);
  this->declare_parameter<int>("kill_channel", 5);
  this->declare_parameter<double>("switch_low_threshold", 1200.0);
  this->declare_parameter<double>("switch_high_threshold", 1750.0);

  this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("serial_baud", 115200);

  // Fallback / alternate nested parameters (matches hardware_params.yaml & por_params.yaml)
  this->declare_parameter<int>("rc_in.arm_channel", -1);
  this->declare_parameter<int>("rc_in.arm_threshold", -1);
  this->declare_parameter<int>("rc_in.mode_channel", -1);
  this->declare_parameter<int>("rc_in.kill_channel", -1);
  this->declare_parameter<std::string>("rc_in.serial_port", "");
  this->declare_parameter<int>("rc_in.baud_rate", -1);
  this->declare_parameter<std::string>("rc_in.kill_switch_topic", "");

  // Mock auto / mission mode parameters
  this->declare_parameter<bool>("enable_mock_auto", true);
  this->declare_parameter<bool>("auto_arm_on_auto_mode", true);
  this->declare_parameter<std::string>("mock_auto_flight_mode", "MANUAL");
  this->declare_parameter<double>("mock_auto_publish_rate_hz", 20.0);

  this->declare_parameter<int>("mock_auto_forward_pwm", 1540);
  this->declare_parameter<int>("mock_auto_throttle_pwm", 1540);
  this->declare_parameter<int>("mock_auto_lateral_pwm", 1500);
  this->declare_parameter<int>("mock_auto_yaw_pwm", 1500);
  this->declare_parameter<int>("mock_auto_pitch_pwm", 1500);
  this->declare_parameter<int>("mock_auto_roll_pwm", 1500);

  this->declare_parameter<double>("mock_auto_linear_x", 0.1);
  this->declare_parameter<double>("mock_auto_linear_z", 0.1);

  this->declare_parameter<int>("pitch_channel", 1);
  this->declare_parameter<int>("roll_channel", 2);
  this->declare_parameter<int>("throttle_channel", 3);
  this->declare_parameter<int>("yaw_channel", 4);
  this->declare_parameter<int>("forward_channel", 5);
  this->declare_parameter<int>("lateral_channel", 6);

  // Retrieve values
  this->get_parameter("rc_topic", _rcTopic);
  this->get_parameter("kill_switch_topic", _killSwitchTopic);
  this->get_parameter("override_topic", _overrideTopic);
  this->get_parameter("setpoint_topic", _setpointTopic);
  this->get_parameter("state_topic", _stateTopic);

  this->get_parameter("arm_channel", _armChannel);
  this->get_parameter("arm_threshold", _armThreshold);
  this->get_parameter("mode_channel", _modeChannel);
  this->get_parameter("kill_channel", _killChannel);
  this->get_parameter("switch_low_threshold", _switchLowThreshold);
  this->get_parameter("switch_high_threshold", _switchHighThreshold);

  this->get_parameter("serial_port", _serialDevice);
  this->get_parameter("serial_baud", _serialBaud);

  // Apply rc_in.* overrides if set
  int nested_arm_ch = -1;
  this->get_parameter("rc_in.arm_channel", nested_arm_ch);
  if (nested_arm_ch >= 0) {
    _armChannel = nested_arm_ch;
  }

  int nested_arm_thresh = -1;
  this->get_parameter("rc_in.arm_threshold", nested_arm_thresh);
  if (nested_arm_thresh >= 0) {
    _armThreshold = nested_arm_thresh;
  }

  int nested_mode_ch = -1;
  this->get_parameter("rc_in.mode_channel", nested_mode_ch);
  if (nested_mode_ch >= 0) {
    _modeChannel = nested_mode_ch;
  }

  int nested_kill_ch = -1;
  this->get_parameter("rc_in.kill_channel", nested_kill_ch);
  if (nested_kill_ch >= 0) {
    _killChannel = nested_kill_ch;
  }

  std::string nested_serial_port;
  this->get_parameter("rc_in.serial_port", nested_serial_port);
  if (!nested_serial_port.empty()) {
    _serialDevice = nested_serial_port;
  }

  int nested_baud = -1;
  this->get_parameter("rc_in.baud_rate", nested_baud);
  if (nested_baud > 0) {
    _serialBaud = nested_baud;
  }

  std::string nested_kill_topic;
  this->get_parameter("rc_in.kill_switch_topic", nested_kill_topic);
  if (!nested_kill_topic.empty()) {
    _killSwitchTopic = nested_kill_topic;
  }

  this->get_parameter("enable_mock_auto", _enableMockAuto);
  this->get_parameter("auto_arm_on_auto_mode", _autoArmOnAutoMode);
  this->get_parameter("mock_auto_flight_mode", _mockAutoFlightMode);
  this->get_parameter("mock_auto_publish_rate_hz", _mockAutoPublishRateHz);

  this->get_parameter("mock_auto_forward_pwm", _mockAutoForwardPwm);
  this->get_parameter("mock_auto_throttle_pwm", _mockAutoThrottlePwm);
  this->get_parameter("mock_auto_lateral_pwm", _mockAutoLateralPwm);
  this->get_parameter("mock_auto_yaw_pwm", _mockAutoYawPwm);
  this->get_parameter("mock_auto_pitch_pwm", _mockAutoPitchPwm);
  this->get_parameter("mock_auto_roll_pwm", _mockAutoRollPwm);

  this->get_parameter("mock_auto_linear_x", _mockAutoLinearX);
  this->get_parameter("mock_auto_linear_z", _mockAutoLinearZ);

  this->get_parameter("pitch_channel", _pitchChannel);
  this->get_parameter("roll_channel", _rollChannel);
  this->get_parameter("throttle_channel", _throttleChannel);
  this->get_parameter("yaw_channel", _yawChannel);
  this->get_parameter("forward_channel", _forwardChannel);
  this->get_parameter("lateral_channel", _lateralChannel);

  RCLCPP_INFO(this->get_logger(), "RC input: %s", _rcTopic.c_str());
  RCLCPP_INFO(
    this->get_logger(), "Switches - arm:%d mode:%d kill:%d (low<%.0f, high>%.0f)", _armChannel,
    _modeChannel, _killChannel, _switchLowThreshold, _switchHighThreshold);
}

void BanyuTeleop::initCommunication()
{
  _rcinSubscriber = this->create_subscription<mavros_msgs::msg::RCIn>(
    _rcTopic, 10, std::bind(&BanyuTeleop::rcinCallback, this, std::placeholders::_1));

  _stateSubscriber = this->create_subscription<mavros_msgs::msg::State>(
    _stateTopic, 10, std::bind(&BanyuTeleop::stateCallback, this, std::placeholders::_1));

  _killSwitchPublisher = this->create_publisher<std_msgs::msg::UInt8>(_killSwitchTopic, 10);
  _rcOverridePub = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(_overrideTopic, 10);
  _setpointPub = this->create_publisher<mavros_msgs::msg::PositionTarget>(_setpointTopic, 10);

  _armingClient = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  _setModeClient = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
}

void BanyuTeleop::stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
  _isArmed = msg->armed;
  _currentMode = msg->mode;
}

void BanyuTeleop::handleArmSwitch(const mavros_msgs::msg::RCIn::SharedPtr & msg)
{
  if (static_cast<size_t>(_armChannel) >= msg->channels.size()) {
    return;
  }
  const auto threshold = static_cast<uint16_t>(std::clamp(_armThreshold, 0, 65535));
  const bool arm_request = msg->channels[_armChannel] >= threshold;
  if (_armStateInitialized && arm_request != _lastArmRequest) {
    if (!arm_request && _mockAutoActive) {
      stopMockAutoMode(true);
    }
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
    if (_mockAutoActive) {
      RCLCPP_INFO(
        this->get_logger(),
        "Mode switch moved to %s while Mock Auto Mode active. Saved for restore upon exit to MANUAL.",
        mode.c_str());
    } else {
      setFlightMode(mode);
    }
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

  _currentOperationalMode = operational_mode;

  if (_killSwitchPublisher) {
    std_msgs::msg::UInt8 status_msg;
    status_msg.data = operational_mode;
    _killSwitchPublisher->publish(status_msg);
  }

  if (!_killStateInitialized || command != _lastKillCommand) {
    RCLCPP_INFO(this->get_logger(), "Kill switch position changed: %d (operational_mode: %u)",
      command, operational_mode);
    writeKillCommand(command);

    if (_enableMockAuto) {
      if (operational_mode == 2) {
        startMockAutoMode();
      } else if (operational_mode == 1) {
        stopMockAutoMode(false);
      } else if (operational_mode == 0) {
        stopMockAutoMode(true);
      }
    }
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

void BanyuTeleop::startMockAutoMode()
{
  if (_mockAutoActive) {
    return;
  }
  _mockAutoActive = true;
  RCLCPP_INFO(
    this->get_logger(),
    "Starting Mock ESP32 Auto/Mission Mode (Land Testing). "
    "Commanding all thrusters slowly (Forward=%d, Throttle=%d PWM), "
    "rejecting manual RC stick commands.",
    _mockAutoForwardPwm, _mockAutoThrottlePwm);

  if (_autoArmOnAutoMode && !_isArmed) {
    RCLCPP_INFO(this->get_logger(), "Auto-arming vehicle for mock mission mode...");
    armVehicle(true);
  }

  if (!_mockAutoFlightMode.empty() && _currentMode != _mockAutoFlightMode) {
    RCLCPP_INFO(this->get_logger(), "Setting flight mode to %s for mock auto test",
      _mockAutoFlightMode.c_str());
    setFlightMode(_mockAutoFlightMode);
  }

  const double safe_rate = std::max(1.0, _mockAutoPublishRateHz);
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / safe_rate));
  _mockAutoTimer = this->create_wall_timer(
    period, std::bind(&BanyuTeleop::mockAutoTimerCallback, this));

  publishMockAutoOutputs();
}

void BanyuTeleop::stopMockAutoMode(bool disarm)
{
  if (!_mockAutoActive) {
    return;
  }
  _mockAutoActive = false;
  if (_mockAutoTimer) {
    _mockAutoTimer->cancel();
    _mockAutoTimer.reset();
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Stopping Mock ESP32 Auto/Mission Mode. Releasing RC overrides and resetting setpoints.");

  mavros_msgs::msg::OverrideRCIn release_msg;
  release_msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
  for (int i = 0; i < 3; ++i) {
    _rcOverridePub->publish(release_msg);
  }

  publishZeroSetpoint();

  if (disarm) {
    RCLCPP_INFO(this->get_logger(), "Disarming vehicle on exit from auto mode...");
    armVehicle(false);
  } else if (_modeStateInitialized && !_lastMode.empty()) {
    setFlightMode(_lastMode);
  }
}

void BanyuTeleop::mockAutoTimerCallback()
{
  if (!_mockAutoActive) {
    return;
  }
  publishMockAutoOutputs();
}

bool BanyuTeleop::isSwitchChannel(int chan_1indexed) const
{
  const int idx = chan_1indexed - 1;
  return idx == _armChannel || idx == _modeChannel || idx == _killChannel;
}

void BanyuTeleop::publishMockAutoOutputs()
{
  mavros_msgs::msg::OverrideRCIn override_msg;
  override_msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);

  if (_pitchChannel >= 1 && _pitchChannel <= 18 && !isSwitchChannel(_pitchChannel)) {
    override_msg.channels[_pitchChannel - 1] = static_cast<uint16_t>(_mockAutoPitchPwm);
  }
  if (_rollChannel >= 1 && _rollChannel <= 18 && !isSwitchChannel(_rollChannel)) {
    override_msg.channels[_rollChannel - 1] = static_cast<uint16_t>(_mockAutoRollPwm);
  }
  if (_yawChannel >= 1 && _yawChannel <= 18 && !isSwitchChannel(_yawChannel)) {
    override_msg.channels[_yawChannel - 1] = static_cast<uint16_t>(_mockAutoYawPwm);
  }
  if (_lateralChannel >= 1 && _lateralChannel <= 18 && !isSwitchChannel(_lateralChannel)) {
    override_msg.channels[_lateralChannel - 1] = static_cast<uint16_t>(_mockAutoLateralPwm);
  }

  if (_forwardChannel >= 1 && _forwardChannel <= 18 && !isSwitchChannel(_forwardChannel)) {
    override_msg.channels[_forwardChannel - 1] = static_cast<uint16_t>(_mockAutoForwardPwm);
  } else if (!isSwitchChannel(_pitchChannel) && _pitchChannel >= 1 && _pitchChannel <= 18) {
    override_msg.channels[_pitchChannel - 1] = static_cast<uint16_t>(_mockAutoForwardPwm);
  }

  if (_throttleChannel >= 1 && _throttleChannel <= 18 && !isSwitchChannel(_throttleChannel)) {
    override_msg.channels[_throttleChannel - 1] = static_cast<uint16_t>(_mockAutoThrottlePwm);
  }

  if (_armChannel >= 0 && _armChannel < 18) {
    override_msg.channels[_armChannel] = mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE;
  }
  if (_modeChannel >= 0 && _modeChannel < 18) {
    override_msg.channels[_modeChannel] = mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE;
  }
  if (_killChannel >= 0 && _killChannel < 18) {
    override_msg.channels[_killChannel] = mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE;
  }

  _rcOverridePub->publish(override_msg);

  if (_currentMode == "GUIDED") {
    mavros_msgs::msg::PositionTarget target;
    target.header.stamp = this->now();
    target.header.frame_id = "base_link";
    target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;
    target.type_mask = 0x5C7;
    target.velocity.x = _mockAutoLinearX;
    target.velocity.y = 0.0;
    target.velocity.z = _mockAutoLinearZ;
    target.yaw_rate = 0.0;
    _setpointPub->publish(target);
  }
}

void BanyuTeleop::publishZeroSetpoint()
{
  mavros_msgs::msg::PositionTarget target;
  target.header.stamp = this->now();
  target.header.frame_id = "base_link";
  target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;
  target.type_mask = 0x5C7;
  target.velocity.x = 0.0;
  target.velocity.y = 0.0;
  target.velocity.z = 0.0;
  target.yaw_rate = 0.0;
  _setpointPub->publish(target);
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
          _isArmed = arm;
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
          _currentMode = mode;
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
  if (_serialDevice.empty() || _serialDevice == "none") {
    _serialConnected = false;
    return;
  }

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
    RCLCPP_WARN(
      this->get_logger(), "Could not open kill switch serial %s: %s (continuing without hardware ESP32 serial)",
      _serialDevice.c_str(), e.what());
  }
}

void BanyuTeleop::writeKillCommand(int command)
{
  if (_serialDevice.empty() || _serialDevice == "none") {
    return;
  }
  if (!_serialConnected || !_serialPort) {
    openSerial();
  }
  if (!_serialConnected || !_serialPort) {
    return;
  }

  const std::string payload = std::to_string(command);
  try {
    boost::asio::write(*_serialPort, boost::asio::buffer(payload));
    RCLCPP_INFO(this->get_logger(), "Kill switch command sent to ESP32: %d", command);
  } catch (const boost::system::system_error & e) {
    _serialPort.reset();
    _serialConnected = false;
    RCLCPP_WARN(
      this->get_logger(), "Failed to send kill switch command %d: %s", command, e.what());
  }
}

}   // namespace banyu_teleop
