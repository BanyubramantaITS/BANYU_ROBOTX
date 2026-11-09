#include "banyu_teleop/banyu_teleop.hpp"
#include "banyu_teleop/esp32_indicator.hpp"

#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <utility>

namespace banyu_teleop
{

BanyuTeleop::BanyuTeleop(const rclcpp::NodeOptions & options)
: Node("banyu_teleop", options)
{
  declareParameters();
  initCommunication();
  openSerial();

  // Reads the ESP32 state reports and keeps its LEDs in sync (they reset when the board reboots).
  _indicatorTimer = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&BanyuTeleop::indicatorTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "BanyuTeleop node initialized (mock auto mode: %s).",
    _enableMockAuto ? "ENABLED" : "DISABLED");
}

BanyuTeleop::~BanyuTeleop()
{
  if (_mockAutoActive) {
    stopMockAutoMode();
  }
  if (_gimbalOverrideActive) {
    publishReleaseOverride();
    _gimbalOverrideActive = false;
  }
  if (_lightIsOn) {
    setLightState(false);
  }
  closeSerial();
}

void BanyuTeleop::declareParameters()
{
  this->declare_parameter<std::string>("rc_topic", "/mavros/rc/in");
  this->declare_parameter<std::string>("kill_switch_topic", "/banyu/hardware/kill_switch");
  this->declare_parameter<std::string>("override_topic", "/mavros/rc/override");
  this->declare_parameter<std::string>("setpoint_topic", "/mavros/setpoint_raw/local");
  this->declare_parameter<std::string>("state_topic", "/mavros/state");

  this->declare_parameter<int>("mode_channel", 7);
  this->declare_parameter<int>("kill_channel", 5);
  this->declare_parameter<double>("switch_low_threshold", 1200.0);
  this->declare_parameter<double>("switch_high_threshold", 1750.0);
  this->declare_parameter<double>("gimbal_deadband", 50.0);

  this->declare_parameter<std::string>("kill_switch_board_topic",
    "/banyu/hardware/kill_switch_board");
  this->declare_parameter<std::string>("serial_port", "/dev/esp32_kill");
  this->declare_parameter<int>("serial_baud", 115200);
  this->declare_parameter<bool>("use_hardware_kill", true);
  this->declare_parameter<double>("kill_report_timeout_s", 1.5);
  this->declare_parameter<double>("kill_report_grace_s", 0.3);
  this->declare_parameter<double>("indicator_resend_period_s", 1.0);
  this->declare_parameter<double>("rc_timeout_s", 3.0);

  // Mock auto / mission mode parameters
  this->declare_parameter<bool>("enable_mock_auto", false);
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

  this->declare_parameter<int>("pitch_channel", 0);
  this->declare_parameter<int>("roll_channel", 0);
  this->declare_parameter<int>("throttle_channel", 3);
  this->declare_parameter<int>("yaw_channel", 4);
  this->declare_parameter<int>("forward_channel", 2);
  this->declare_parameter<int>("lateral_channel", 1);

  this->declare_parameter<bool>("light_enabled", true);
  this->declare_parameter<int>("light_channel", 9);
  this->declare_parameter<double>("light_on_pwm", 1900.0);
  this->declare_parameter<double>("light_off_pwm", 1100.0);
  this->declare_parameter<std::string>("command_service", "/mavros/cmd/command");

  // Retrieve values
  this->get_parameter("rc_topic", _rcTopic);
  this->get_parameter("kill_switch_topic", _killSwitchTopic);
  this->get_parameter("override_topic", _overrideTopic);
  this->get_parameter("setpoint_topic", _setpointTopic);
  this->get_parameter("state_topic", _stateTopic);

  this->get_parameter("mode_channel", _modeChannel);
  this->get_parameter("kill_channel", _killChannel);
  this->get_parameter("switch_low_threshold", _switchLowThreshold);
  this->get_parameter("switch_high_threshold", _switchHighThreshold);
  this->get_parameter("gimbal_deadband", _gimbalDeadband);

  this->get_parameter("kill_switch_board_topic", _killBoardTopic);
  this->get_parameter("serial_port", _serialDevice);
  this->get_parameter("serial_baud", _serialBaud);
  this->get_parameter("use_hardware_kill", _useHardwareKill);
  this->get_parameter("kill_report_timeout_s", _killReportTimeoutS);
  this->get_parameter("kill_report_grace_s", _killReportGraceS);
  this->get_parameter("indicator_resend_period_s", _indicatorResendPeriodS);
  this->get_parameter("rc_timeout_s", _rcTimeoutS);

  this->get_parameter("enable_mock_auto", _enableMockAuto);
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

  this->get_parameter("light_enabled", _lightEnabled);
  this->get_parameter("light_channel", _lightChannel);
  this->get_parameter("light_on_pwm", _lightOnPwm);
  this->get_parameter("light_off_pwm", _lightOffPwm);
  this->get_parameter("command_service", _commandService);

  RCLCPP_INFO(this->get_logger(), "RC input: %s", _rcTopic.c_str());
  RCLCPP_INFO(
    this->get_logger(), "Switches - mode (SC):%d kill/arm (SB):%d (low<%.0f, high>%.0f)",
    _modeChannel, _killChannel, _switchLowThreshold, _switchHighThreshold);
  warnOnSwitchChannelOverlap();
}

void BanyuTeleop::warnOnSwitchChannelOverlap()
{
  // /mavros/rc/in reports RC override values, so a switch on a stick channel reads whatever
  // the autonomy stack streams there (e.g. neutral 1500 = SB Mid = MANUAL) instead of the switch.
  const std::pair<const char *, int> sticks[] = {
    {"pitch", _pitchChannel}, {"roll", _rollChannel}, {"throttle", _throttleChannel},
    {"yaw", _yawChannel}, {"forward", _forwardChannel}, {"lateral", _lateralChannel}};
  const std::pair<const char *, int> switches[] = {
    {"kill/arm (SB)", _killChannel}, {"flight mode (SC)", _modeChannel}};
  for (const auto & [switch_name, switch_index] : switches) {
    for (const auto & [stick_name, stick_channel] : sticks) {
      if (switch_index + 1 == stick_channel) {
        RCLCPP_ERROR(
          this->get_logger(),
          "%s switch is on CH%d, which is also the %s stick channel. RC override on that "
          "channel (POR rc_override, mock auto) is read back as the switch position. "
          "Move the switch or the stick channel (RCMAP) before an autonomous run.",
          switch_name, switch_index + 1, stick_name);
      }
    }
  }
}

void BanyuTeleop::initCommunication()
{
  _rcinSubscriber = this->create_subscription<mavros_msgs::msg::RCIn>(
    _rcTopic, 10, std::bind(&BanyuTeleop::rcinCallback, this, std::placeholders::_1));

  _stateSubscriber = this->create_subscription<mavros_msgs::msg::State>(
    _stateTopic, 10, std::bind(&BanyuTeleop::stateCallback, this, std::placeholders::_1));

  _killSwitchPublisher = this->create_publisher<std_msgs::msg::UInt8>(_killSwitchTopic, 10);
  _killBoardPublisher = this->create_publisher<std_msgs::msg::UInt8>(_killBoardTopic, 10);
  _rcOverridePub = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(_overrideTopic, 10);
  _setpointPub = this->create_publisher<mavros_msgs::msg::PositionTarget>(_setpointTopic, 10);

  _armingClient = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  _commandClient = this->create_client<mavros_msgs::srv::CommandLong>(_commandService);
  _setModeClient = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
}

void BanyuTeleop::stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
  const bool became_disarmed = _isArmed && !msg->armed;
  const bool became_armed = !_isArmed && msg->armed;
  _isArmed = msg->armed;
  _currentMode = msg->mode;

  if (became_disarmed) {
    RCLCPP_INFO(this->get_logger(), "Pixhawk disarmed: turning light OFF.");
    setLightState(false);
  } else if (became_armed && (_currentOperationalMode == 1 || _currentOperationalMode == 2)) {
    RCLCPP_INFO(this->get_logger(), "Pixhawk armed in operational mode %u: turning light ON.",
      _currentOperationalMode);
    setLightState(true);
  }
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

  const bool exited_autonomous = (_prevOperationalMode == 2 && _currentOperationalMode != 2);

  if (!_modeStateInitialized) {
    RCLCPP_INFO(this->get_logger(), "Flight mode switch (SC) initial position: %s", mode.c_str());
    _lastMode = mode;
    _modeStateInitialized = true;
  } else if (_currentOperationalMode == 2) {
    // In Autonomous mode (SB Up): reject flight mode changes from SC
    if (mode != _lastMode) {
      RCLCPP_WARN(
        this->get_logger(),
        "Flight mode change to %s rejected while in Autonomous mode (SB Up). "
        "Operator must switch SB to Manual mode first.",
        mode.c_str());
    }
    _lastMode = mode;
  } else {
    // In Manual (1) or Disarmed (0) mode:
    // Apply mode change if SC was toggled OR vehicle just exited Autonomous mode
    if (mode != _lastMode || exited_autonomous) {
      setFlightMode(mode);
    }
    _lastMode = mode;
  }
}

bool BanyuTeleop::isGimbalDeflected(const mavros_msgs::msg::RCIn::SharedPtr & msg) const
{
  const int stick_channels[] = {
    _pitchChannel, _rollChannel, _throttleChannel,
    _yawChannel, _forwardChannel, _lateralChannel};
  for (int ch : stick_channels) {
    if (ch >= 1 && static_cast<size_t>(ch - 1) < msg->channels.size() && !isSwitchChannel(ch)) {
      const uint16_t pwm = msg->channels[ch - 1];
      if (std::abs(static_cast<int>(pwm) - 1500) > _gimbalDeadband) {
        return true;
      }
    }
  }
  return false;
}

void BanyuTeleop::handleGimbalInput(const mavros_msgs::msg::RCIn::SharedPtr & msg)
{
  const bool deflected = isGimbalDeflected(msg);

  if (_currentOperationalMode == 2) {
    // In Autonomous mode (SB Up): reject manual gimbal stick input and hold neutral override
    if (deflected) {
      if (!_gimbalDeflectedLast) {
        RCLCPP_WARN(
          this->get_logger(),
          "Manual gimbal input rejected while in Autonomous mode (SB Up). "
          "Operator must switch SB to Manual mode first.");
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Manual gimbal input rejected while in Autonomous mode (SB Up). "
          "Operator must switch SB to Manual mode first.");
      }
    }
    _gimbalDeflectedLast = deflected;

    if (!_mockAutoActive) {
      publishNeutralGimbalOverride();
      _gimbalOverrideActive = true;
    }
  } else {
    // In Manual (1) or Disarmed (0) mode: gimbal input is allowed natively by ArduSub
    if (_gimbalOverrideActive) {
      if (!_mockAutoActive) {
        publishReleaseOverride();
      }
      _gimbalOverrideActive = false;
    }
    _gimbalDeflectedLast = false;
  }
}

void BanyuTeleop::handleKillSwitch(const mavros_msgs::msg::RCIn::SharedPtr & msg)
{
  if (static_cast<size_t>(_killChannel) >= msg->channels.size()) {
    return;
  }
  const uint16_t pwm = msg->channels[_killChannel];

  // operational_mode as in VehicleStatus (issue #15): 0 = disarmed / kill, 1 = manual,
  // 2 = autonomous. esp32_indicator.hpp maps it to the indicator board's own numbering.
  uint8_t operational_mode;
  if (pwm < _switchLowThreshold) {
    operational_mode = 2;   // SB Up: AUTONOMOUS
  } else if (pwm > _switchHighThreshold) {
    operational_mode = 0;   // SB Down: DISARMED / KILL
  } else {
    operational_mode = 1;   // SB Mid: MANUAL
  }
  const int command = operational_mode;

  _lastRcAt = _steadyClock.now();

  // A hardware kill holds the vehicle in kill until the input is released or SB Down clears it.
  if (_hwKillLatched && operational_mode == 0) {
    _hwKillLatched = false;
    _hwKillAwaitRelease = false;
    RCLCPP_WARN(this->get_logger(), "Hardware kill acknowledged with SB Down; latch cleared.");
  }
  const uint8_t effective_mode = _hwKillLatched ? 0 : operational_mode;
  _currentOperationalMode = effective_mode;

  if (_killSwitchPublisher) {
    std_msgs::msg::UInt8 status_msg;
    status_msg.data = effective_mode;
    _killSwitchPublisher->publish(status_msg);
  }
  // The board ignores the SB mode during a hardware kill and only leaves its2 once released;
  // commanding its2 here would hide the release.
  const uint8_t board_mode = _hwKillAwaitRelease ? operational_mode : effective_mode;
  if (board_mode != _commandedMode) {
    writeKillCommand(board_mode);
  }

  if (!_killStateInitialized || command != _lastKillCommand) {
    RCLCPP_INFO(this->get_logger(), "Kill switch position changed: %d (operational_mode: %u)",
      command, effective_mode);

    if (_hwKillLatched) {
      RCLCPP_WARN(this->get_logger(), "%s",
        _hwKillAwaitRelease ?
        "Hardware kill active: the SB position applies once it is released." :
        "Hardware kill is latched: SB change ignored. Move SB Down to acknowledge.");
    } else if (operational_mode == 0) {
      // SB Down: disarm both ESP32 kill switch and Pixhawk
      if (_gimbalOverrideActive) {
        if (!_mockAutoActive) {
          publishReleaseOverride();
        }
        _gimbalOverrideActive = false;
      }
      if (_mockAutoActive) {
        stopMockAutoMode();
      }
      armVehicle(false);
      setLightState(false);
    } else if (operational_mode == 1) {
      // SB Middle: manual mode, automatically arm Pixhawk
      if (_gimbalOverrideActive) {
        if (!_mockAutoActive) {
          publishReleaseOverride();
        }
        _gimbalOverrideActive = false;
      }
      if (_mockAutoActive) {
        stopMockAutoMode();
      }
      if (!_isArmed) {
        armVehicle(true);
      }
      setLightState(true);
    } else if (operational_mode == 2) {
      // SB Up: autonomous mode, arm Pixhawk if not armed
      if (!_isArmed) {
        armVehicle(true);
      }
      if (_enableMockAuto) {
        startMockAutoMode();
      }
      setLightState(true);
    }
  }
  _lastKillCommand = command;
  _killStateInitialized = true;
}

void BanyuTeleop::latchHardwareKill()
{
  _hwKillLatched = true;
  _hwKillAwaitRelease = true;
  _currentOperationalMode = 0;
  RCLCPP_ERROR(this->get_logger(),
    "ESP32 reports a hardware kill (emergency input active): disarming. "
    "The SB position applies again once it is released.");
  if (_gimbalOverrideActive) {
    if (!_mockAutoActive) {
      publishReleaseOverride();
    }
    _gimbalOverrideActive = false;
  }
  if (_mockAutoActive) {
    stopMockAutoMode();
  }
  armVehicle(false);
  setLightState(false);
  if (_killSwitchPublisher) {
    std_msgs::msg::UInt8 status_msg;
    status_msg.data = 0;
    _killSwitchPublisher->publish(status_msg);
  }
}

void BanyuTeleop::handleHardwareKillRelease()
{
  _hwKillAwaitRelease = false;
  const bool rc_connected = _lastRcAt.nanoseconds() != 0 &&
    (_steadyClock.now() - _lastRcAt).seconds() <= _rcTimeoutS;
  if (!rc_connected) {
    RCLCPP_ERROR(this->get_logger(),
      "Hardware kill released but RC is lost: holding kill. "
      "Move SB Down to acknowledge once RC is back.");
    writeKillCommand(0);
    return;
  }
  _hwKillLatched = false;
  _killStateInitialized = false;  // the next RC message applies the current SB position
  RCLCPP_WARN(this->get_logger(), "Hardware kill released: returning to the SB position.");
}

void BanyuTeleop::handleIndicatorLine(const std::string & line)
{
  const int reported = parseIndicatorReport(line);
  if (reported < 0) {
    return;
  }
  const rclcpp::Time now = _steadyClock.now();
  _lastReportAt = now;
  if (_reportStaleWarned) {
    RCLCPP_INFO(this->get_logger(), "ESP32 kill switch reports again.");
    _reportStaleWarned = false;
  }
  if (_killBoardPublisher) {
    std_msgs::msg::UInt8 msg;
    msg.data = static_cast<uint8_t>(reported);
    _killBoardPublisher->publish(msg);
  }

  if (_hwKillAwaitRelease && reported != 0) {
    handleHardwareKillRelease();
  }

  // The board shows EMERGENCY for a software its2 too, so an emergency only means the hardware
  // input while a non-kill mode is commanded, and only once the board had time to apply it.
  const bool settled = _commandedMode > 0 &&
    (now - _commandChangedAt).seconds() >= _killReportGraceS;
  if (_useHardwareKill && reported == 0 && settled && !_hwKillLatched) {
    latchHardwareKill();
  }
}

void BanyuTeleop::indicatorTimerCallback()
{
  if (_serialDevice.empty() || _serialDevice == "none") {
    return;
  }
  const rclcpp::Time now = _steadyClock.now();
  if (!_serialConnected) {
    if ((now - _lastReconnectAt).seconds() >= 1.0) {
      _lastReconnectAt = now;
      openSerial(true);
    }
    if (!_serialConnected) {
      return;
    }
  }

  readIndicatorSerial();

  if (_commandedMode >= 0 && (now - _lastCommandSentAt).seconds() >= _indicatorResendPeriodS) {
    writeKillCommand(_commandedMode);
  }

  const bool stale = _lastReportAt.nanoseconds() == 0 ||
    (now - _lastReportAt).seconds() > _killReportTimeoutS;
  if (stale && !_reportStaleWarned && _commandedMode >= 0) {
    RCLCPP_ERROR(this->get_logger(),
      "No state report from the ESP32 kill switch on %s for %.1f s: hardware kill not monitored.",
      _serialDevice.c_str(), _killReportTimeoutS);
    _reportStaleWarned = true;
  }
}

void BanyuTeleop::readIndicatorSerial()
{
  if (!_serialConnected || !_serialPort) {
    return;
  }
  const int fd = _serialPort->native_handle();
  int available = 0;
  if (ioctl(fd, FIONREAD, &available) < 0) {
    RCLCPP_WARN(this->get_logger(), "Kill switch serial lost on %s.", _serialDevice.c_str());
    closeSerial();
    return;
  }
  char buf[256];
  while (available > 0) {
    const ssize_t n = ::read(fd, buf, std::min(sizeof(buf), static_cast<size_t>(available)));
    if (n <= 0) {
      break;
    }
    _rxBuffer.append(buf, static_cast<size_t>(n));
    available -= static_cast<int>(n);
  }

  size_t eol;
  while ((eol = _rxBuffer.find('\n')) != std::string::npos) {
    handleIndicatorLine(_rxBuffer.substr(0, eol));
    _rxBuffer.erase(0, eol + 1);
  }
  if (_rxBuffer.size() > 1024) {
    _rxBuffer.clear();
  }
}

void BanyuTeleop::rcinCallback(const mavros_msgs::msg::RCIn::SharedPtr msg)
{
  handleKillSwitch(msg);
  handleModeSwitch(msg);
  handleGimbalInput(msg);
  _prevOperationalMode = _currentOperationalMode;
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

void BanyuTeleop::stopMockAutoMode()
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
  return idx == _modeChannel || idx == _killChannel;
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

void BanyuTeleop::publishNeutralGimbalOverride()
{
  if (!_rcOverridePub) {
    return;
  }

  mavros_msgs::msg::OverrideRCIn override_msg;
  override_msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);

  const int stick_channels[] = {
    _pitchChannel, _rollChannel, _throttleChannel,
    _yawChannel, _forwardChannel, _lateralChannel};

  for (int ch : stick_channels) {
    if (ch >= 1 && ch <= 18 && !isSwitchChannel(ch)) {
      override_msg.channels[ch - 1] = 1500;
    }
  }

  if (_modeChannel >= 0 && _modeChannel < 18) {
    override_msg.channels[_modeChannel] = mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE;
  }
  if (_killChannel >= 0 && _killChannel < 18) {
    override_msg.channels[_killChannel] = mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE;
  }

  _rcOverridePub->publish(override_msg);
}

void BanyuTeleop::publishReleaseOverride()
{
  if (!_rcOverridePub) {
    return;
  }

  mavros_msgs::msg::OverrideRCIn release_msg;
  release_msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
  for (int i = 0; i < 3; ++i) {
    _rcOverridePub->publish(release_msg);
  }
}

void BanyuTeleop::armVehicle(bool arm)
{
  if (!arm) {
    _isArmed = false;
  }

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

void BanyuTeleop::openSerial(bool quiet)
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
    _rxBuffer.clear();
    RCLCPP_INFO(
      this->get_logger(), "Kill switch serial connected on %s @ %d baud", _serialDevice.c_str(),
      _serialBaud);
  } catch (const boost::system::system_error & e) {
    _serialPort.reset();
    _serialConnected = false;
    if (!quiet) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not open kill switch serial %s: %s (continuing without hardware ESP32 serial)",
        _serialDevice.c_str(), e.what());
    }
  }
}

void BanyuTeleop::closeSerial()
{
  if (_serialPort && _serialPort->is_open()) {
    try {
      _serialPort->close();
    } catch (...) {
    }
  }
  _serialPort.reset();
  _serialConnected = false;
}

void BanyuTeleop::writeKillCommand(int command)
{
  const rclcpp::Time now = _steadyClock.now();
  if (command != _commandedMode) {
    _commandedMode = command;
    _commandChangedAt = now;
  }
  _lastCommandSentAt = now;

  if (_serialDevice.empty() || _serialDevice == "none") {
    return;
  }
  if (!_serialConnected || !_serialPort) {
    openSerial(true);
  }
  if (!_serialConnected || !_serialPort) {
    return;
  }

  const std::string payload = indicatorCommand(static_cast<uint8_t>(command));
  try {
    boost::asio::write(*_serialPort, boost::asio::buffer(payload));
    RCLCPP_DEBUG(this->get_logger(), "Kill switch command sent to ESP32: %d", command);
  } catch (const boost::system::system_error & e) {
    closeSerial();
    RCLCPP_WARN(
      this->get_logger(), "Failed to send kill switch command %d: %s", command, e.what());
  }
}

void BanyuTeleop::setLightState(bool on)
{
  if (!_lightEnabled) {
    return;
  }

  if (_lightStateInitialized && _targetLightOn == on) {
    return;
  }
  _targetLightOn = on;

  if (!_commandClient || !_commandClient->service_is_ready()) {
    return;
  }

  auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
  request->command = 183;  // MAV_CMD_DO_SET_SERVO
  request->param1 = static_cast<float>(_lightChannel);
  request->param2 = on ? static_cast<float>(_lightOnPwm) : static_cast<float>(_lightOffPwm);

  RCLCPP_INFO(this->get_logger(), "Commanding light (SERVO%d) %s (PWM: %.0f)...",
    _lightChannel, on ? "ON" : "OFF", request->param2);

  const uint32_t req_id = ++_lightRequestSeq;
  _commandClient->async_send_request(
    request,
    [this, on, req_id](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future) {
      try {
        auto response = future.get();
        if (response->success) {
          if (req_id == _lightRequestSeq) {
            _lightIsOn = on;
            _lightStateInitialized = true;
          }
          RCLCPP_INFO(this->get_logger(), "Light successfully turned %s", on ? "ON" : "OFF");
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to set light state (result: %u)",
            response->result);
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Command service call for light threw an exception: %s",
          e.what());
      }
    });
}

}   // namespace banyu_teleop
