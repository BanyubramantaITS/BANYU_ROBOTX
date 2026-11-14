#pragma once

#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <boost/asio.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace banyu_teleop
{

class BanyuTeleop : public rclcpp::Node
{
public:
  explicit BanyuTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BanyuTeleop() override;

  bool isMockAutoActive() const {return _mockAutoActive;}
  uint8_t getOperationalMode() const {return _currentOperationalMode;}
  bool isVehicleArmed() const {return _isArmed;}
  const std::string & getCurrentMode() const {return _currentMode;}
  bool isGimbalDeflected() const {return _gimbalDeflectedLast;}
  bool isGimbalInputRejected() const {return _currentOperationalMode == 2 && _gimbalDeflectedLast;}
  bool isGimbalOverrideActive() const {return _gimbalOverrideActive;}
  bool isHardwareKillLatched() const {return _hwKillLatched;}
  bool isLightOn() const {return _lightIsOn;}
  bool isLightEnabled() const {return _lightEnabled;}

  // One state line from the ESP32 indicator ("itsN"); public so tests can feed reports.
  void handleIndicatorLine(const std::string & line);

  void rcinCallback(const mavros_msgs::msg::RCIn::SharedPtr msg);
  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);

private:
  void declareParameters();
  void warnOnSwitchChannelOverlap();
  void initCommunication();

  void handleModeSwitch(const mavros_msgs::msg::RCIn::SharedPtr & msg);
  void handleKillSwitch(const mavros_msgs::msg::RCIn::SharedPtr & msg);
  void handleGimbalInput(const mavros_msgs::msg::RCIn::SharedPtr & msg);
  void handlePrepareSwitch(const mavros_msgs::msg::RCIn::SharedPtr & msg);
  void requestPrepareAutonomy();
  bool isGimbalDeflected(const mavros_msgs::msg::RCIn::SharedPtr & msg) const;
  void publishNeutralGimbalOverride();
  void publishReleaseOverride();

  void armVehicle(bool arm);
  void setFlightMode(const std::string & mode);
  void setLightState(bool on);

  void openSerial(bool quiet = false);
  void closeSerial();
  void writeKillCommand(int command);
  void indicatorTimerCallback();
  void readIndicatorSerial();
  void latchHardwareKill();
  void handleHardwareKillRelease();

  void startMockAutoMode();
  void stopMockAutoMode();
  void mockAutoTimerCallback();
  void publishMockAutoOutputs();
  void publishZeroSetpoint();
  bool isSwitchChannel(int chan_1indexed) const;

  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr _rcinSubscriber;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr _stateSubscriber;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr _killSwitchPublisher;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr _killBoardPublisher;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr _rcOverridePub;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr _setpointPub;

  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr _armingClient;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr _commandClient;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr _setModeClient;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _prepareClient;
  rclcpp::TimerBase::SharedPtr _mockAutoTimer;
  rclcpp::TimerBase::SharedPtr _indicatorTimer;

  std::string _rcTopic;
  std::string _killSwitchTopic;
  std::string _killBoardTopic;
  std::string _overrideTopic;
  std::string _setpointTopic;
  std::string _stateTopic;
  std::string _commandService = "/mavros/cmd/command";

  bool _lightEnabled = true;
  int _lightChannel = 9;
  double _lightOnPwm = 1900.0;
  double _lightOffPwm = 1100.0;
  bool _lightIsOn = false;
  bool _lightStateInitialized = false;
  bool _targetLightOn = false;
  uint32_t _lightRequestSeq = 0;

  int _modeChannel = 7;
  int _killChannel = 5;
  double _switchLowThreshold = 1200.0;
  double _switchHighThreshold = 1750.0;
  double _gimbalDeadband = 50.0;

  // Momentary SH: a press (low -> high) with SB Up calls the mission's prepare_autonomy.
  int _prepareChannel = 4;
  std::string _prepareService = "/banyu/mission/prepare_autonomy";
  bool _prepareSwitchInitialized = false;
  bool _prepareSwitchPressed = false;
  bool _preparePending = false;
  rclcpp::Time _prepareRequestedAt{0, 0, RCL_STEADY_TIME};

  std::string _serialDevice = "/dev/esp32_kill";
  int _serialBaud = 115200;
  bool _useHardwareKill = true;
  double _killReportTimeoutS = 1.5;
  double _killReportGraceS = 0.3;
  double _indicatorResendPeriodS = 1.0;
  double _rcTimeoutS = 3.0;

  std::string _lastMode;
  bool _modeStateInitialized = false;

  int _lastKillCommand = -1;
  bool _killStateInitialized = false;
  bool _gimbalDeflectedLast = false;
  bool _gimbalOverrideActive = false;
  uint8_t _currentOperationalMode = 1;
  uint8_t _prevOperationalMode = 1;
  bool _isArmed = false;
  std::string _currentMode = "MANUAL";

  boost::asio::io_context _io;
  std::unique_ptr<boost::asio::serial_port> _serialPort;
  bool _serialConnected = false;
  std::string _rxBuffer;

  // Mode last sent to the ESP32 (-1 = none) and when it last changed.
  int _commandedMode = -1;
  rclcpp::Time _commandChangedAt{0, 0, RCL_STEADY_TIME};
  rclcpp::Time _lastCommandSentAt{0, 0, RCL_STEADY_TIME};
  rclcpp::Time _lastReportAt{0, 0, RCL_STEADY_TIME};
  rclcpp::Time _lastReconnectAt{0, 0, RCL_STEADY_TIME};
  rclcpp::Time _lastRcAt{0, 0, RCL_STEADY_TIME};
  bool _hwKillLatched = false;
  // Latched and waiting for the board to report the kill input released.
  bool _hwKillAwaitRelease = false;
  bool _reportStaleWarned = false;
  rclcpp::Clock _steadyClock{RCL_STEADY_TIME};

  bool _enableMockAuto = true;
  std::string _mockAutoFlightMode = "MANUAL";
  double _mockAutoPublishRateHz = 20.0;

  int _mockAutoForwardPwm = 1540;
  int _mockAutoThrottlePwm = 1540;
  int _mockAutoLateralPwm = 1500;
  int _mockAutoYawPwm = 1500;
  int _mockAutoPitchPwm = 1500;
  int _mockAutoRollPwm = 1500;

  double _mockAutoLinearX = 0.1;
  double _mockAutoLinearZ = 0.1;

  int _pitchChannel = 1;
  int _rollChannel = 2;
  int _throttleChannel = 3;
  int _yawChannel = 4;
  int _forwardChannel = 5;
  int _lateralChannel = 6;

  bool _mockAutoActive = false;
};

}   // namespace banyu_teleop
