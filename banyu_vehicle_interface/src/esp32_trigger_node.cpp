#include "banyu_vehicle_interface/esp32_trigger_node.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>

namespace banyu_vehicle_interface
{

namespace
{

speed_t baudToSpeed(int baud)
{
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return B115200;
  }
}

std::string trim(const std::string & str)
{
  const size_t first = str.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const size_t last = str.find_last_not_of(" \t\r\n");
  return str.substr(first, last - first + 1);
}

// ESP32 indicator firmware (indikator_banyu.ino) numbers its states differently from
// operational_mode: board its0 = manual, its1 = auto, its2 = emergency.
std::string indicatorCommand(uint8_t operational_mode)
{
  switch (operational_mode) {
    case 1:
      return "its0\n";
    case 2:
      return "its1\n";
    default:
      return "its2\n";
  }
}

int parseIndicatorReport(const std::string & line)
{
  const std::string trimmed = trim(line);
  if (trimmed == "its0") {
    return 1;
  }
  if (trimmed == "its1") {
    return 2;
  }
  if (trimmed == "its2") {
    return 0;
  }
  return -1;
}

bool parseModeCommand(const std::string & cmd, uint8_t & mode)
{
  const std::string trimmed = trim(cmd);
  if (trimmed.size() != 1 || trimmed[0] < '0' || trimmed[0] > '2') {
    return false;
  }
  mode = static_cast<uint8_t>(trimmed[0] - '0');
  return true;
}

}  // namespace

Esp32TriggerNode::Esp32TriggerNode(const rclcpp::NodeOptions & options)
: Node("esp32_trigger_node", options)
{
  port_ = declare_parameter<std::string>("port", "/dev/esp32_kill");
  baud_ = declare_parameter<int>("baud", 115200);
  timer_period_s_ = declare_parameter<double>("timer_period_s", 0.1);
  append_newline_ = declare_parameter<bool>("append_newline", false);

  // Mode commands = VehicleStatus.operational_mode (issue #15): '0' kill, '1' manual,
  // '2' autonomous; they go to the board as its2 / its0 / its1 lines. Other strings are sent raw.
  default_trigger_cmd_ = declare_parameter<std::string>("default_trigger_cmd", "2");
  trigger_on_cmd_ = declare_parameter<std::string>("trigger_on_cmd", "2");
  trigger_off_cmd_ = declare_parameter<std::string>("trigger_off_cmd", "0");

  trigger_service_name_ = declare_parameter<std::string>(
    "trigger_service", "/banyu/trigger/fire");
  set_trigger_service_name_ = declare_parameter<std::string>(
    "set_trigger_service", "/banyu/trigger/set");
  command_service_name_ = declare_parameter<std::string>(
    "command_service", "/banyu/trigger/command");
  command_topic_name_ = declare_parameter<std::string>(
    "command_topic", "/banyu/trigger/cmd");
  monitor_topic_name_ = declare_parameter<std::string>(
    "monitor_topic", "/banyu/trigger/monitor");
  kill_switch_topic_name_ = declare_parameter<std::string>(
    "kill_switch_topic", "/banyu/hardware/kill_switch");
  // Without teleop nothing else reports the relay state, so a '0' would cut thruster power
  // while vehicle status still said AUTONOMOUS and the Pixhawk stayed armed.
  publish_kill_switch_ = declare_parameter<bool>("publish_kill_switch", true);
  // The board reports its state every 500 ms; EMERGENCY while '1'/'2' is commanded means its
  // hardware kill input is active.
  use_hardware_kill_ = declare_parameter<bool>("use_hardware_kill", true);
  kill_report_grace_s_ = declare_parameter<double>("kill_report_grace_s", 0.3);
  kill_report_count_ = std::max<int>(1, declare_parameter<int>("kill_report_count", 1));
  kill_release_to_manual_ = declare_parameter<bool>("kill_release_to_manual", false);
  resend_period_s_ = declare_parameter<double>("resend_period_s", 1.0);
  fcu_mode_sync_ = declare_parameter<bool>("fcu_mode_sync", true);
  fcu_manual_mode_ = declare_parameter<std::string>("fcu_manual_mode", "MANUAL");
  fcu_sync_retry_s_ = std::max(0.1, declare_parameter<double>("fcu_sync_retry_s", 1.0));
  fcu_sync_timeout_s_ = declare_parameter<double>("fcu_sync_timeout_s", 10.0);
  const auto fcu_state_topic = declare_parameter<std::string>("fcu_state_topic", "/mavros/state");
  const auto set_mode_service = declare_parameter<std::string>("set_mode_service", "/mavros/set_mode");
  const auto arming_service = declare_parameter<std::string>("arming_service", "/mavros/cmd/arming");

  trigger_service_ = create_service<std_srvs::srv::Trigger>(
    trigger_service_name_,
    std::bind(
      &Esp32TriggerNode::handleTrigger, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_trigger_service_ = create_service<std_srvs::srv::SetBool>(
    set_trigger_service_name_,
    std::bind(
      &Esp32TriggerNode::handleSetTrigger, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  command_service_ = create_service<banyu_msgs::srv::TriggerCommand>(
    command_service_name_,
    std::bind(
      &Esp32TriggerNode::handleSendCommand, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  command_subscriber_ = create_subscription<std_msgs::msg::String>(
    command_topic_name_, rclcpp::QoS(10),
    std::bind(&Esp32TriggerNode::handleCommandTopic, this, std::placeholders::_1));

  monitor_publisher_ = create_publisher<std_msgs::msg::String>(
    monitor_topic_name_, rclcpp::QoS(10));
  if (publish_kill_switch_) {
    kill_switch_publisher_ = create_publisher<std_msgs::msg::UInt8>(
      kill_switch_topic_name_, rclcpp::QoS(10));
  }

  if (fcu_mode_sync_) {
    fcu_state_subscriber_ = create_subscription<mavros_msgs::msg::State>(
      fcu_state_topic, rclcpp::QoS(10),
      [this](const mavros_msgs::msg::State::SharedPtr msg) {
        fcu_state_received_ = true;
        fcu_armed_ = msg->armed;
        fcu_mode_ = msg->mode;
      });
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>(set_mode_service);
    arming_client_ = create_client<mavros_msgs::srv::CommandBool>(arming_service);
  }

  if (openSerialPort()) {
    RCLCPP_INFO(get_logger(), "Berhasil konek ke ESP32 di %s", port_.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(), "Gagal buka port %s: %s. Akan mencoba berkala.", port_.c_str(),
        strerror(errno));
  }

  const auto period = std::chrono::duration<double>(std::max(0.01, timer_period_s_));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&Esp32TriggerNode::timerCallback, this));
}

Esp32TriggerNode::~Esp32TriggerNode()
{
  closeSerialPort();
}

bool Esp32TriggerNode::isPortOpen() const
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  return serial_fd_ >= 0;
}

bool Esp32TriggerNode::openSerialPort()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  closeSerialPortLocked();

  serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    closeSerialPortLocked();
    return false;
  }

  const speed_t speed = baudToSpeed(baud_);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 8N1 raw mode
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= (CLOCAL | CREAD);

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  tcflush(serial_fd_, TCIFLUSH);
  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    closeSerialPortLocked();
    return false;
  }

  return true;
}

void Esp32TriggerNode::closeSerialPortLocked()
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

void Esp32TriggerNode::closeSerialPort()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  closeSerialPortLocked();
}

bool Esp32TriggerNode::sendTrigger(const std::string & cmd)
{
  if (cmd.empty()) {
    RCLCPP_WARN(get_logger(), "Perintah trigger tidak boleh kosong.");
    return false;
  }

  // Record before writing: a kill must reach the software stack even if the serial write fails.
  recordCommandedMode(cmd);

  uint8_t mode = 0;
  const bool is_mode = parseModeCommand(cmd, mode);
  std::string formatted_cmd = is_mode ? indicatorCommand(mode) : cmd;
  if (!is_mode && append_newline_ && formatted_cmd.back() != '\n') {
    formatted_cmd.push_back('\n');
  }
  if (is_mode) {
    last_mode_write_ = steady_clock_.now();
  }

  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "Port serial belum terbuka.");
    return false;
  }

  size_t total_written = 0;
  while (total_written < formatted_cmd.length()) {
    const ssize_t written = write(
      serial_fd_, formatted_cmd.c_str() + total_written,
      formatted_cmd.length() - total_written);
    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      const int err = errno;
      RCLCPP_ERROR(get_logger(), "Gagal mengirim data ke serial: %s", strerror(err));
      if (err == EIO || err == EBADF || err == ENXIO || err == ENODEV || err == EPIPE) {
        RCLCPP_WARN(get_logger(), "Serial disconnected during write. Closing port.");
        closeSerialPortLocked();
      }
      return false;
    }
    total_written += static_cast<size_t>(written);
  }
  return true;
}

void Esp32TriggerNode::handleTrigger(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!isPortOpen()) {
    response->success = false;
    response->message = "Serial port not open";
    RCLCPP_WARN(get_logger(), "Trigger service called but serial port is not open");
    return;
  }

  const bool ok = sendTrigger(default_trigger_cmd_);
  response->success = ok;
  response->message = ok ? "Trigger command sent successfully" : "Failed to write to serial port";
}

void Esp32TriggerNode::handleSetTrigger(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!isPortOpen()) {
    response->success = false;
    response->message = "Serial port not open";
    RCLCPP_WARN(get_logger(), "SetTrigger service called but serial port is not open");
    return;
  }

  const std::string & cmd = request->data ? trigger_on_cmd_ : trigger_off_cmd_;
  const bool ok = sendTrigger(cmd);
  response->success = ok;
  response->message = ok ? ("Set trigger sent: " + cmd) : "Failed to write to serial port";
}

void Esp32TriggerNode::handleSendCommand(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<banyu_msgs::srv::TriggerCommand::Request> request,
  std::shared_ptr<banyu_msgs::srv::TriggerCommand::Response> response)
{
  if (!isPortOpen()) {
    response->success = false;
    response->message = "Serial port not open";
    RCLCPP_WARN(get_logger(), "SendCommand service called but serial port is not open");
    return;
  }

  if (request->command.empty()) {
    response->success = false;
    response->message = "Command string cannot be empty";
    return;
  }

  const bool ok = sendTrigger(request->command);
  response->success = ok;
  response->message = ok ? ("Command sent: " + request->command) : "Failed to write to serial port";
}

void Esp32TriggerNode::handleCommandTopic(const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg || msg->data.empty()) {
    return;
  }
  sendTrigger(msg->data);
}

void Esp32TriggerNode::recordCommandedMode(const std::string & cmd)
{
  uint8_t mode = 0;
  if (!parseModeCommand(cmd, mode)) {
    return;
  }
  if (!has_commanded_mode_ || mode != commanded_mode_) {
    commanded_at_ = steady_clock_.now();
  }
  commanded_mode_ = mode;
  has_commanded_mode_ = true;
  if (hw_kill_latched_ && mode == 0) {
    hw_kill_latched_ = false;
    RCLCPP_WARN(get_logger(), "Hardware kill acknowledged by a '0' command; latch cleared.");
  }
  publishCommandedMode();
}

void Esp32TriggerNode::handleBoardReport(const std::string & line)
{
  const int reported = parseIndicatorReport(line);
  if (reported < 0) {
    return;
  }
  if (hw_kill_latched_ && kill_release_to_manual_) {
    // Kill input released: the board reports a normal state again for kill_report_count reports in a row.
    if (reported == 0) {
      release_reports_seen_ = 0;
    } else if (++release_reports_seen_ >= kill_report_count_) {
      release_reports_seen_ = 0;
      hw_kill_latched_ = false;
      commanded_mode_ = 1;
      commanded_at_ = steady_clock_.now();
      hw_kill_pending_write_ = true;
      RCLCPP_WARN(
        get_logger(), "ESP32 kill switch released: MANUAL. Send '2' for AUTO (the interrupted run needs a new "
        "prepare and start).");
      publishCommandedMode();
    }
    return;
  }
  if (reported != 0 || !use_hardware_kill_ || hw_kill_latched_ || !has_commanded_mode_ ||
    commanded_mode_ == 0)
  {
    if (reported != 0 && kill_reports_seen_ > 0) {
      RCLCPP_WARN(
        get_logger(), "ESP32 kill input glitch: %d emergency report(s), then normal again (not latched)",
        kill_reports_seen_);
    }
    kill_reports_seen_ = 0;
    return;
  }
  // A report emitted before the board applied the last command is not a hardware kill.
  if ((steady_clock_.now() - commanded_at_).seconds() < kill_report_grace_s_) {
    return;
  }
  if (++kill_reports_seen_ < kill_report_count_) {
    return;
  }
  kill_reports_seen_ = 0;
  release_reports_seen_ = 0;
  hw_kill_latched_ = true;
  // With kill_release_to_manual the board shows the kill itself; writing its2 would keep it red after the release.
  hw_kill_pending_write_ = !kill_release_to_manual_;
  RCLCPP_ERROR(
    get_logger(), "ESP32 reports a hardware kill (emergency input active): publishing kill. %s",
    kill_release_to_manual_ ? "Put the kill switch back in to return to MANUAL." :
    "Send '0' to acknowledge before commanding a mode again.");
  publishCommandedMode();
}

void Esp32TriggerNode::publishCommandedMode()
{
  if (!kill_switch_publisher_ || !has_commanded_mode_) {
    return;
  }
  std_msgs::msg::UInt8 msg;
  msg.data = hw_kill_latched_ ? 0 : commanded_mode_;
  kill_switch_publisher_->publish(msg);
}

void Esp32TriggerNode::syncFcu()
{
  if (!fcu_mode_sync_ || !has_commanded_mode_) {
    return;
  }
  const uint8_t mode = hw_kill_latched_ ? 0 : commanded_mode_;
  const auto now = steady_clock_.now();
  if (mode != fcu_sync_mode_) {
    fcu_sync_mode_ = mode;
    fcu_sync_pending_ = mode != 2;
    fcu_sync_started_ = now;
    last_set_mode_request_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
    last_arming_request_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
  }
  if (!fcu_sync_pending_) {
    return;
  }

  const bool want_armed = mode == 1;
  const char * target = want_armed ? "armed" : "disarmed";
  const bool mode_ok = fcu_state_received_ && fcu_mode_ == fcu_manual_mode_;
  const bool armed_ok = fcu_state_received_ && fcu_armed_ == want_armed;
  if (mode_ok && armed_ok) {
    fcu_sync_pending_ = false;
    RCLCPP_INFO(get_logger(), "Pixhawk %s and %s", fcu_manual_mode_.c_str(), target);
    return;
  }
  if ((now - fcu_sync_started_).seconds() > fcu_sync_timeout_s_) {
    fcu_sync_pending_ = false;
    RCLCPP_ERROR(
      get_logger(), "Pixhawk did not reach %s %s within %.1f s (mode '%s', %s); set it from QGC",
      fcu_manual_mode_.c_str(), target, fcu_sync_timeout_s_, fcu_mode_.c_str(),
      fcu_armed_ ? "armed" : "disarmed");
    return;
  }

  auto due = [&](const rclcpp::Time & last) {
      return last.nanoseconds() == 0 || (now - last).seconds() >= fcu_sync_retry_s_;
    };
  if (!mode_ok && due(last_set_mode_request_) && set_mode_client_->service_is_ready()) {
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = fcu_manual_mode_;
    set_mode_client_->async_send_request(request);
    last_set_mode_request_ = now;
    RCLCPP_INFO(get_logger(), "Requesting Pixhawk mode %s", fcu_manual_mode_.c_str());
  }
  // Disarm at once on a kill; arm for MANUAL only once the mode has changed (the mission flies GUIDED/ALT_HOLD).
  if (!armed_ok && (!want_armed || mode_ok) && due(last_arming_request_) &&
    arming_client_->service_is_ready())
  {
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = want_armed;
    arming_client_->async_send_request(request);
    last_arming_request_ = now;
    RCLCPP_INFO(get_logger(), "Requesting Pixhawk %s", want_armed ? "arm" : "disarm");
  }
}

void Esp32TriggerNode::timerCallback()
{
  // Keep the commanded mode fresh for vehicle status (kill_switch_timeout_s).
  publishCommandedMode();
  syncFcu();
  if (!isPortOpen()) {
    reconnect_counter_++;
    const int retry_interval = static_cast<int>(1.0 / std::max(0.01, timer_period_s_));
    if (reconnect_counter_ >= retry_interval) {
      reconnect_counter_ = 0;
      if (openSerialPort()) {
        RCLCPP_INFO(get_logger(), "Berhasil konek kembali ke ESP32 di %s", port_.c_str());
      }
    }
    return;
  }
  processIncomingData();

  // Show the kill on the board, and resend the mode because the board forgets it on reboot.
  const bool resend_due = has_commanded_mode_ && !(hw_kill_latched_ && kill_release_to_manual_) &&
    (steady_clock_.now() - last_mode_write_).seconds() >= resend_period_s_;
  if (hw_kill_pending_write_ || resend_due) {
    hw_kill_pending_write_ = false;
    const uint8_t mode = hw_kill_latched_ ? 0 : commanded_mode_;
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_fd_ >= 0) {
      const std::string line = indicatorCommand(mode);
      if (write(serial_fd_, line.c_str(), line.size()) < 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Failed to resend mode to ESP32: %s", strerror(errno));
      }
      last_mode_write_ = steady_clock_.now();
    }
  }
}

void Esp32TriggerNode::processIncomingData()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (serial_fd_ < 0) {
    return;
  }

  int bytes_avail = 0;
  if (ioctl(serial_fd_, FIONREAD, &bytes_avail) < 0) {
    const int err = errno;
    if (err == EIO || err == EBADF || err == ENOTTY || err == ENODEV) {
      RCLCPP_WARN(
        get_logger(), "Serial port error during ioctl: %s. Closing port.",
        strerror(err));
      closeSerialPortLocked();
    }
    return;
  }
  if (bytes_avail <= 0) {
    return;
  }

  char buf[256];
  while (bytes_avail > 0) {
    const size_t chunk = std::min(sizeof(buf), static_cast<size_t>(bytes_avail));
    const ssize_t bytes_read = read(serial_fd_, buf, chunk);
    if (bytes_read < 0) {
      const int err = errno;
      if (err != EAGAIN && err != EWOULDBLOCK && err != EINTR) {
        RCLCPP_WARN(
          get_logger(), "Serial read error: %s. Closing port.",
          strerror(err));
        closeSerialPortLocked();
      }
      break;
    }
    if (bytes_read == 0) {
      break;
    }
    bytes_avail -= bytes_read;
    rx_buffer_.append(buf, bytes_read);
  }

  // Parse lines terminated by \n or \r
  size_t delim_pos = 0;
  while ((delim_pos = rx_buffer_.find_first_of("\r\n")) != std::string::npos) {
    const std::string line = rx_buffer_.substr(0, delim_pos);
    const std::string trimmed = trim(line);
    if (!trimmed.empty()) {
      const bool board_report = parseIndicatorReport(trimmed) >= 0;
      if (!board_report) {
        RCLCPP_INFO(get_logger(), "[MONITOR] %s", trimmed.c_str());
      }
      if (monitor_publisher_) {
        std_msgs::msg::String msg;
        msg.data = trimmed;
        monitor_publisher_->publish(msg);
      }
      handleBoardReport(trimmed);
    }

    const size_t next_valid = rx_buffer_.find_first_not_of("\r\n", delim_pos);
    if (next_valid != std::string::npos) {
      rx_buffer_.erase(0, next_valid);
    } else {
      rx_buffer_.clear();
      break;
    }
  }

  if (rx_buffer_.size() > 1024) {
    rx_buffer_.clear();
  }
}

}  // namespace banyu_vehicle_interface
