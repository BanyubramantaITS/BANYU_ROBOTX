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

}  // namespace

Esp32TriggerNode::Esp32TriggerNode(const rclcpp::NodeOptions & options)
: Node("esp32_trigger_node", options)
{
  port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
  baud_ = declare_parameter<int>("baud", 115200);
  timer_period_s_ = declare_parameter<double>("timer_period_s", 0.1);
  append_newline_ = declare_parameter<bool>("append_newline", false);

  default_trigger_cmd_ = declare_parameter<std::string>("default_trigger_cmd", "1");
  trigger_on_cmd_ = declare_parameter<std::string>("trigger_on_cmd", "1");
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

  std::string formatted_cmd = cmd;
  if (append_newline_ && formatted_cmd.back() != '\n') {
    formatted_cmd.push_back('\n');
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

void Esp32TriggerNode::timerCallback()
{
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
      RCLCPP_INFO(get_logger(), "[MONITOR] %s", trimmed.c_str());
      if (monitor_publisher_) {
        std_msgs::msg::String msg;
        msg.data = trimmed;
        monitor_publisher_->publish(msg);
      }
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
