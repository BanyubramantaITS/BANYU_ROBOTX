#include "banyu_vehicle_interface/esp32_bridge_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>

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

Esp32BridgeNode::Esp32BridgeNode(const rclcpp::NodeOptions & options)
: Node("esp32_bridge_node", options),
  serial_fd_(-1),
  is_running_(false)
{
  port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
  baud_ = declare_parameter<int>("baud", 115200);
  heartbeat_rate_hz_ = declare_parameter<double>("heartbeat_rate_hz", 10.0);
  serial_timeout_s_ = declare_parameter<double>("serial_timeout_s", 0.5);

  kill_switch_pub_ = create_publisher<std_msgs::msg::UInt8>(
    "/banyu/hardware/kill_switch", rclcpp::QoS(10));

  if (!openSerialPort()) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to open serial port '%s'. Reader thread will attempt to reconnect.",
      port_.c_str());
  }

  is_running_ = true;
  reader_thread_ = std::thread(&Esp32BridgeNode::readerLoop, this);

  const double safe_rate_hz = std::max(0.1, heartbeat_rate_hz_);
  const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / safe_rate_hz));
  heartbeat_timer_ = create_wall_timer(
    std::max(std::chrono::milliseconds(1), period),
    std::bind(&Esp32BridgeNode::heartbeatCallback, this));
}

Esp32BridgeNode::~Esp32BridgeNode()
{
  is_running_ = false;
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
  closeSerialPort();
}

bool Esp32BridgeNode::isPortOpen() const
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  return serial_fd_ >= 0;
}

bool Esp32BridgeNode::openSerialPort()
{
  closeSerialPort();

  std::lock_guard<std::mutex> lock(serial_mutex_);
  serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0) {
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcgetattr failed on '%s': %s", port_.c_str(), strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  const speed_t speed = baudToSpeed(baud_);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 8N1 configuration
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= (CLOCAL | CREAD);

  // Raw input and output
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;

  // Read timeout using VMIN & VTIME (in tenths of a second)
  const int timeout_decisec = static_cast<int>(std::round(serial_timeout_s_ * 10.0));
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = static_cast<cc_t>(std::clamp(timeout_decisec, 1, 255));

  tcflush(serial_fd_, TCIFLUSH);
  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcsetattr failed on '%s': %s", port_.c_str(), strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  RCLCPP_INFO(
    get_logger(),
    "Connected to ESP32 serial bridge on '%s' at %d baud (8N1).",
    port_.c_str(), baud_);
  return true;
}

void Esp32BridgeNode::closeSerialPort()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

void Esp32BridgeNode::heartbeatCallback()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (serial_fd_ >= 0) {
    const ssize_t written = write(serial_fd_, "H\n", 2);
    if (written < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to send heartbeat to '%s': %s",
        port_.c_str(), strerror(errno));
    }
  }
}

void Esp32BridgeNode::processIncomingBytes(const char * data, size_t length)
{
  rx_buffer_.append(data, length);

  size_t delim_pos = 0;
  while ((delim_pos = rx_buffer_.find_first_of("\r\n")) != std::string::npos) {
    const std::string line = rx_buffer_.substr(0, delim_pos);
    const size_t next = rx_buffer_.find_first_not_of("\r\n", delim_pos);
    if (next != std::string::npos) {
      rx_buffer_.erase(0, next);
    } else {
      rx_buffer_.clear();
    }

    const std::string trimmed = trim(line);
    if (trimmed == "0" || trimmed == "1" || trimmed == "2") {
      std_msgs::msg::UInt8 msg;
      msg.data = static_cast<uint8_t>(trimmed[0] - '0');
      kill_switch_pub_->publish(msg);
    }
  }

  if (rx_buffer_.size() == 1) {
    const char c = rx_buffer_[0];
    if (c == '0' || c == '1' || c == '2') {
      std_msgs::msg::UInt8 msg;
      msg.data = static_cast<uint8_t>(c - '0');
      kill_switch_pub_->publish(msg);
      rx_buffer_.clear();
    }
  }

  if (rx_buffer_.size() > 1024) {
    rx_buffer_.clear();
  }
}

void Esp32BridgeNode::readerLoop()
{
  char buf[64];
  while (is_running_ && rclcpp::ok()) {
    int current_fd = -1;
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      current_fd = serial_fd_;
    }

    if (current_fd < 0) {
      if (!openSerialPort()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        current_fd = serial_fd_;
      }
    }

    const ssize_t bytes_read = read(current_fd, buf, sizeof(buf));
    if (bytes_read > 0) {
      processIncomingBytes(buf, static_cast<size_t>(bytes_read));
    } else if (bytes_read < 0) {
      if (errno != EAGAIN && errno != EINTR) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Serial read error on '%s': %s. Closing port.",
          port_.c_str(), strerror(errno));
        closeSerialPort();
      }
    }
  }
}

}  // namespace banyu_vehicle_interface
