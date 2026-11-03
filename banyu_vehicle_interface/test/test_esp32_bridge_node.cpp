#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "banyu_vehicle_interface/esp32_bridge_node.hpp"

class Esp32BridgeNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    if (master_fd_ >= 0) {
      close(master_fd_);
      master_fd_ = -1;
    }
    if (slave_fd_ >= 0) {
      close(slave_fd_);
      slave_fd_ = -1;
    }
  }

  bool createPtyPair(std::string & slave_path)
  {
    if (openpty(&master_fd_, &slave_fd_, nullptr, nullptr, nullptr) != 0) {
      return false;
    }
    char * name = ttyname(slave_fd_);
    if (!name) {
      return false;
    }
    slave_path = name;
    const int flags = fcntl(master_fd_, F_GETFL, 0);
    fcntl(master_fd_, F_SETFL, flags | O_NONBLOCK);
    return true;
  }

  int master_fd_{-1};
  int slave_fd_{-1};
};

TEST_F(Esp32BridgeNodeTest, ParsesValidKillSwitchLines)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", "/dev/nonexistent_dummy_port"},
    {"heartbeat_rate_hz", 1.0}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32BridgeNode>(options);
  std::vector<uint8_t> received;

  auto sub = node->create_subscription<std_msgs::msg::UInt8>(
    "/banyu/hardware/kill_switch", rclcpp::QoS(10),
    [&received](const std_msgs::msg::UInt8::SharedPtr msg) {
      received.push_back(msg->data);
    });

  // Test lines with various newlines
  const std::string chunk1 = "0\n1\r\n2\n";
  node->processIncomingBytes(chunk1.c_str(), chunk1.length());

  auto start = std::chrono::steady_clock::now();
  while (received.size() < 3 &&
    (std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(500))
  {
    rclcpp::spin_some(node);
  }

  ASSERT_EQ(received.size(), 3U);
  EXPECT_EQ(received[0], 0);
  EXPECT_EQ(received[1], 1);
  EXPECT_EQ(received[2], 2);
}

TEST_F(Esp32BridgeNodeTest, RejectsGarbageAndNonModeDigits)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", "/dev/nonexistent_dummy_port"}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32BridgeNode>(options);
  std::vector<uint8_t> received;

  auto sub = node->create_subscription<std_msgs::msg::UInt8>(
    "/banyu/hardware/kill_switch", rclcpp::QoS(10),
    [&received](const std_msgs::msg::UInt8::SharedPtr msg) {
      received.push_back(msg->data);
    });

  // Strings containing numbers 0, 1, 2 but not standalone mode commands
  const std::string garbage = "IP: 192.168.1.100\r\nBootloader 0.12\nTemp: 25.0 C\n";
  node->processIncomingBytes(garbage.c_str(), garbage.length());

  rclcpp::spin_some(node);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  rclcpp::spin_some(node);

  EXPECT_TRUE(received.empty());
}

TEST_F(Esp32BridgeNodeTest, ParsesRawSingleByteWithoutNewline)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", "/dev/nonexistent_dummy_port"}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32BridgeNode>(options);
  std::vector<uint8_t> received;

  auto sub = node->create_subscription<std_msgs::msg::UInt8>(
    "/banyu/hardware/kill_switch", rclcpp::QoS(10),
    [&received](const std_msgs::msg::UInt8::SharedPtr msg) {
      received.push_back(msg->data);
    });

  // Single raw byte
  node->processIncomingBytes("2", 1);
  rclcpp::spin_some(node);

  ASSERT_EQ(received.size(), 1U);
  EXPECT_EQ(received[0], 2);
}

TEST_F(Esp32BridgeNodeTest, PTYPortOpenAndShutdownCleanly)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"baud", 115200},
    {"heartbeat_rate_hz", 10.0}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32BridgeNode>(options);
  EXPECT_TRUE(node->isPortOpen());
}

TEST_F(Esp32BridgeNodeTest, ParsesIndicatorReports)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", "/dev/nonexistent_dummy_port"},
    {"heartbeat_rate_hz", 1.0}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32BridgeNode>(options);
  std::vector<uint8_t> received;
  auto sub = node->create_subscription<std_msgs::msg::UInt8>(
    "/banyu/hardware/kill_switch", rclcpp::QoS(10),
    [&received](const std_msgs::msg::UInt8::SharedPtr msg) {
      received.push_back(msg->data);
    });

  // Board its0/its1/its2 = manual/auto/emergency -> operational_mode 1/2/0.
  const std::string chunk = "its0\r\nits1\r\nits2\r\nits3\r\n";
  node->processIncomingBytes(chunk.c_str(), chunk.length());

  auto start = std::chrono::steady_clock::now();
  while (received.size() < 3 &&
    (std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(500))
  {
    rclcpp::spin_some(node);
  }
  ASSERT_EQ(received.size(), 3U);
  EXPECT_EQ(received[0], 1);
  EXPECT_EQ(received[1], 2);
  EXPECT_EQ(received[2], 0);
}
