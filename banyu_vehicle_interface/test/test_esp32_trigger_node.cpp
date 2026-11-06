#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include "banyu_vehicle_interface/esp32_trigger_node.hpp"

class Esp32TriggerNodeTest : public ::testing::Test
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
    // Set master non-blocking for tests
    const int flags = fcntl(master_fd_, F_GETFL, 0);
    fcntl(master_fd_, F_SETFL, flags | O_NONBLOCK);
    return true;
  }

  int master_fd_{-1};
  int slave_fd_{-1};
};

TEST_F(Esp32TriggerNodeTest, OpenPtyAndInitialize)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"baud", 115200}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  EXPECT_TRUE(node->isPortOpen());
}

TEST_F(Esp32TriggerNodeTest, SendTriggerWritesRawBytes)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"baud", 115200}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  ASSERT_TRUE(node->isPortOpen());

  // Test sending '0'
  EXPECT_TRUE(node->sendTrigger("0"));
  char buf[16] = {0};
  ssize_t n = read(master_fd_, buf, sizeof(buf) - 1);
  EXPECT_EQ(std::string(buf, n > 0 ? n : 0), "its2\n");

  // Test sending '1'
  EXPECT_TRUE(node->sendTrigger("1"));
  memset(buf, 0, sizeof(buf));
  n = read(master_fd_, buf, sizeof(buf) - 1);
  EXPECT_EQ(std::string(buf, n > 0 ? n : 0), "its0\n");

  // Test sending '2'
  EXPECT_TRUE(node->sendTrigger("2"));
  memset(buf, 0, sizeof(buf));
  n = read(master_fd_, buf, sizeof(buf) - 1);
  EXPECT_EQ(std::string(buf, n > 0 ? n : 0), "its1\n");
}

TEST_F(Esp32TriggerNodeTest, ProcessIncomingDataParsesLines)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"baud", 115200}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  ASSERT_TRUE(node->isPortOpen());

  // Send data to master_fd so node reads from slave
  const std::string msg = "ESP32 READY\r\nSTATUS: RUNNING\n";
  const ssize_t written = write(master_fd_, msg.c_str(), msg.length());
  EXPECT_EQ(written, static_cast<ssize_t>(msg.length()));

  // Process data in node
  node->processIncomingData();

  // Test partial message streaming across calls
  const std::string part1 = "TEMP: 2";
  const std::string part2 = "5.5C\r\n";
  write(master_fd_, part1.c_str(), part1.length());
  node->processIncomingData();
  write(master_fd_, part2.c_str(), part2.length());
  node->processIncomingData();
}

TEST_F(Esp32TriggerNodeTest, FailsToOpenInvalidPort)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", "/dev/nonexistent_serial_device_test_123"},
    {"baud", 115200}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  EXPECT_FALSE(node->isPortOpen());
  EXPECT_FALSE(node->sendTrigger("0"));
}

TEST_F(Esp32TriggerNodeTest, ProcessEmptyAndWhitespaceLines)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"baud", 115200}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  ASSERT_TRUE(node->isPortOpen());

  // Empty lines and spaces should be safely processed without crashing
  const std::string blanks = "\n   \r\n\t\t\n   \n";
  write(master_fd_, blanks.c_str(), blanks.length());
  node->processIncomingData();
}

TEST_F(Esp32TriggerNodeTest, MultiLineAndVariousNewlines)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"baud", 115200}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  ASSERT_TRUE(node->isPortOpen());

  // Mix of \r, \n, and \r\n
  const std::string mixed = "LINE_CR\rLINE_LF\nLINE_CRLF\r\nFINAL_LINE\n";
  write(master_fd_, mixed.c_str(), mixed.length());
  node->processIncomingData();
}

TEST_F(Esp32TriggerNodeTest, RapidSuccessionTriggers)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"baud", 115200}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  ASSERT_TRUE(node->isPortOpen());

  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(node->sendTrigger("0"));
    EXPECT_TRUE(node->sendTrigger("1"));
    EXPECT_TRUE(node->sendTrigger("2"));
  }

  // Mode commands go out as 5-byte "itsN\n" lines.
  char buf[256] = {0};
  ssize_t total_read = 0;
  while (total_read < 150) {
    ssize_t n = read(master_fd_, buf + total_read, sizeof(buf) - total_read);
    if (n > 0) {
      total_read += n;
    } else {
      break;
    }
  }
  EXPECT_EQ(total_read, 150);
}

TEST_F(Esp32TriggerNodeTest, TriggerServiceExecution)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"baud", 115200},
    {"default_trigger_cmd", "1"}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  ASSERT_TRUE(node->isPortOpen());

  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto res = std::make_shared<std_srvs::srv::Trigger::Response>();
  node->handleTrigger(nullptr, req, res);

  EXPECT_TRUE(res->success);
  EXPECT_NE(res->message.find("successfully"), std::string::npos);

  char buf[16] = {0};
  ssize_t n = read(master_fd_, buf, sizeof(buf) - 1);
  EXPECT_EQ(std::string(buf, n > 0 ? n : 0), "its0\n");
}

TEST_F(Esp32TriggerNodeTest, SetTriggerServiceExecution)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"baud", 115200},
    {"trigger_on_cmd", "1"},
    {"trigger_off_cmd", "0"}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  ASSERT_TRUE(node->isPortOpen());

  // Test SetTrigger(true) -> "1"
  auto req_on = std::make_shared<std_srvs::srv::SetBool::Request>();
  req_on->data = true;
  auto res_on = std::make_shared<std_srvs::srv::SetBool::Response>();
  node->handleSetTrigger(nullptr, req_on, res_on);

  EXPECT_TRUE(res_on->success);
  char buf[16] = {0};
  ssize_t n = read(master_fd_, buf, sizeof(buf) - 1);
  EXPECT_EQ(std::string(buf, n > 0 ? n : 0), "its0\n");

  // Test SetTrigger(false) -> "0"
  auto req_off = std::make_shared<std_srvs::srv::SetBool::Request>();
  req_off->data = false;
  auto res_off = std::make_shared<std_srvs::srv::SetBool::Response>();
  node->handleSetTrigger(nullptr, req_off, res_off);

  EXPECT_TRUE(res_off->success);
  memset(buf, 0, sizeof(buf));
  n = read(master_fd_, buf, sizeof(buf) - 1);
  EXPECT_EQ(std::string(buf, n > 0 ? n : 0), "its2\n");
}

TEST_F(Esp32TriggerNodeTest, CommandServiceExecution)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"baud", 115200}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  ASSERT_TRUE(node->isPortOpen());

  // Test valid command "2"
  auto req = std::make_shared<banyu_msgs::srv::TriggerCommand::Request>();
  req->command = "2";
  auto res = std::make_shared<banyu_msgs::srv::TriggerCommand::Response>();
  node->handleSendCommand(nullptr, req, res);

  EXPECT_TRUE(res->success);
  char buf[16] = {0};
  ssize_t n = read(master_fd_, buf, sizeof(buf) - 1);
  EXPECT_EQ(std::string(buf, n > 0 ? n : 0), "its1\n");

  // Test empty command rejection
  req->command = "";
  auto res_empty = std::make_shared<banyu_msgs::srv::TriggerCommand::Response>();
  node->handleSendCommand(nullptr, req, res_empty);
  EXPECT_FALSE(res_empty->success);
  EXPECT_NE(res_empty->message.find("cannot be empty"), std::string::npos);
}

TEST_F(Esp32TriggerNodeTest, ServiceRejectionsOnClosedPort)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", "/dev/nonexistent_device_trigger_closed_port_test"}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  EXPECT_FALSE(node->isPortOpen());

  // Trigger service rejection
  auto trig_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto trig_res = std::make_shared<std_srvs::srv::Trigger::Response>();
  node->handleTrigger(nullptr, trig_req, trig_res);
  EXPECT_FALSE(trig_res->success);

  // SetBool service rejection
  auto set_req = std::make_shared<std_srvs::srv::SetBool::Request>();
  set_req->data = true;
  auto set_res = std::make_shared<std_srvs::srv::SetBool::Response>();
  node->handleSetTrigger(nullptr, set_req, set_res);
  EXPECT_FALSE(set_res->success);

  // Command service rejection
  auto cmd_req = std::make_shared<banyu_msgs::srv::TriggerCommand::Request>();
  cmd_req->command = "1";
  auto cmd_res = std::make_shared<banyu_msgs::srv::TriggerCommand::Response>();
  node->handleSendCommand(nullptr, cmd_req, cmd_res);
  EXPECT_FALSE(cmd_res->success);
}

TEST_F(Esp32TriggerNodeTest, CommandTopicExecution)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"baud", 115200}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  ASSERT_TRUE(node->isPortOpen());

  auto msg = std::make_shared<std_msgs::msg::String>();
  msg->data = "0";
  node->handleCommandTopic(msg);

  char buf[16] = {0};
  ssize_t n = read(master_fd_, buf, sizeof(buf) - 1);
  EXPECT_EQ(std::string(buf, n > 0 ? n : 0), "its2\n");

  // Null and empty message should be safely ignored
  node->handleCommandTopic(nullptr);
  msg->data = "";
  node->handleCommandTopic(msg);
}

TEST_F(Esp32TriggerNodeTest, MonitorTopicPublishing)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"baud", 115200},
    {"monitor_topic", "/test/monitor"}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  ASSERT_TRUE(node->isPortOpen());

  std::string received_msg;
  auto sub = node->create_subscription<std_msgs::msg::String>(
    "/test/monitor", 10,
    [&received_msg](const std_msgs::msg::String::SharedPtr msg) {
      received_msg = msg->data;
    });

  const std::string text = "STATUS: TRIGGERED\r\n";
  write(master_fd_, text.c_str(), text.length());

  const auto start = std::chrono::steady_clock::now();
  while (received_msg.empty() &&
    (std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(500))
  {
    node->processIncomingData();
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(received_msg, "STATUS: TRIGGERED");
}

TEST_F(Esp32TriggerNodeTest, EmptyCommandRejectedBySendTrigger)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"baud", 115200}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  ASSERT_TRUE(node->isPortOpen());
  EXPECT_FALSE(node->sendTrigger(""));
}

TEST_F(Esp32TriggerNodeTest, AppendNewlineOption)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"baud", 115200},
    {"append_newline", true}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  ASSERT_TRUE(node->isPortOpen());

  // Raw (non-mode) commands get the newline; mode commands are already "itsN\n" lines.
  EXPECT_TRUE(node->sendTrigger("H"));

  char buf[16] = {0};
  ssize_t n = read(master_fd_, buf, sizeof(buf) - 1);
  EXPECT_EQ(n, 2);
  EXPECT_EQ(buf[0], 'H');
  EXPECT_EQ(buf[1], '\n');
}

TEST_F(Esp32TriggerNodeTest, DisconnectDetectionOnWriteFailure)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"baud", 115200}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  ASSERT_TRUE(node->isPortOpen());

  // Close master fd so that writing from slave fails
  close(master_fd_);
  master_fd_ = -1;

  // On pty, writing to slave when master is closed causes EIO
  const bool ok = node->sendTrigger("1");
  EXPECT_FALSE(ok);
  // Port should now be marked closed due to disconnect detection
  EXPECT_FALSE(node->isPortOpen());
}

TEST_F(Esp32TriggerNodeTest, ModeCommandsPublishKillSwitchEvenWhenWriteFails)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", "/dev/nonexistent_banyu_port"},
    {"kill_switch_topic", "/test/kill_switch"}
  });

  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  ASSERT_FALSE(node->isPortOpen());

  std::vector<uint8_t> received;
  auto sub = node->create_subscription<std_msgs::msg::UInt8>(
    "/test/kill_switch", 10,
    [&received](const std_msgs::msg::UInt8::SharedPtr msg) {received.push_back(msg->data);});

  // The timer republishes the commanded mode, so drain the queue and check the latest.
  auto spin_for = [&](int ms) {
      const auto start = std::chrono::steady_clock::now();
      while ((std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(ms)) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    };
  spin_for(200);   // let the subscription match before publishing
  EXPECT_TRUE(received.empty());   // nothing commanded yet: nothing published

  EXPECT_FALSE(node->sendTrigger("0"));   // serial fails, the kill still reaches ROS
  spin_for(300);
  ASSERT_FALSE(received.empty());
  EXPECT_EQ(received.back(), 0);

  node->sendTrigger("H");                 // not a mode byte: no change
  spin_for(300);
  EXPECT_EQ(received.back(), 0);

  node->sendTrigger("2");
  spin_for(300);
  EXPECT_EQ(received.back(), 2);
}

TEST_F(Esp32TriggerNodeTest, BoardEmergencyReportLatchesKill)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"kill_switch_topic", "/test/hw_kill_switch"},
    {"kill_report_grace_s", 0.05}
  });
  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  ASSERT_TRUE(node->isPortOpen());

  std::vector<uint8_t> received;
  auto sub = node->create_subscription<std_msgs::msg::UInt8>(
    "/test/hw_kill_switch", 10,
    [&received](const std_msgs::msg::UInt8::SharedPtr msg) {received.push_back(msg->data);});
  auto spin_for = [&](int ms) {
      const auto start = std::chrono::steady_clock::now();
      while ((std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(ms)) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    };
  auto board_says = [&](const char * report) {
      ASSERT_GT(write(master_fd_, report, strlen(report)), 0);
      spin_for(250);
    };

  EXPECT_TRUE(node->sendTrigger("2"));
  spin_for(200);
  board_says("its1\r\n");
  EXPECT_FALSE(node->isHardwareKillLatched());
  ASSERT_FALSE(received.empty());
  EXPECT_EQ(received.back(), 2);

  board_says("its2\r\n");
  EXPECT_TRUE(node->isHardwareKillLatched());
  EXPECT_EQ(received.back(), 0);

  // Commanding auto again does not clear the latch; '0' acknowledges it.
  node->sendTrigger("2");
  spin_for(200);
  EXPECT_EQ(received.back(), 0);
  node->sendTrigger("0");
  EXPECT_FALSE(node->isHardwareKillLatched());
}

TEST_F(Esp32TriggerNodeTest, ShortEmergencyGlitchDoesNotLatchKill)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"kill_switch_topic", "/test/hw_kill_glitch"},
    {"kill_report_grace_s", 0.05},
    {"kill_report_count", 3}
  });
  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  ASSERT_TRUE(node->isPortOpen());
  auto spin_for = [&](int ms) {
      const auto start = std::chrono::steady_clock::now();
      while ((std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(ms)) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    };
  auto board_says = [&](const char * report) {
      ASSERT_GT(write(master_fd_, report, strlen(report)), 0);
      spin_for(250);
    };

  EXPECT_TRUE(node->sendTrigger("2"));
  spin_for(200);
  // Two emergency reports, then normal: a glitch, not a kill.
  board_says("its2\r\n");
  board_says("its2\r\n");
  board_says("its1\r\n");
  EXPECT_FALSE(node->isHardwareKillLatched());

  // Three in a row: a held kill input.
  board_says("its2\r\n");
  board_says("its2\r\n");
  EXPECT_FALSE(node->isHardwareKillLatched());
  board_says("its2\r\n");
  EXPECT_TRUE(node->isHardwareKillLatched());
}

TEST_F(Esp32TriggerNodeTest, KillSwitchReleaseReturnsToManual)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"kill_switch_topic", "/test/hw_kill_release"},
    {"kill_report_grace_s", 0.05},
    {"kill_report_count", 2},
    {"kill_release_to_manual", true}
  });
  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);
  ASSERT_TRUE(node->isPortOpen());
  std::vector<uint8_t> received;
  auto sub = node->create_subscription<std_msgs::msg::UInt8>(
    "/test/hw_kill_release", 10,
    [&received](const std_msgs::msg::UInt8::SharedPtr msg) {received.push_back(msg->data);});
  auto spin_for = [&](int ms) {
      const auto start = std::chrono::steady_clock::now();
      while ((std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(ms)) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    };
  auto board_says = [&](const char * report) {
      ASSERT_GT(write(master_fd_, report, strlen(report)), 0);
      spin_for(250);
    };
  auto drain = [&]() {
      char buf[256];
      std::string out;
      ssize_t n;
      while ((n = read(master_fd_, buf, sizeof(buf))) > 0) {out.append(buf, n);}
      return out;
    };

  EXPECT_TRUE(node->sendTrigger("2"));
  spin_for(200);
  board_says("its2\r\n");
  board_says("its2\r\n");
  ASSERT_TRUE(node->isHardwareKillLatched());
  EXPECT_EQ(received.back(), 0);
  drain();
  spin_for(1300);
  // The board shows the kill itself: nothing is written over it while the switch is out.
  EXPECT_EQ(drain().find("its"), std::string::npos);

  // Switch back in: the board reports its last state again -> MANUAL, its0 written.
  board_says("its1\r\n");
  board_says("its1\r\n");
  EXPECT_FALSE(node->isHardwareKillLatched());
  EXPECT_EQ(received.back(), 1);
  spin_for(200);
  EXPECT_NE(drain().find("its0"), std::string::npos);
}

TEST_F(Esp32TriggerNodeTest, ManualAndKillSwitchPixhawkToManual)
{
  std::string slave_path;
  ASSERT_TRUE(createPtyPair(slave_path));

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"port", slave_path},
    {"kill_switch_topic", "/test/fcu_sync/kill_switch"},
    {"fcu_state_topic", "/test/fcu_sync/state"},
    {"set_mode_service", "/test/fcu_sync/set_mode"},
    {"arming_service", "/test/fcu_sync/arming"},
    {"fcu_sync_retry_s", 0.2}
  });
  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>(options);

  // Fake Pixhawk: flying the mission in GUIDED, armed.
  auto fcu = std::make_shared<rclcpp::Node>("fake_fcu");
  mavros_msgs::msg::State state;
  state.connected = true;
  state.armed = true;
  state.mode = "GUIDED";
  int set_mode_calls = 0;
  int arming_calls = 0;
  auto state_pub = fcu->create_publisher<mavros_msgs::msg::State>("/test/fcu_sync/state", 10);
  auto set_mode_srv = fcu->create_service<mavros_msgs::srv::SetMode>(
    "/test/fcu_sync/set_mode",
    [&](const std::shared_ptr<mavros_msgs::srv::SetMode::Request> req,
    std::shared_ptr<mavros_msgs::srv::SetMode::Response> res) {
      ++set_mode_calls;
      state.mode = req->custom_mode;
      res->mode_sent = true;
    });
  auto arming_srv = fcu->create_service<mavros_msgs::srv::CommandBool>(
    "/test/fcu_sync/arming",
    [&](const std::shared_ptr<mavros_msgs::srv::CommandBool::Request> req,
    std::shared_ptr<mavros_msgs::srv::CommandBool::Response> res) {
      ++arming_calls;
      state.armed = req->value;
      res->success = true;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(fcu);
  auto spin_for = [&](int ms) {
      const auto start = std::chrono::steady_clock::now();
      while ((std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(ms)) {
        state_pub->publish(state);
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    };

  // AUTO: the mission manager owns the FCU mode.
  EXPECT_TRUE(node->sendTrigger("2"));
  spin_for(600);
  EXPECT_EQ(state.mode, "GUIDED");
  EXPECT_EQ(set_mode_calls, 0);
  EXPECT_EQ(arming_calls, 0);

  // MANUAL: Pixhawk MANUAL, stays armed.
  EXPECT_TRUE(node->sendTrigger("1"));
  spin_for(800);
  EXPECT_EQ(state.mode, "MANUAL");
  EXPECT_TRUE(state.armed);
  EXPECT_EQ(arming_calls, 0);

  // KILL: MANUAL and disarmed.
  state.mode = "ALT_HOLD";
  EXPECT_TRUE(node->sendTrigger("0"));
  spin_for(800);
  EXPECT_EQ(state.mode, "MANUAL");
  EXPECT_FALSE(state.armed);

  // MANUAL again after a kill: armed in MANUAL.
  EXPECT_TRUE(node->sendTrigger("1"));
  spin_for(800);
  EXPECT_EQ(state.mode, "MANUAL");
  EXPECT_TRUE(state.armed);
  const int calls = set_mode_calls + arming_calls;
  spin_for(600);
  EXPECT_EQ(set_mode_calls + arming_calls, calls);  // confirmed: no more requests
}
