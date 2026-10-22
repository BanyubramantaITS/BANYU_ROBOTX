#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "banyu_teleop/banyu_teleop.hpp"

#include <chrono>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

class BanyuTeleopTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    test_node_ = std::make_shared<rclcpp::Node>("test_banyu_teleop_fixture");
  }

  void TearDown() override
  {
    test_node_.reset();
  }

  std::shared_ptr<rclcpp::Node> test_node_;
};

TEST_F(BanyuTeleopTest, DefaultParametersAndInitialState)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"serial_port", "none"},
    {"enable_mock_auto", true},
  });

  auto teleop = std::make_shared<banyu_teleop::BanyuTeleop>(options);
  EXPECT_FALSE(teleop->isMockAutoActive());
  EXPECT_EQ(teleop->getOperationalMode(), 1);
  EXPECT_FALSE(teleop->isVehicleArmed());
}

TEST_F(BanyuTeleopTest, ActivatesMockAutoModeOnRCAutoSwitch)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"serial_port", "none"},
    {"enable_mock_auto", true},
    {"arm_channel", 6},
    {"kill_channel", 5},
    {"mode_channel", 7},
    {"switch_low_threshold", 1200.0},
    {"switch_high_threshold", 1750.0},
    {"mock_auto_forward_pwm", 1540},
    {"mock_auto_throttle_pwm", 1540},
  });

  auto teleop = std::make_shared<banyu_teleop::BanyuTeleop>(options);

  mavros_msgs::msg::OverrideRCIn last_override;
  bool override_received = false;
  auto override_sub = test_node_->create_subscription<mavros_msgs::msg::OverrideRCIn>(
    "/mavros/rc/override", 10,
    [&](const mavros_msgs::msg::OverrideRCIn::SharedPtr msg) {
      last_override = *msg;
      override_received = true;
    });

  std_msgs::msg::UInt8 last_kill;
  bool kill_received = false;
  auto kill_sub = test_node_->create_subscription<std_msgs::msg::UInt8>(
    "/banyu/hardware/kill_switch", 10,
    [&](const std_msgs::msg::UInt8::SharedPtr msg) {
      last_kill = *msg;
      kill_received = true;
    });

  // Send RC input commanding autonomous mode (CH6 / kill_channel index 5 < 1200) and armed (CH7 / arm_channel index 6 >= 1500)
  auto rc_msg = std::make_shared<mavros_msgs::msg::RCIn>();
  rc_msg->channels = {1500, 1500, 1500, 1500, 1500, 1000, 2000, 1500};
  // CH6 (index 5) = 1000 (< 1200 -> Auto mode!)
  // CH7 (index 6) = 2000 (>= 1500 -> armed)

  teleop->rcinCallback(rc_msg);

  EXPECT_TRUE(teleop->isMockAutoActive());
  EXPECT_EQ(teleop->getOperationalMode(), 2);

  // Spin to receive published messages
  auto start = std::chrono::steady_clock::now();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(test_node_);
  exec.add_node(teleop);
  while ((!override_received || !kill_received) &&
    std::chrono::steady_clock::now() - start < 1s)
  {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_TRUE(kill_received);
  EXPECT_EQ(last_kill.data, 2);

  EXPECT_TRUE(override_received);
  // Throttle is index 2 (CH3)
  EXPECT_EQ(last_override.channels[2], 1540);
  // Switch channels 5, 6, 7 must remain CHAN_RELEASE (0)
  EXPECT_EQ(last_override.channels[5], mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
  EXPECT_EQ(last_override.channels[6], mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
  EXPECT_EQ(last_override.channels[7], mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
}

TEST_F(BanyuTeleopTest, RejectsManualSticksWhileInAutoMode)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"serial_port", "none"},
    {"enable_mock_auto", true},
    {"arm_channel", 6},
    {"kill_channel", 5},
    {"mode_channel", 7},
    {"mock_auto_forward_pwm", 1540},
    {"mock_auto_throttle_pwm", 1540},
  });

  auto teleop = std::make_shared<banyu_teleop::BanyuTeleop>(options);

  mavros_msgs::msg::OverrideRCIn last_override;
  auto override_sub = test_node_->create_subscription<mavros_msgs::msg::OverrideRCIn>(
    "/mavros/rc/override", 10,
    [&](const mavros_msgs::msg::OverrideRCIn::SharedPtr msg) {
      last_override = *msg;
    });

  // 1. Enter auto mode
  auto rc_msg = std::make_shared<mavros_msgs::msg::RCIn>();
  rc_msg->channels = {1500, 1500, 1500, 1500, 1500, 988, 2000, 1500};
  teleop->rcinCallback(rc_msg);
  EXPECT_TRUE(teleop->isMockAutoActive());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(test_node_);
  exec.add_node(teleop);
  for (int i = 0; i < 5; ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  // 2. Now simulate pilot deflecting sticks to max while still in auto mode
  auto manual_deflected = std::make_shared<mavros_msgs::msg::RCIn>();
  manual_deflected->channels = {1900, 1900, 1900, 1900, 1900, 988, 2000, 1500};
  teleop->rcinCallback(manual_deflected);

  for (int i = 0; i < 5; ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  // Verify that the override outputs did NOT follow the 1900 stick deflection,
  // but remained at the safe, slow mock values
  EXPECT_TRUE(teleop->isMockAutoActive());
  EXPECT_EQ(last_override.channels[2], 1540); // Throttle remains at slow value
  EXPECT_EQ(last_override.channels[4], 1540); // Forward remains at slow value
  EXPECT_EQ(last_override.channels[0], 1500); // Pitch remains locked neutral
  EXPECT_EQ(last_override.channels[1], 1500); // Roll remains locked neutral
  EXPECT_EQ(last_override.channels[3], 1500); // Yaw remains locked neutral
}

TEST_F(BanyuTeleopTest, CancelsMockAutoOnSwitchBackToManual)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"serial_port", "none"},
    {"enable_mock_auto", true},
    {"arm_channel", 6},
    {"kill_channel", 5},
    {"mode_channel", 7},
  });

  auto teleop = std::make_shared<banyu_teleop::BanyuTeleop>(options);

  mavros_msgs::msg::OverrideRCIn last_override;
  auto override_sub = test_node_->create_subscription<mavros_msgs::msg::OverrideRCIn>(
    "/mavros/rc/override", 10,
    [&](const mavros_msgs::msg::OverrideRCIn::SharedPtr msg) {
      last_override = *msg;
    });

  // 1. Enter auto mode
  auto rc_msg = std::make_shared<mavros_msgs::msg::RCIn>();
  rc_msg->channels = {1500, 1500, 1500, 1500, 1500, 988, 2000, 1500};
  teleop->rcinCallback(rc_msg);
  EXPECT_TRUE(teleop->isMockAutoActive());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(test_node_);
  exec.add_node(teleop);
  for (int i = 0; i < 5; ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  // 2. Switch back to manual (mid position 1500)
  auto manual_msg = std::make_shared<mavros_msgs::msg::RCIn>();
  manual_msg->channels = {1500, 1500, 1500, 1500, 1500, 1500, 2000, 1500};
  teleop->rcinCallback(manual_msg);

  EXPECT_FALSE(teleop->isMockAutoActive());
  EXPECT_EQ(teleop->getOperationalMode(), 1);

  for (int i = 0; i < 5; ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  // Verify that all overrides are released (set to 0 / CHAN_RELEASE)
  for (size_t i = 0; i < last_override.channels.size(); ++i) {
    EXPECT_EQ(last_override.channels[i], mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
  }
}

TEST_F(BanyuTeleopTest, CancelsMockAutoOnKillSwitch)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"serial_port", "none"},
    {"enable_mock_auto", true},
    {"arm_channel", 6},
    {"kill_channel", 5},
    {"mode_channel", 7},
  });

  auto teleop = std::make_shared<banyu_teleop::BanyuTeleop>(options);

  mavros_msgs::msg::OverrideRCIn last_override;
  auto override_sub = test_node_->create_subscription<mavros_msgs::msg::OverrideRCIn>(
    "/mavros/rc/override", 10,
    [&](const mavros_msgs::msg::OverrideRCIn::SharedPtr msg) {
      last_override = *msg;
    });

  // 1. Enter auto mode
  auto rc_msg = std::make_shared<mavros_msgs::msg::RCIn>();
  rc_msg->channels = {1500, 1500, 1500, 1500, 1500, 988, 2000, 1500};
  teleop->rcinCallback(rc_msg);
  EXPECT_TRUE(teleop->isMockAutoActive());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(test_node_);
  exec.add_node(teleop);
  for (int i = 0; i < 5; ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  // 2. Flip kill switch (> 1750, e.g. 2011)
  auto kill_msg = std::make_shared<mavros_msgs::msg::RCIn>();
  kill_msg->channels = {1500, 1500, 1500, 1500, 1500, 2011, 2000, 1500};
  teleop->rcinCallback(kill_msg);

  EXPECT_FALSE(teleop->isMockAutoActive());
  EXPECT_EQ(teleop->getOperationalMode(), 0);

  for (int i = 0; i < 5; ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  // Verify that overrides are released
  for (size_t i = 0; i < last_override.channels.size(); ++i) {
    EXPECT_EQ(last_override.channels[i], mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
  }
}

TEST_F(BanyuTeleopTest, CancelsMockAutoOnArmSwitchDisarm)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"serial_port", "none"},
    {"enable_mock_auto", true},
    {"arm_channel", 6},
    {"arm_threshold", 1500},
    {"kill_channel", 5},
    {"mode_channel", 7},
  });

  auto teleop = std::make_shared<banyu_teleop::BanyuTeleop>(options);

  // 1. Enter auto mode with vehicle armed
  auto rc_msg = std::make_shared<mavros_msgs::msg::RCIn>();
  rc_msg->channels = {1500, 1500, 1500, 1500, 1500, 988, 2000, 1500};
  teleop->rcinCallback(rc_msg);
  EXPECT_TRUE(teleop->isMockAutoActive());

  // 2. Disarm using arm switch (channel 6 drops below threshold)
  auto disarm_msg = std::make_shared<mavros_msgs::msg::RCIn>();
  disarm_msg->channels = {1500, 1500, 1500, 1500, 1500, 988, 1000, 1500};
  teleop->rcinCallback(disarm_msg);

  EXPECT_FALSE(teleop->isMockAutoActive());
}
