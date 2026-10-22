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
#include <string>
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

  // Send RC input commanding autonomous mode (CH6 / kill_channel index 5 < 1200)
  auto rc_msg = std::make_shared<mavros_msgs::msg::RCIn>();
  rc_msg->channels = {1500, 1500, 1500, 1500, 1500, 1000, 1500, 1500};
  // CH6 (index 5) = 1000 (< 1200 -> Auto mode!)

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
  // Switch channels 5 and 7 must remain CHAN_RELEASE (0)
  EXPECT_EQ(last_override.channels[5], mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
  EXPECT_EQ(last_override.channels[7], mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
}

TEST_F(BanyuTeleopTest, RejectsManualSticksWhileInAutoMode)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"serial_port", "none"},
    {"enable_mock_auto", true},
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
  rc_msg->channels = {1500, 1500, 1500, 1500, 1500, 988, 1500, 1500};
  teleop->rcinCallback(rc_msg);
  EXPECT_TRUE(teleop->isMockAutoActive());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(test_node_);
  exec.add_node(teleop);
  for (int i = 0; i < 5; ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  // 2. Simulate pilot deflecting sticks to max while still in auto mode
  auto manual_deflected = std::make_shared<mavros_msgs::msg::RCIn>();
  manual_deflected->channels = {1900, 1900, 1900, 1900, 1900, 988, 1500, 1500};
  teleop->rcinCallback(manual_deflected);

  for (int i = 0; i < 5; ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  // Verify that the override outputs remained at the safe, slow mock values
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
  rc_msg->channels = {1500, 1500, 1500, 1500, 1500, 988, 1500, 1500};
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
  manual_msg->channels = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
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
  rc_msg->channels = {1500, 1500, 1500, 1500, 1500, 988, 1500, 1500};
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
  kill_msg->channels = {1500, 1500, 1500, 1500, 1500, 2011, 1500, 1500};
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

TEST_F(BanyuTeleopTest, UnmappedSFDoesNotAffectArmOrAuto)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"serial_port", "none"},
    {"enable_mock_auto", true},
    {"kill_channel", 5},
    {"mode_channel", 7},
  });

  auto teleop = std::make_shared<banyu_teleop::BanyuTeleop>(options);

  // 1. Enter auto mode with SB Up (index 5 = 988)
  auto rc_msg = std::make_shared<mavros_msgs::msg::RCIn>();
  rc_msg->channels = {1500, 1500, 1500, 1500, 1500, 988, 2011, 1500};
  teleop->rcinCallback(rc_msg);
  EXPECT_TRUE(teleop->isMockAutoActive());

  // 2. Toggle SF (index 6) to 988 (disarm on old mapping)
  auto toggle_sf = std::make_shared<mavros_msgs::msg::RCIn>();
  toggle_sf->channels = {1500, 1500, 1500, 1500, 1500, 988, 988, 1500};
  teleop->rcinCallback(toggle_sf);

  // Verify that SF toggle has no effect on auto mode or operational mode
  EXPECT_TRUE(teleop->isMockAutoActive());
  EXPECT_EQ(teleop->getOperationalMode(), 2);
}

TEST_F(BanyuTeleopTest, SBDrivesArmingAndDisarming)
{
  // Set up mock arming service server
  std::vector<bool> arm_calls;
  auto arm_service = test_node_->create_service<mavros_msgs::srv::CommandBool>(
    "/mavros/cmd/arming",
    [&](const std::shared_ptr<mavros_msgs::srv::CommandBool::Request> request,
        std::shared_ptr<mavros_msgs::srv::CommandBool::Response> response) {
      arm_calls.push_back(request->value);
      response->success = true;
      response->result = 0;
    });

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"serial_port", "none"},
    {"enable_mock_auto", false},
    {"kill_channel", 5},
    {"mode_channel", 7},
  });

  auto teleop = std::make_shared<banyu_teleop::BanyuTeleop>(options);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(test_node_);
  exec.add_node(teleop);

  // 1. SB Middle (1500): Manual mode -> automatically arms Pixhawk
  auto rc_mid = std::make_shared<mavros_msgs::msg::RCIn>();
  rc_mid->channels = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
  teleop->rcinCallback(rc_mid);

  for (int i = 0; i < 10; ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_EQ(teleop->getOperationalMode(), 1);
  ASSERT_FALSE(arm_calls.empty());
  EXPECT_TRUE(arm_calls.back());

  // 2. SB Down (2011): Disarm both ESP32 kill switch and Pixhawk
  auto rc_down = std::make_shared<mavros_msgs::msg::RCIn>();
  rc_down->channels = {1500, 1500, 1500, 1500, 1500, 2011, 1500, 1500};
  teleop->rcinCallback(rc_down);

  for (int i = 0; i < 10; ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_EQ(teleop->getOperationalMode(), 0);
  ASSERT_GE(arm_calls.size(), 2u);
  EXPECT_FALSE(arm_calls.back());
  EXPECT_FALSE(teleop->isVehicleArmed());

  // 3. SB Up (988): Autonomous mode -> arms Pixhawk if not armed
  auto rc_up = std::make_shared<mavros_msgs::msg::RCIn>();
  rc_up->channels = {1500, 1500, 1500, 1500, 1500, 988, 1500, 1500};
  teleop->rcinCallback(rc_up);

  for (int i = 0; i < 10; ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_EQ(teleop->getOperationalMode(), 2);
  ASSERT_GE(arm_calls.size(), 3u);
  EXPECT_TRUE(arm_calls.back());
}

TEST_F(BanyuTeleopTest, SCModeChangeRejectedInAutonomousAndSyncedOnExit)
{
  std::vector<std::string> mode_requests;
  auto set_mode_service = test_node_->create_service<mavros_msgs::srv::SetMode>(
    "/mavros/set_mode",
    [&](const std::shared_ptr<mavros_msgs::srv::SetMode::Request> request,
        std::shared_ptr<mavros_msgs::srv::SetMode::Response> response) {
      mode_requests.push_back(request->custom_mode);
      response->mode_sent = true;
    });

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"serial_port", "none"},
    {"enable_mock_auto", false},
    {"kill_channel", 5},
    {"mode_channel", 7},
  });

  auto teleop = std::make_shared<banyu_teleop::BanyuTeleop>(options);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(test_node_);
  exec.add_node(teleop);

  // 1. Initial message in Manual mode (SB=1500, SC=1500 -> MANUAL)
  auto rc_init = std::make_shared<mavros_msgs::msg::RCIn>();
  rc_init->channels = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
  teleop->rcinCallback(rc_init);
  for (int i = 0; i < 5; ++i) { exec.spin_some(); std::this_thread::sleep_for(10ms); }
  EXPECT_TRUE(mode_requests.empty());

  // 2. In Manual mode, toggle SC to STABILIZE (CH8 / index 7 = 900) -> accepted
  auto rc_stab = std::make_shared<mavros_msgs::msg::RCIn>();
  rc_stab->channels = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 900};
  teleop->rcinCallback(rc_stab);
  for (int i = 0; i < 5; ++i) { exec.spin_some(); std::this_thread::sleep_for(10ms); }
  ASSERT_EQ(mode_requests.size(), 1u);
  EXPECT_EQ(mode_requests.back(), "STABILIZE");

  // 3. Switch SB to Autonomous (SB = 988, SC = 900 STABILIZE)
  auto rc_auto = std::make_shared<mavros_msgs::msg::RCIn>();
  rc_auto->channels = {1500, 1500, 1500, 1500, 1500, 988, 1500, 900};
  teleop->rcinCallback(rc_auto);
  for (int i = 0; i < 5; ++i) { exec.spin_some(); std::this_thread::sleep_for(10ms); }
  EXPECT_EQ(teleop->getOperationalMode(), 2);

  // 4. While in Autonomous mode, toggle SC to ALT_HOLD (2000) -> REJECTED
  auto rc_auto_alt = std::make_shared<mavros_msgs::msg::RCIn>();
  rc_auto_alt->channels = {1500, 1500, 1500, 1500, 1500, 988, 1500, 2000};
  teleop->rcinCallback(rc_auto_alt);
  for (int i = 0; i < 5; ++i) { exec.spin_some(); std::this_thread::sleep_for(10ms); }
  // Request count remains 1; no new request was sent in autonomous mode
  EXPECT_EQ(mode_requests.size(), 1u);

  // 5. Operator switches SB back to Manual (1500) -> immediately syncs SC mode (ALT_HOLD)
  auto rc_manual_restore = std::make_shared<mavros_msgs::msg::RCIn>();
  rc_manual_restore->channels = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 2000};
  teleop->rcinCallback(rc_manual_restore);
  for (int i = 0; i < 5; ++i) { exec.spin_some(); std::this_thread::sleep_for(10ms); }
  ASSERT_EQ(mode_requests.size(), 2u);
  EXPECT_EQ(mode_requests.back(), "ALT_HOLD");
}
