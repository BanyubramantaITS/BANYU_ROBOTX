#include <gtest/gtest.h>

#include "banyu_vehicle_interface/vehicle_status_tracker.hpp"

namespace banyu_vehicle_interface
{
namespace
{

rclcpp::Time timeAt(double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
}

VehicleStatusTracker::Config testConfig()
{
  VehicleStatusTracker::Config config;
  config.fcu_timeout_s = 1.0;
  config.rc_timeout_s = 1.0;
  config.gps_timeout_s = 1.0;
  config.rc_takeover_channel = 0;
  config.rc_takeover_threshold = 1700;
  config.autonomy_mode = "GUIDED";
  return config;
}

mavros_msgs::msg::State state(bool connected, const std::string & mode)
{
  mavros_msgs::msg::State message;
  message.connected = connected;
  message.mode = mode;
  return message;
}

mavros_msgs::msg::RCIn rc(uint16_t value)
{
  mavros_msgs::msg::RCIn message;
  message.channels = {value};
  return message;
}

}  // namespace

TEST(VehicleStatusTrackerTest, StartsFailSafeAndFcuBecomesStale)
{
  VehicleStatusTracker tracker(testConfig());
  auto initial = tracker.status(timeAt(0.0));
  EXPECT_FALSE(initial.fcu_connected);
  EXPECT_FALSE(initial.fcu_status_fresh);
  EXPECT_EQ(initial.authority, "UNKNOWN");

  tracker.updateState(state(true, "GUIDED"), timeAt(0.0));
  auto current = tracker.status(timeAt(0.5));
  EXPECT_TRUE(current.fcu_connected);
  EXPECT_TRUE(current.fcu_status_fresh);

  auto stale = tracker.status(timeAt(1.1));
  EXPECT_FALSE(stale.fcu_connected);
  EXPECT_FALSE(stale.fcu_status_fresh);
}

TEST(VehicleStatusTrackerTest, RcFreshnessExpiresAndDisarmsAuthority)
{
  VehicleStatusTracker tracker(testConfig());
  tracker.updateState(state(true, "GUIDED"), timeAt(0.0));
  tracker.updateRc(rc(1500), timeAt(0.0));

  EXPECT_TRUE(tracker.status(timeAt(0.5)).rc_fresh);
  EXPECT_FALSE(tracker.status(timeAt(1.1)).rc_fresh);
  EXPECT_EQ(tracker.status(timeAt(1.1)).authority, "UNKNOWN");
}

TEST(VehicleStatusTrackerTest, MalformedRcDoesNotRefreshDeadman)
{
  VehicleStatusTracker tracker(testConfig());
  tracker.updateState(state(true, "GUIDED"), timeAt(0.0));
  tracker.updateRc(rc(1500), timeAt(0.0));
  ASSERT_TRUE(tracker.status(timeAt(0.1)).rc_fresh);

  mavros_msgs::msg::RCIn malformed;
  tracker.updateRc(malformed, timeAt(0.2));

  const auto rejected = tracker.status(timeAt(0.2));
  EXPECT_FALSE(rejected.rc_fresh);
  EXPECT_FALSE(rejected.operator_takeover);
  EXPECT_EQ(rejected.authority, "UNKNOWN");

  tracker.updateRc(rc(1500), timeAt(0.3));
  EXPECT_TRUE(tracker.status(timeAt(0.3)).rc_fresh);
}

TEST(VehicleStatusTrackerTest, HighRcTakeoverWinsOverAutonomy)
{
  VehicleStatusTracker tracker(testConfig());
  tracker.updateState(state(true, "GUIDED"), timeAt(0.0));
  tracker.updateRc(rc(1800), timeAt(0.0));

  const auto result = tracker.status(timeAt(0.1));
  EXPECT_TRUE(result.operator_takeover);
  EXPECT_EQ(result.authority, "OPERATOR");
}

TEST(VehicleStatusTrackerTest, PublishesCurrentMode)
{
  VehicleStatusTracker tracker(testConfig());
  tracker.updateState(state(true, "MANUAL"), timeAt(0.0));
  tracker.updateRc(rc(1500), timeAt(0.0));

  const auto result = tracker.status(timeAt(0.1));
  EXPECT_EQ(result.mode, "MANUAL");
  EXPECT_EQ(result.authority, "MANUAL");
}

TEST(VehicleStatusTrackerTest, InvalidGpsIsNotValid)
{
  VehicleStatusTracker tracker(testConfig());
  sensor_msgs::msg::NavSatFix gps;
  gps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  gps.latitude = 12.0;
  gps.longitude = 34.0;
  tracker.updateGps(gps, timeAt(0.0));

  const auto result = tracker.status(timeAt(0.1));
  EXPECT_FALSE(result.gps_valid);
  EXPECT_FALSE(result.gps_fresh);
}

}  // namespace banyu_vehicle_interface
