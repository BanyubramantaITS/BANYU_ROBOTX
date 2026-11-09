#include <gtest/gtest.h>

#include <cmath>

#include <rx_msgs/msg/vehicle_type.hpp>

#include "banyu_communication/task4_zones.hpp"

using banyu_communication::Task4ZoneTracker;
using rx_msgs::msg::Command;
using rx_msgs::msg::VehicleType;

namespace
{
Command zone(uint32_t seq, uint8_t target, double lat, double lon, float radius)
{
  Command c;
  c.command_seq = seq;
  c.target_vehicle = target;
  c.command_type = Command::CMD_TASK4_KEEP_OUT_ZONE;
  c.center.latitude = lat;
  c.center.longitude = lon;
  c.radius_m = radius;
  return c;
}

Command allClear(uint32_t seq, uint8_t target)
{
  Command c;
  c.command_seq = seq;
  c.target_vehicle = target;
  c.command_type = Command::CMD_TASK4_ALL_CLEAR;
  return c;
}

Command moving(uint32_t seq, uint8_t target, float heading_deg, float speed, float radius)
{
  Command c;
  c.command_seq = seq;
  c.target_vehicle = target;
  c.command_type = Command::CMD_TASK4_MOVING_OBJECT;
  c.position.latitude = -33.0;
  c.position.longitude = 151.0;
  c.heading_deg = heading_deg;
  c.speed_mps = speed;
  c.radius_m = radius;
  return c;
}

banyu_msgs::msg::Task4Zones snapshot(const Task4ZoneTracker & tracker)
{
  banyu_msgs::msg::Task4Zones msg;
  tracker.fill(msg);
  return msg;
}
}  // namespace

TEST(Task4ZoneTracker, StaticZonesAccumulateUntilAllClear)
{
  Task4ZoneTracker tracker({});
  auto r = tracker.handle(zone(1, VehicleType::TYPE_UUV, -33.0, 151.0, 20.0F), 0.0);
  EXPECT_TRUE(r.relevant);
  EXPECT_TRUE(r.changed);
  EXPECT_TRUE(r.ack);
  tracker.handle(zone(2, VehicleType::TYPE_UNKNOWN, -33.001, 151.001, 10.0F), 1.0);
  ASSERT_EQ(tracker.staticCount(), 2u);
  const auto msg = snapshot(tracker);
  EXPECT_EQ(msg.static_radius_m.size(), 2u);
  EXPECT_DOUBLE_EQ(msg.static_radius_m[0], 20.0);
  EXPECT_EQ(msg.last_event, "keep_out_zone");

  r = tracker.handle(allClear(3, VehicleType::TYPE_UUV), 2.0);
  EXPECT_TRUE(r.changed);
  EXPECT_TRUE(r.ack);
  EXPECT_EQ(tracker.staticCount(), 0u);
  EXPECT_EQ(snapshot(tracker).last_event, "all_clear");
}

TEST(Task4ZoneTracker, SameZoneResentUpdatesItsRadius)
{
  Task4ZoneTracker tracker({});
  tracker.handle(zone(1, VehicleType::TYPE_UUV, -33.0, 151.0, 20.0F), 0.0);
  const auto r = tracker.handle(zone(2, VehicleType::TYPE_UUV, -33.0, 151.0, 35.0F), 1.0);
  EXPECT_TRUE(r.changed);
  ASSERT_EQ(tracker.staticCount(), 1u);
  EXPECT_DOUBLE_EQ(snapshot(tracker).static_radius_m[0], 35.0);
}

TEST(Task4ZoneTracker, DuplicateSeqAndTypeIsIgnoredButAnotherTypeIsNot)
{
  Task4ZoneTracker tracker({});
  tracker.handle(zone(5, VehicleType::TYPE_USV, -33.0, 151.0, 20.0F), 0.0);
  // The same RxCommand split per addressed vehicle arrives again for the UUV: ignored as a zone, still acked.
  auto dup = tracker.handle(zone(5, VehicleType::TYPE_UUV, -33.5, 151.5, 99.0F), 1.0);
  EXPECT_TRUE(dup.duplicate);
  EXPECT_FALSE(dup.changed);
  EXPECT_EQ(tracker.staticCount(), 1u);
  EXPECT_DOUBLE_EQ(snapshot(tracker).static_radius_m[0], 20.0);
  // Same seq, different type: a different command.
  const auto clear = tracker.handle(allClear(5, VehicleType::TYPE_UUV), 2.0);
  EXPECT_FALSE(clear.duplicate);
  EXPECT_EQ(tracker.staticCount(), 0u);
}

TEST(Task4ZoneTracker, UsvTargetedZonesAreAvoidedButNotAcked)
{
  Task4ZoneTracker tracker({});
  const auto usv = tracker.handle(zone(1, VehicleType::TYPE_USV, -33.0, 151.0, 20.0F), 0.0);
  EXPECT_TRUE(usv.relevant);
  EXPECT_TRUE(usv.changed);
  EXPECT_FALSE(usv.ack);
  EXPECT_EQ(tracker.staticCount(), 1u);
  const auto usv_clear = tracker.handle(allClear(2, VehicleType::TYPE_USV), 1.0);
  EXPECT_FALSE(usv_clear.ack);
  EXPECT_EQ(tracker.staticCount(), 0u);
  // The UAV's zones are none of ours.
  const auto uav = tracker.handle(zone(3, VehicleType::TYPE_UAV, -33.0, 151.0, 20.0F), 2.0);
  EXPECT_FALSE(uav.relevant);
  EXPECT_FALSE(uav.ack);
  EXPECT_EQ(tracker.staticCount(), 0u);
}

TEST(Task4ZoneTracker, InvalidZonesAreRejectedButStillAcked)
{
  Task4ZoneTracker tracker({});
  EXPECT_TRUE(tracker.handle(zone(1, VehicleType::TYPE_UUV, -33.0, 151.0, 0.0F), 0.0).rejected);
  EXPECT_TRUE(tracker.handle(zone(2, VehicleType::TYPE_UUV, 123.0, 151.0, 10.0F), 0.0).rejected);
  EXPECT_TRUE(tracker.handle(zone(3, VehicleType::TYPE_UUV, std::nan(""), 151.0, 10.0F), 0.0).rejected);
  EXPECT_TRUE(tracker.handle(zone(4, VehicleType::TYPE_UUV, -33.0, 151.0, 5000.0F), 0.0).rejected);
  EXPECT_EQ(tracker.staticCount(), 0u);
  EXPECT_TRUE(tracker.handle(zone(5, VehicleType::TYPE_UUV, 0.0, 0.0, 0.0F), 0.0).ack);
}

TEST(Task4ZoneTracker, RunResetForgetsZonesAndSequenceNumbers)
{
  Task4ZoneTracker tracker({});
  tracker.handle(zone(1, VehicleType::TYPE_UUV, -33.0, 151.0, 20.0F), 0.0);
  tracker.handle(moving(2, VehicleType::TYPE_UUV, 90.0F, 2.0F, 5.0F), 0.0);
  ASSERT_FALSE(tracker.empty());
  tracker.reset();
  EXPECT_TRUE(tracker.empty());
  EXPECT_EQ(snapshot(tracker).last_event, "reset");
  // RoboCommand restarts its numbering at 1 for the next run: seq 1 is accepted again.
  const auto r = tracker.handle(zone(1, VehicleType::TYPE_UUV, -33.0, 151.0, 20.0F), 10.0);
  EXPECT_FALSE(r.duplicate);
  EXPECT_EQ(tracker.staticCount(), 1u);
}

TEST(Task4ZoneTracker, MovingObjectKeepsTheLatestAlertWithAnEnuHeading)
{
  Task4ZoneTracker tracker({});
  // Compass 90 deg = east = ENU 0 rad; compass 0 = north = ENU pi/2.
  auto r = tracker.handle(moving(1, VehicleType::TYPE_UUV, 90.0F, 2.0F, 8.0F), 5.0);
  EXPECT_TRUE(r.changed);
  EXPECT_FALSE(r.ack);  // the protocol has no ack for it: off by default
  auto msg = snapshot(tracker);
  EXPECT_TRUE(msg.moving_active);
  EXPECT_NEAR(msg.moving_heading_enu_rad, 0.0, 1e-9);
  EXPECT_DOUBLE_EQ(msg.moving_radius_m, 8.0);
  EXPECT_DOUBLE_EQ(tracker.movingReceivedS(), 5.0);

  tracker.handle(moving(3, VehicleType::TYPE_USV, 0.0F, 1.0F, 6.0F), 6.0);
  msg = snapshot(tracker);
  EXPECT_NEAR(msg.moving_heading_enu_rad, M_PI / 2.0, 1e-9);
  EXPECT_DOUBLE_EQ(msg.moving_speed_mps, 1.0);
  // A late, older alert does not replace it.
  const auto old = tracker.handle(moving(2, VehicleType::TYPE_UUV, 180.0F, 9.0F, 6.0F), 7.0);
  EXPECT_TRUE(old.duplicate);
  EXPECT_DOUBLE_EQ(snapshot(tracker).moving_speed_mps, 1.0);
  // ALL_CLEAR is for the static zones; the moving alert stays until it expires.
  tracker.handle(allClear(4, VehicleType::TYPE_UUV), 8.0);
  EXPECT_TRUE(tracker.movingActive());
  EXPECT_FALSE(tracker.expire(100.0));
  EXPECT_TRUE(tracker.expire(6.0 + 121.0));
  EXPECT_FALSE(tracker.movingActive());
}

TEST(Task4ZoneTracker, MovingObjectAckIsOptIn)
{
  Task4ZoneTracker::Config config;
  config.ack_moving_object = true;
  Task4ZoneTracker tracker(config);
  EXPECT_TRUE(tracker.handle(moving(1, VehicleType::TYPE_UUV, 10.0F, 1.0F, 5.0F), 0.0).ack);
  EXPECT_FALSE(tracker.handle(moving(2, VehicleType::TYPE_USV, 10.0F, 1.0F, 5.0F), 0.0).ack);
  EXPECT_TRUE(tracker.handle(moving(3, VehicleType::TYPE_UNKNOWN, 10.0F, 1.0F, 5.0F), 0.0).ack);
}

TEST(Task4ZoneTracker, RejectsInvalidMovingObjects)
{
  Task4ZoneTracker tracker({});
  EXPECT_TRUE(tracker.handle(moving(1, VehicleType::TYPE_UUV, 10.0F, -1.0F, 5.0F), 0.0).rejected);
  EXPECT_TRUE(tracker.handle(moving(2, VehicleType::TYPE_UUV, 10.0F, 1.0F, 0.0F), 0.0).rejected);
  EXPECT_TRUE(tracker.handle(moving(3, VehicleType::TYPE_UUV, std::nanf(""), 1.0F, 5.0F), 0.0).rejected);
  EXPECT_FALSE(tracker.movingActive());
}
