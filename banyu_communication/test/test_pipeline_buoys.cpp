#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "banyu_communication/pipeline_buoys.hpp"
#include "rx_msgs/msg/vehicle_type.hpp"

namespace
{
using banyu_communication::PipelineBuoysTracker;
using rx_msgs::msg::PipelineBuoys;
constexpr double kPi = 3.14159265358979323846;

rx_msgs::msg::LatLng point(double latitude, double longitude)
{
  rx_msgs::msg::LatLng value;
  value.latitude = latitude;
  value.longitude = longitude;
  return value;
}

rx_msgs::msg::Course course(const std::string & id = "ITSN-SIM-A")
{
  rx_msgs::msg::Course value;
  value.course_id = id;
  value.pinger_freq_hz = 25000;
  value.boundary = {point(33.8093, -118.3949), point(33.8093, -118.3929),
    point(33.8113, -118.3929), point(33.8113, -118.3949), point(33.8093, -118.3949)};
  return value;
}

// Buoy A west of buoy B, about 11 m apart on the same latitude (the sim course).
PipelineBuoys buoys(uint32_t seq, uint8_t active = PipelineBuoys::ACTIVE_A, bool both = true)
{
  PipelineBuoys value;
  value.course_id = "ITSN-SIM-A";
  value.source_vehicle = rx_msgs::msg::VehicleType::TYPE_USV;
  value.seq = seq;
  value.buoy_a = point(33.810438, -118.393392);
  value.buoy_b = both ? point(33.810438, -118.393272) : rx_msgs::msg::LatLng{};
  value.buoy_b_valid = both;
  value.active_buoy = active;
  value.uncertainty_m = 2.0F;
  return value;
}

builtin_interfaces::msg::Time stamp(int32_t sec)
{
  builtin_interfaces::msg::Time value;
  value.sec = sec;
  return value;
}

PipelineBuoysTracker tracker(PipelineBuoysTracker::Config config = {})
{
  PipelineBuoysTracker value(config);
  value.setCourse(course());
  return value;
}
}  // namespace

TEST(PipelineBuoysTracker, ActiveATargetPointsTowardBuoyB)
{
  auto value = tracker();
  ASSERT_EQ(value.offer(buoys(1)), "");
  const auto target = value.target(stamp(10));
  ASSERT_TRUE(target.has_value());
  EXPECT_DOUBLE_EQ(target->latitude, 33.810438);
  EXPECT_DOUBLE_EQ(target->longitude, -118.393392);
  ASSERT_TRUE(target->heading_valid);
  EXPECT_NEAR(target->heading_enu_rad, 0.0, 1e-3);   // east
  EXPECT_DOUBLE_EQ(target->uncertainty_m, 2.0);
  EXPECT_EQ(target->source_seq, 1u);
  EXPECT_FALSE(target->locked);
}

TEST(PipelineBuoysTracker, ActiveBTargetPointsTowardBuoyA)
{
  auto value = tracker();
  ASSERT_EQ(value.offer(buoys(1, PipelineBuoys::ACTIVE_B)), "");
  const auto target = value.target(stamp(10));
  ASSERT_TRUE(target.has_value());
  EXPECT_DOUBLE_EQ(target->longitude, -118.393272);
  ASSERT_TRUE(target->heading_valid);
  EXPECT_NEAR(std::abs(target->heading_enu_rad), kPi, 1e-3);   // west
}

TEST(PipelineBuoysTracker, RejectsWrongCourseSourceAndInvalidContent)
{
  auto value = tracker();
  auto other_course = buoys(1);
  other_course.course_id = "PRACTICE";
  EXPECT_EQ(value.offer(other_course).rfind("COURSE_MISMATCH", 0), 0u);

  auto uav = buoys(1);
  uav.source_vehicle = rx_msgs::msg::VehicleType::TYPE_UAV;
  EXPECT_EQ(value.offer(uav), "SOURCE_NOT_USV");

  auto nan = buoys(1);
  nan.buoy_a.latitude = std::nan("");
  EXPECT_EQ(value.offer(nan), "INVALID_BUOY_A");

  EXPECT_EQ(value.offer(buoys(1, PipelineBuoys::ACTIVE_B, false)), "INVALID_ACTIVE_BUOY");
  EXPECT_EQ(value.offer(buoys(1, 7)), "INVALID_ACTIVE_BUOY");

  auto outside = buoys(1);
  outside.buoy_b = point(33.8200, -118.3930);
  EXPECT_EQ(value.offer(outside), "OUTSIDE_COURSE");

  auto negative = buoys(1);
  negative.uncertainty_m = -1.0F;
  EXPECT_EQ(value.offer(negative), "INVALID_UNCERTAINTY");
  EXPECT_FALSE(value.hasObservation());
}

TEST(PipelineBuoysTracker, RequiresCourse)
{
  PipelineBuoysTracker value;
  EXPECT_EQ(value.offer(buoys(1)), "COURSE_UNAVAILABLE");
}

TEST(PipelineBuoysTracker, KeepsNewestSeq)
{
  auto value = tracker();
  ASSERT_EQ(value.offer(buoys(5)), "");
  EXPECT_EQ(value.offer(buoys(5)), "DUPLICATE");
  EXPECT_EQ(value.offer(buoys(4, PipelineBuoys::ACTIVE_B)), "STALE_SEQ");
  ASSERT_EQ(value.offer(buoys(6, PipelineBuoys::ACTIVE_B)), "");
  EXPECT_DOUBLE_EQ(value.target(stamp(1))->longitude, -118.393272);
}

TEST(PipelineBuoysTracker, AcceptsRestartedUsvOnlyWithNewerStamp)
{
  auto value = tracker();
  auto first = buoys(9);
  first.header.stamp = stamp(100);
  ASSERT_EQ(value.offer(first), "");
  auto restarted = buoys(1, PipelineBuoys::ACTIVE_B);
  EXPECT_EQ(value.offer(restarted), "STALE_SEQ");   // no stamp: cannot prove it is newer
  restarted.header.stamp = stamp(200);
  EXPECT_EQ(value.offer(restarted), "");
}

TEST(PipelineBuoysTracker, OneBuoyOrUnknownActiveGivesNoTarget)
{
  auto value = tracker();
  ASSERT_EQ(value.offer(buoys(1, PipelineBuoys::ACTIVE_UNKNOWN, false)), "");
  EXPECT_FALSE(value.target(stamp(1)).has_value());
  ASSERT_EQ(value.offer(buoys(2, PipelineBuoys::ACTIVE_A, false)), "");
  const auto target = value.target(stamp(1));
  ASSERT_TRUE(target.has_value());
  EXPECT_FALSE(target->heading_valid);   // no pipe direction from a single buoy
}

TEST(PipelineBuoysTracker, ZeroUncertaintyUsesConfiguredDefault)
{
  PipelineBuoysTracker::Config config;
  config.default_uncertainty_m = 4.0;
  auto value = tracker(config);
  auto unknown = buoys(1);
  unknown.uncertainty_m = 0.0F;
  ASSERT_EQ(value.offer(unknown), "");
  EXPECT_DOUBLE_EQ(value.target(stamp(1))->uncertainty_m, 4.0);
  value.beginRun(1, 1);
  EXPECT_DOUBLE_EQ(value.activeBuoyFix(stamp(1), 25000)->uncertainty_m, 4.0);
}

TEST(PipelineBuoysTracker, ActiveBuoyFixIsRunTaggedAndGreen)
{
  auto value = tracker();
  ASSERT_EQ(value.offer(buoys(1, PipelineBuoys::ACTIVE_B)), "");
  EXPECT_FALSE(value.activeBuoyFix(stamp(1), 25000).has_value());   // no run yet
  value.beginRun(3, 7);
  const auto buoy = value.activeBuoyFix(stamp(12), 25000);
  ASSERT_TRUE(buoy.has_value());
  EXPECT_EQ(buoy->run_id, 3u);
  EXPECT_EQ(buoy->epoch, 7u);
  EXPECT_GE(buoy->revision, 1u);
  EXPECT_EQ(buoy->source_id, "usv_pipeline_buoys");
  EXPECT_EQ(buoy->course_id, "ITSN-SIM-A");
  EXPECT_EQ(buoy->endpoint_id, "start_buoy");
  EXPECT_TRUE(buoy->green_confirmed);
  EXPECT_FALSE(buoy->pinger_confirmed);
  EXPECT_EQ(buoy->pinger_freq_hz, 25000u);
  EXPECT_DOUBLE_EQ(buoy->fix.longitude, -118.393272);
  EXPECT_EQ(buoy->header.stamp.sec, 12);
  EXPECT_EQ(buoy->fix.header.stamp.sec, 12);
}

TEST(PipelineBuoysTracker, FallbackOnlyWhenEnabledAndNeverGreenByDefault)
{
  auto without = tracker();
  without.beginRun(1, 1);
  EXPECT_FALSE(without.activeBuoyFix(stamp(1), 25000).has_value());

  PipelineBuoysTracker::Config config;
  config.fallback_enabled = true;
  config.fallback_latitude = 33.81;
  config.fallback_longitude = -118.39;
  auto with = tracker(config);
  with.beginRun(1, 1);
  auto buoy = with.activeBuoyFix(stamp(1), 25000);
  ASSERT_TRUE(buoy.has_value());
  EXPECT_DOUBLE_EQ(buoy->fix.latitude, 33.81);
  EXPECT_FALSE(buoy->green_confirmed);

  ASSERT_EQ(with.offer(buoys(1)), "");   // the USV report replaces the fallback
  buoy = with.activeBuoyFix(stamp(2), 25000);
  EXPECT_DOUBLE_EQ(buoy->fix.latitude, 33.810438);
  EXPECT_TRUE(buoy->green_confirmed);

  config.fallback_latitude = 0.0;
  config.fallback_longitude = 0.0;
  EXPECT_THROW(PipelineBuoysTracker{config}, std::invalid_argument);
}

TEST(PipelineBuoysTracker, LockedIgnoresUpdatesAfterDive)
{
  auto value = tracker();
  ASSERT_EQ(value.offer(buoys(1)), "");
  value.beginRun(1, 1);
  value.lock();
  EXPECT_EQ(value.offer(buoys(2, PipelineBuoys::ACTIVE_B)).rfind("LOCKED", 0), 0u);
  EXPECT_DOUBLE_EQ(value.activeBuoyFix(stamp(1), 25000)->fix.longitude, -118.393392);
  EXPECT_TRUE(value.target(stamp(1))->locked);
  value.unlock();
  EXPECT_EQ(value.offer(buoys(2, PipelineBuoys::ACTIVE_B)), "");
}

TEST(PipelineBuoysTracker, CourseChangeDropsObservation)
{
  auto value = tracker();
  ASSERT_EQ(value.offer(buoys(1)), "");
  value.setCourse(course());
  EXPECT_TRUE(value.hasObservation());
  value.setCourse(course("ITSN-SIM-B"));
  EXPECT_FALSE(value.hasObservation());
}

TEST(PipelineBuoysTracker, PolygonAndGeometryHelpers)
{
  const auto ring = course().boundary;
  EXPECT_TRUE(PipelineBuoysTracker::insidePolygon(point(33.8100, -118.3940), ring));
  EXPECT_FALSE(PipelineBuoysTracker::insidePolygon(point(33.8200, -118.3940), ring));
  const auto a = point(33.810438, -118.393392);
  const auto north = point(33.810538, -118.393392);
  EXPECT_NEAR(PipelineBuoysTracker::enuBearing(a, north), kPi / 2.0, 1e-6);
  EXPECT_NEAR(PipelineBuoysTracker::distanceM(a, north), 11.13, 0.05);
}
