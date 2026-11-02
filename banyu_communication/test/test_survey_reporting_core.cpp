#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>
#include <thread>
#include <vector>

#include <unistd.h>

#include "banyu_communication/survey_reporting.hpp"

namespace
{
using banyu_communication::SurveyLedger;
using banyu_communication::SurfaceGuard;
constexpr int64_t kSecond = 1000000000LL;

builtin_interfaces::msg::Time stamp(int64_t value)
{
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(value / kSecond);
  result.nanosec = static_cast<uint32_t>(value % kSecond);
  return result;
}

SurveyLedger::Config ledgerConfig(uint8_t tier = 2)
{
  SurveyLedger::Config config;
  config.evidence_sources = {"survey-association"};
  config.buoy_sources = {"observed-green-buoy"};
  config.tier = tier;
  return config;
}

banyu_msgs::msg::Task2SurveyEvidence evidence(uint8_t tier = 2)
{
  banyu_msgs::msg::Task2SurveyEvidence value;
  value.header.stamp = stamp(11 * kSecond);
  value.run_id = 13;
  value.epoch = 1;
  value.revision = 1;
  value.source_id = "survey-association";
  value.low_progress_endpoint_id = "west-end";
  value.high_progress_endpoint_id = "east-end";
  value.tier = tier;
  value.traversal_complete = true;
  value.active_buoy.header.stamp = stamp(10 * kSecond);
  value.active_buoy.run_id = 13;
  value.active_buoy.epoch = 1;
  value.active_buoy.revision = 1;
  value.active_buoy.source_id = "observed-green-buoy";
  value.active_buoy.course_id = "course-a";
  value.active_buoy.endpoint_id = "west-end";
  value.active_buoy.fix.header.stamp = stamp(10 * kSecond);
  value.active_buoy.fix.status.status = 0;
  value.active_buoy.fix.latitude = 33.0;
  value.active_buoy.fix.longitude = -118.0;
  value.active_buoy.uncertainty_m = 0.5;
  value.active_buoy.green_confirmed = true;
  value.active_buoy.pinger_freq_hz = 25000;
  for (uint32_t i = 0; i < 4; ++i) {
    banyu_msgs::msg::Task2RingRecord ring;
    ring.run_id = 13;
    ring.epoch = 1;
    ring.node_id = 100 + i;
    ring.frame_sequence = i + 1;
    ring.color = i == 2 ? 1 : 2;
    ring.confidence = 0.9F;
    ring.physical_identity_verified = true;
    ring.ordering_verified = true;
    ring.pipeline_progress_m = 0.1 + i * 10.0;
    ring.vehicle_pose_map.header.frame_id = "map";
    ring.vehicle_pose_map.header.stamp = stamp(10 * kSecond + i * 100000000LL);
    ring.vehicle_pose_map.pose.orientation.w = 1.0;
    value.rings.push_back(ring);
  }
  return value;
}

void initialize(SurveyLedger & ledger)
{
  ledger.beginRun(13, 1, 10 * kSecond);
  ledger.setCourse("course-a", 25000);
}

SurfaceGuard::Config guardConfig()
{
  SurfaceGuard::Config config;
  config.sources = {"calibrated-pressure"};
  return config;
}

banyu_msgs::msg::WaterDepth depth(int64_t source_stamp)
{
  banyu_msgs::msg::WaterDepth value;
  value.header.stamp = stamp(source_stamp);
  value.run_id = 13;
  value.epoch = 1;
  value.depth_m = 0.05;
  value.valid = true;
  value.calibrated = true;
  value.datum = "water_surface";
  value.source_id = "calibrated-pressure";
  value.calibration_id = "bench-calibration-1";
  return value;
}

void surfaceSamples(SurfaceGuard & guard, int64_t offset = 0)
{
  for (int i = 0; i <= 6; ++i) {
    const auto elapsed = offset + i * 200000000LL;
    guard.observe(depth(10 * kSecond + elapsed), 10 * kSecond + elapsed, elapsed);
  }
}


}  // namespace

TEST(SurveyLedger, ReportsAllRedPositionsFromBothObservedActiveEndpoints)
{
  for (std::size_t red = 0; red < 4; ++red) {
    for (const bool reverse : {false, true}) {
      SCOPED_TRACE(std::to_string(red) + (reverse ? " reverse" : " forward"));
      SurveyLedger ledger(ledgerConfig());
      initialize(ledger);
      auto value = evidence();
      for (std::size_t i = 0; i < 4; ++i) {
        value.rings[i].color = i == red ? 1 : 2;
      }
      if (reverse) {value.active_buoy.endpoint_id = "east-end";}
      std::reverse(value.rings.begin(), value.rings.end());
      ASSERT_TRUE(ledger.ingest(value, 11 * kSecond).empty());
      const auto report = ledger.build("ordered-report");
      for (std::size_t i = 0; i < 4; ++i) {
        const auto physical = reverse ? 3 - i : i;
        EXPECT_EQ(report.original_conditions[i], physical == red ? 2 : 1);
      }
    }
  }
}

TEST(SurveyLedger, BentPipeUsesProgressRatherThanVehicleDistanceOrAcceptanceOrder)
{
  SurveyLedger ledger(ledgerConfig());
  initialize(ledger);
  auto value = evidence();
  const std::vector<double> x{0, 10, 10, 0};
  const std::vector<double> y{0, 0, 10, 10};
  for (std::size_t i = 0; i < 4; ++i) {
    value.rings[i].vehicle_pose_map.pose.position.x = x[i];
    value.rings[i].vehicle_pose_map.pose.position.y = y[i];
  }
  std::swap(value.rings[1], value.rings[3]);
  ASSERT_TRUE(ledger.ingest(value, 11 * kSecond).empty());
  EXPECT_EQ(ledger.build("bent-pipe").original_conditions,
    (std::array<uint8_t, 4>{1, 1, 2, 1}));
  std::reverse(value.rings.begin(), value.rings.end());
  value.revision++;
  ASSERT_TRUE(ledger.ingest(value, 11 * kSecond).empty());
  EXPECT_EQ(ledger.build("bent-pipe").original_conditions,
    (std::array<uint8_t, 4>{1, 1, 2, 1}));
}

TEST(SurveyLedger, IncompleteAmbiguousAndUnverifiedDataNeverBuildsAReport)
{
  for (int variant = 0; variant < 9; ++variant) {
    SCOPED_TRACE(variant);
    SurveyLedger ledger(ledgerConfig());
    initialize(ledger);
    auto value = evidence();
    if (variant < 4) {value.rings.resize(variant);}
    if (variant == 4) {value.rings.push_back(value.rings.front());}
    if (variant == 5) {value.rings[1].node_id = value.rings[0].node_id;}
    if (variant == 6) {value.rings[0].physical_identity_verified = false;}
    if (variant == 7) {value.rings[0].ordering_verified = false;}
    if (variant == 8) {value.rings[0].color = 3;}
    EXPECT_FALSE(ledger.ingest(value, 11 * kSecond).empty());
    EXPECT_FALSE(ledger.complete());
    EXPECT_THROW(ledger.build("invalid"), std::logic_error);
  }
}

TEST(SurveyLedger, InvalidNewSnapshotBlocksReportAndPreservesOriginalRepairCondition)
{
  SurveyLedger ledger(ledgerConfig());
  initialize(ledger);
  auto value = evidence();
  ASSERT_TRUE(ledger.ingest(value, 11 * kSecond).empty());
  value.rings.push_back(value.rings.front());
  value.revision++;
  EXPECT_FALSE(ledger.ingest(value, 11 * kSecond).empty());
  EXPECT_FALSE(ledger.complete());
  value.rings.pop_back();
  for (const uint8_t post_repair_color : {uint8_t{2}, uint8_t{3}, uint8_t{1}}) {
    value.rings[2].color = post_repair_color;
    value.revision++;
    ASSERT_TRUE(ledger.ingest(value, 11 * kSecond).empty());
    EXPECT_EQ(ledger.build("original").original_conditions[2], 2u);
  }
}

TEST(SurveyLedger, PartialOriginalRedCannotBecomeIntactWhenCompleteSnapshotArrives)
{
  SurveyLedger ledger(ledgerConfig());
  initialize(ledger);
  auto partial = evidence();
  const auto red = partial.rings[2];
  partial.rings = {red};
  EXPECT_FALSE(ledger.ingest(partial, 11 * kSecond).empty());
  auto complete = evidence();
  complete.revision = 2;
  complete.rings[2].color = 2;
  ASSERT_TRUE(ledger.ingest(complete, 11 * kSecond).empty());
  EXPECT_EQ(ledger.build("partial-original").original_conditions[2], 2u);
  complete.revision++;
  complete.rings[2].pipeline_progress_m += 0.01;
  EXPECT_FALSE(ledger.ingest(complete, 11 * kSecond).empty());
  EXPECT_FALSE(ledger.complete());
}

TEST(SurveyLedger, MissingRunOriginAndMalformedSourceStampsFailClosed)
{
  SurveyLedger ledger(ledgerConfig());
  ledger.beginRun(13, 1, 0);
  ledger.setCourse("course-a", 25000);
  EXPECT_EQ(ledger.ingest(evidence(), 11 * kSecond), "REPORT_RUN_CONTEXT_UNVERIFIED");
  initialize(ledger);
  auto value = evidence();
  value.rings[0].vehicle_pose_map.header.stamp.nanosec = 1000000000;
  EXPECT_FALSE(ledger.ingest(value, 11 * kSecond).empty());
  EXPECT_FALSE(ledger.complete());
}

TEST(SurveyLedger, WrongRunOldRevisionAndUntrustedSourcesDoNotEraseAcceptedEvidence)
{
  SurveyLedger ledger(ledgerConfig());
  initialize(ledger);
  auto value = evidence();
  ASSERT_TRUE(ledger.ingest(value, 11 * kSecond).empty());
  value.rings.clear();
  EXPECT_EQ(ledger.ingest(value, 11 * kSecond), "STALE_EVIDENCE");
  value.revision++;
  value.epoch = 2;
  EXPECT_EQ(ledger.ingest(value, 11 * kSecond), "WRONG_RUN");
  value.epoch = 1;
  value.source_id = "untrusted";
  EXPECT_EQ(ledger.ingest(value, 11 * kSecond), "EVIDENCE_SOURCE_UNTRUSTED");
  EXPECT_TRUE(ledger.complete());
  EXPECT_EQ(ledger.build("preserved").original_conditions[2], 2u);
}

TEST(SurveyLedger, ActiveBuoyMustBeObservedAndPinnedToCourseAndEndpoint)
{
  for (int variant = 0; variant < 7; ++variant) {
    SurveyLedger ledger(ledgerConfig());
    initialize(ledger);
    auto value = evidence();
    if (variant == 0) {value.active_buoy.green_confirmed = false;}
    if (variant == 1) {value.active_buoy.course_id = "different-course";}
    if (variant == 2) {value.active_buoy.fix.status.status = -1;}
    if (variant == 3) {value.active_buoy.uncertainty_m = 100;}
    if (variant == 4) {value.active_buoy.endpoint_id = "unknown-end";}
    if (variant == 5) {
      value.active_buoy.pinger_confirmed = true;
      value.active_buoy.pinger_freq_hz = 35000;
    }
    if (variant == 6) {
      value.active_buoy.fix.latitude =
        std::numeric_limits<double>::quiet_NaN();
    }
    EXPECT_FALSE(ledger.ingest(value, 11 * kSecond).empty());
    EXPECT_FALSE(ledger.complete());
  }
  SurveyLedger ledger(ledgerConfig());
  initialize(ledger);
  auto value = evidence();
  value.active_buoy.header.stamp = stamp(9500000000LL);
  value.active_buoy.fix.header.stamp = value.active_buoy.header.stamp;
  ASSERT_TRUE(ledger.ingest(value, 11 * kSecond).empty());
  value.revision++;
  value.header.stamp = stamp(60 * kSecond);
  ASSERT_TRUE(ledger.ingest(value, 60 * kSecond).empty());
  value.revision++;
  value.active_buoy.endpoint_id = "east-end";
  EXPECT_FALSE(ledger.ingest(value, 60 * kSecond).empty());
  EXPECT_FALSE(ledger.complete());
}

TEST(SurveyLedger, HigherTiersRequireSeparatePhysicalRepairAndCompleteObservedCodes)
{
  for (const uint8_t tier : {uint8_t{3}, uint8_t{4}}) {
    SurveyLedger ledger(ledgerConfig(tier));
    initialize(ledger);
    auto value = evidence(tier);
    EXPECT_EQ(ledger.ingest(value, 11 * kSecond), "REPAIR_UNVERIFIED");
    value.revision++;
    value.repair_verified = true;
    EXPECT_EQ(ledger.ingest(value, 11 * kSecond), "RESOURCE_CODE_INCOMPLETE");
    value.revision++;
    value.resource_code_complete = true;
    value.resource_color = tier == 3 ? 4 : 2;
    value.delivery_circle_color = 3;
    ASSERT_TRUE(ledger.ingest(value, 11 * kSecond).empty());
    const auto report = ledger.build("higher-tier");
    EXPECT_EQ(report.resource_color, tier == 3 ? 4 : 2);
    EXPECT_EQ(report.delivery_circle_color, 3u);
    EXPECT_EQ(report.original_conditions[2], 2u);
  }
}

TEST(SurveyLedger, CourseChangeRemainsBlockedUntilAnExplicitNewRun)
{
  SurveyLedger ledger(ledgerConfig());
  initialize(ledger);
  auto value = evidence();
  ASSERT_TRUE(ledger.ingest(value, 11 * kSecond).empty());
  ledger.setCourse("course-b", 35000);
  value.revision++;
  EXPECT_EQ(ledger.ingest(value, 11 * kSecond), "COURSE_CHANGED_DURING_SURVEY");
  EXPECT_FALSE(ledger.complete());
}

TEST(SurfaceGuard, RequiresContinuousFreshCalibratedSamplesRatherThanOneCrossing)
{
  SurfaceGuard guard(guardConfig());
  guard.beginRun(13, 1);
  EXPECT_FALSE(guard.ready(10 * kSecond, 0));
  guard.observe(depth(10 * kSecond), 10 * kSecond, 0);
  EXPECT_FALSE(guard.ready(10 * kSecond, 0));
  surfaceSamples(guard);
  EXPECT_TRUE(guard.ready(11200000000LL, 1200000000LL));
  EXPECT_FALSE(guard.ready(11200000000LL, 1800000000LL));
  EXPECT_EQ(guard.reason(), "SURFACE_TELEMETRY_STALE");
}

TEST(SurfaceGuard, InvalidDatumFutureStampsAndExcursionsResetHold)
{
  for (int variant = 0; variant < 8; ++variant) {
    SurfaceGuard guard(guardConfig());
    guard.beginRun(13, 1);
    surfaceSamples(guard);
    ASSERT_TRUE(guard.ready(11200000000LL, 1200000000LL));
    auto value = depth(11400000000LL);
    if (variant == 0) {value.valid = false;}
    if (variant == 1) {value.calibrated = false;}
    if (variant == 2) {value.datum = "map";}
    if (variant == 3) {value.calibration_id.clear();}
    if (variant == 4) {value.depth_m = std::numeric_limits<double>::quiet_NaN();}
    if (variant == 5) {value.depth_m = 2.0;}
    if (variant == 6) {value.header.stamp = stamp(12 * kSecond);}
    if (variant == 7) {value.calibration_id = "changed-calibration";}
    guard.observe(value, 11400000000LL, 1400000000LL);
    EXPECT_FALSE(guard.ready(11400000000LL, 1400000000LL));
  }
}

TEST(SurfaceGuard, RepeatedStampDuringClockPauseCannotRenewProof)
{
  SurfaceGuard guard(guardConfig());
  guard.beginRun(13, 1);
  surfaceSamples(guard);
  ASSERT_TRUE(guard.ready(11200000000LL, 1200000000LL));
  for (int i = 7; i < 12; ++i) {
    guard.observe(depth(11200000000LL), 11200000000LL, i * 200000000LL);
  }
  EXPECT_FALSE(guard.ready(11200000000LL, 2200000000LL));
}

TEST(SurfaceGuard, GapFromFirstSampleAndNewRunCannotCarryOverDwell)
{
  SurfaceGuard guard(guardConfig());
  guard.beginRun(13, 1);
  guard.observe(depth(10 * kSecond), 10 * kSecond, 0);
  guard.observe(depth(11 * kSecond), 11 * kSecond, kSecond);
  EXPECT_FALSE(guard.ready(11 * kSecond, kSecond));
  surfaceSamples(guard, kSecond);
  ASSERT_TRUE(guard.ready(12200000000LL, 2200000000LL));
  guard.beginRun(14, 2);
  guard.observe(depth(12400000000LL), 12400000000LL, 2400000000LL);
  EXPECT_FALSE(guard.ready(12400000000LL, 2400000000LL));
}


TEST(SurveyLedger, MixedValidInvalidSnapshotPreservesOriginalRed)
{
  SurveyLedger ledger(ledgerConfig());
  initialize(ledger);
  auto mixed = evidence();
  mixed.rings[0].color = 1;
  mixed.rings[1].confidence = 2.0F;

  EXPECT_FALSE(ledger.ingest(mixed, 11 * kSecond).empty());
  EXPECT_FALSE(ledger.complete());

  auto clean = evidence();
  clean.revision = 2;
  for (auto & ring : clean.rings) {
    ring.color = 2;
  }
  ASSERT_TRUE(ledger.ingest(clean, 11 * kSecond).empty());
  EXPECT_TRUE(ledger.complete());

  const auto report = ledger.build("mixed-preserved-red");
  EXPECT_EQ(report.original_conditions[0], 2u);
}

TEST(SurveyLedger, OriginalGreenMustRemainIntactAfterLaterRedFlash)
{
  SurveyLedger ledger(ledgerConfig());
  initialize(ledger);
  auto value = evidence();
  ASSERT_TRUE(ledger.ingest(value, 11 * kSecond).empty());
  ASSERT_EQ(ledger.build("audit-green").original_conditions[0], 1u);

  // The red flash only comes after a verified repair.
  value.revision++;
  value.repair_verified = true;
  value.rings[0].color = 1;
  ASSERT_TRUE(ledger.ingest(value, 11 * kSecond).empty());
  EXPECT_EQ(ledger.build("audit-green").original_conditions[0], 1u);
}

TEST(SurveyLedger, FirstSeenGreenThenRedBeforeRepairIsOriginallyRed)
{
  SurveyLedger ledger(ledgerConfig());
  initialize(ledger);
  auto value = evidence();
  value.repair_verified = false;
  ASSERT_TRUE(ledger.ingest(value, 11 * kSecond).empty());
  ASSERT_EQ(ledger.build("far-green").original_conditions[0], 1u);

  value.revision++;
  value.rings[0].color = 1;
  ASSERT_TRUE(ledger.ingest(value, 11 * kSecond).empty());
  EXPECT_EQ(ledger.build("far-green").original_conditions[0], 2u);
}

TEST(SurveyLedger, OriginalRedMustSurviveUnknownNodeEarlierInSnapshot)
{
  SurveyLedger ledger(ledgerConfig());
  initialize(ledger);
  auto value = evidence();
  value.rings[0].color = 0;
  EXPECT_FALSE(ledger.ingest(value, 11 * kSecond).empty());

  value = evidence();
  value.revision = 2;
  value.rings[2].color = 2;
  ASSERT_TRUE(ledger.ingest(value, 11 * kSecond).empty());
  EXPECT_EQ(ledger.build("audit-mixed").original_conditions[2], 2u);
}

TEST(SurveyLedger, ReorderedSnapshotAndRevisitsPreserveOriginalSurveyConditions)
{
  SurveyLedger ledger(ledgerConfig());
  initialize(ledger);
  auto value = evidence();
  std::reverse(value.rings.begin(), value.rings.end());
  ASSERT_TRUE(ledger.ingest(value, 11 * kSecond).empty());
  EXPECT_TRUE(ledger.complete());

  auto initial_report = ledger.build("run-13-reordered");
  EXPECT_EQ(initial_report.original_conditions[2], 2u);
  EXPECT_EQ(initial_report.original_conditions[0], 1u);
  EXPECT_EQ(initial_report.original_conditions[1], 1u);
  EXPECT_EQ(initial_report.original_conditions[3], 1u);

  value.revision++;
  for (auto & ring : value.rings) {
    if (ring.node_id == 102) {
      ring.color = 2;
    } else if (ring.node_id == 100) {
      ring.color = 3;
    }
  }
  ASSERT_TRUE(ledger.ingest(value, 11 * kSecond).empty());
  auto revisit_report = ledger.build("run-13-revisit");
  EXPECT_EQ(revisit_report.original_conditions[2], 2u);
  EXPECT_EQ(revisit_report.original_conditions[0], 1u);
}

TEST(SurfaceGuard, SingleSampleMustNotVerifyContinuousSurfacing)
{
  auto config = guardConfig();
  config.hold_s = 0.15;
  config.ttl_s = 0.2;
  SurfaceGuard guard(config);
  guard.beginRun(13, 1);
  guard.observe(depth(10 * kSecond), 10 * kSecond, kSecond);
  EXPECT_FALSE(guard.ready(10 * kSecond + 160000000LL, kSecond + 160000000LL));
}

TEST(SurveyLedger, BestEffortReportsWhatWasSeenWithTheVerifiedBuoy)
{
  SurveyLedger ledger(ledgerConfig());
  initialize(ledger);
  auto value = evidence();
  value.rings.resize(3);   // rings 100 (GREEN), 101 (GREEN), 102 (RED) seen, the fourth never reached
  value.traversal_complete = false;
  EXPECT_FALSE(ledger.ingest(value, 11 * kSecond).empty());
  EXPECT_FALSE(ledger.complete());
  ASSERT_TRUE(ledger.bestEffortAvailable());
  const auto report = ledger.buildBestEffort("task2-survey-13-1.best_effort");
  EXPECT_EQ(report.tier, 2);
  EXPECT_DOUBLE_EQ(report.buoy_latitude, 33.0);
  const std::array<uint8_t, 4> expected{1, 1, 2, 1};   // GREEN, GREEN, RED as seen, then GREEN
  EXPECT_EQ(report.original_conditions, expected);
}

TEST(SurveyLedger, BestEffortWithNothingSeenIsAllGreenAndNeedsABuoy)
{
  SurveyLedger ledger(ledgerConfig());
  initialize(ledger);
  EXPECT_FALSE(ledger.bestEffortAvailable());
  EXPECT_THROW(ledger.buildBestEffort("x.best_effort"), std::logic_error);
  auto value = evidence();
  value.rings.clear();
  value.traversal_complete = false;
  ledger.ingest(value, 11 * kSecond);
  ASSERT_TRUE(ledger.bestEffortAvailable());
  const std::array<uint8_t, 4> all_green{1, 1, 1, 1};
  EXPECT_EQ(ledger.buildBestEffort("x.best_effort").original_conditions, all_green);
}

TEST(SurveyLedger, BestEffortOfACompleteSurveyIsTheFullReport)
{
  SurveyLedger ledger(ledgerConfig());
  initialize(ledger);
  ASSERT_TRUE(ledger.ingest(evidence(), 11 * kSecond).empty());
  EXPECT_EQ(ledger.buildBestEffort("x.best_effort").original_conditions,
    ledger.build("x").original_conditions);
}

TEST(SurveyLedger, BestEffortKeepsAVerifiedFlashCode)
{
  SurveyLedger ledger(ledgerConfig(4));
  initialize(ledger);
  auto value = evidence(4);
  value.repair_verified = true;
  value.resource_code_complete = true;
  value.resource_color = 3;
  value.delivery_circle_color = 2;
  value.rings[1].ordering_verified = false;   // the order could not be verified
  EXPECT_FALSE(ledger.ingest(value, 11 * kSecond).empty());
  ASSERT_TRUE(ledger.bestEffortAvailable());
  const auto report = ledger.buildBestEffort("x.best_effort");
  EXPECT_EQ(report.tier, 4);
  EXPECT_EQ(report.resource_color, 3);
  EXPECT_EQ(report.delivery_circle_color, 2);
}
