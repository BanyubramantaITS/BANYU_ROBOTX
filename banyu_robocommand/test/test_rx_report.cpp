#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "banyu_robocommand/rx_report.hpp"

using banyu_msgs::msg::PipelineSurveyReport;

namespace {
std::string hex(const std::string& bytes) {
  static const char* digits = "0123456789abcdef";
  std::string out;
  for (const unsigned char c : bytes) {
    out.push_back(digits[c >> 4]);
    out.push_back(digits[c & 15]);
  }
  return out;
}
}  // namespace

// Golden bytes from RoboNation's generated Python classes (robocommand RobotX_2026, protobuf 7.35):
//   RxReport(team_id='ITSN', vehicle_id='UUV1', seq=7, sent_at=1790528432.5 s,
//            pipeline_survey={active_buoy_position=(1.25, 103.86), segments=[I, D, I, I, I]})
TEST(RxReport, MatchesOfficialEncodingWithFix) {
  PipelineSurveyReport msg;
  msg.active_buoy_latitude = 1.25;
  msg.active_buoy_longitude = 103.86;
  msg.segment_states = {0, 1, 0, 0, 0};
  EXPECT_EQ(hex(banyu_robocommand::encodePipelineSurveyReport(msg, "ITSN", "UUV1", 7, 1790528432500000000LL)),
            "0a044954534e1204555556311807220c08b097e5d5061080cab5ee01621b0a1209000000000000f43f11d7a3703d0af7594012050102010101");
}

// No GPS fix: active_buoy_position unset; a zero Timestamp.nanos is omitted (proto3).
TEST(RxReport, MatchesOfficialEncodingWithoutFix) {
  PipelineSurveyReport msg;
  msg.active_buoy_latitude = std::nan("");
  msg.active_buoy_longitude = std::nan("");
  msg.segment_states = {1};
  EXPECT_EQ(hex(banyu_robocommand::encodePipelineSurveyReport(msg, "ITSN", "UUV1", 1, 5000000000LL)),
            "0a044954534e1204555556311801220208056203120102");
}

// protoc 31 --encode on proto/ (edition lowered to 2023; wire format unchanged, and the same
// route reproduces the second golden above):
//   RxReport(team_id='ITSN', vehicle_id='UUV1', seq=3, sent_at=1790528432.5 s,
//            resource_delivery={task=TASK_INFRA_SURVEY_REPAIR, resource_color=COLOR_ANY,
//                               delivery_circle_color=COLOR_BLUE})
TEST(RxReport, ResourceDeliveryRequest) {
  banyu_msgs::msg::ResourceDeliveryRequest msg;
  msg.task = msg.TASK_INFRA_SURVEY_REPAIR;
  msg.resource_color = msg.COLOR_ANY;
  msg.delivery_circle_color = msg.COLOR_BLUE;
  EXPECT_EQ(hex(banyu_robocommand::encodeResourceDeliveryRequest(msg, "ITSN", "UUV1", 3, 1790528432500000000LL)),
            "0a044954534e1204555556311803220c08b097e5d5061080cab5ee016a06080310041803");
}

TEST(RxReport, Topic) {
  EXPECT_EQ(banyu_robocommand::reportTopic("ITSN", "UUV1"), "robocommand/robotx/ITSN/UUV1/report");
}
