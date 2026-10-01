#include "banyu_robocommand/rx_report.hpp"

#include <cmath>
#include <cstring>

namespace banyu_robocommand {

namespace {

// Protobuf wire types.
constexpr uint32_t kVarint = 0, kFixed64 = 1, kLengthDelimited = 2;

// Field numbers from proto/.
namespace rx_report {
constexpr uint32_t kTeamId = 1, kVehicleId = 2, kSeq = 3, kSentAt = 4, kPipelineSurvey = 12, kResourceDelivery = 13;
}
namespace timestamp {  // google.protobuf.Timestamp (proto3: zero fields are omitted)
constexpr uint32_t kSeconds = 1, kNanos = 2;
}
namespace pipeline_survey {
constexpr uint32_t kActiveBuoyPosition = 1, kSegments = 2;  // segments: repeated enum, packed
}
namespace resource_delivery {
constexpr uint32_t kTask = 1, kResourceColor = 2, kDeliveryCircleColor = 3;
}
namespace lat_lng {
constexpr uint32_t kLatitude = 1, kLongitude = 2;
}
// robocommand.robotx.v1.PipelineSegmentStatus (0 = UNKNOWN).
constexpr uint64_t kSegmentIntact = 1, kSegmentDamaged = 2;

void varint(std::string& out, uint64_t v) {
  while (v >= 0x80) {
    out.push_back(static_cast<char>((v & 0x7f) | 0x80));
    v >>= 7;
  }
  out.push_back(static_cast<char>(v));
}

void tag(std::string& out, uint32_t field, uint32_t wire_type) { varint(out, (uint64_t{field} << 3) | wire_type); }

void bytesField(std::string& out, uint32_t field, const std::string& bytes) {
  tag(out, field, kLengthDelimited);
  varint(out, bytes.size());
  out += bytes;
}

void doubleField(std::string& out, uint32_t field, double v) {
  tag(out, field, kFixed64);
  char buf[8];
  std::memcpy(buf, &v, 8);  // little-endian IEEE 754, as on every target this runs on
  out.append(buf, 8);
}

void enumField(std::string& out, uint32_t field, uint64_t v) {
  if (v == 0) return;  // proto3 implicit presence: the zero value is not sent
  tag(out, field, kVarint);
  varint(out, v);
}

// RxReport envelope around one body in oneof field `body_field`.
std::string envelope(const std::string& team_id, const std::string& vehicle_id, uint32_t seq, int64_t sent_at_ns,
                     uint32_t body_field, const std::string& body) {
  std::string stamp;
  const int64_t seconds = sent_at_ns / 1000000000, nanos = sent_at_ns % 1000000000;
  if (seconds != 0) {
    tag(stamp, timestamp::kSeconds, kVarint);
    varint(stamp, static_cast<uint64_t>(seconds));
  }
  if (nanos != 0) {
    tag(stamp, timestamp::kNanos, kVarint);
    varint(stamp, static_cast<uint64_t>(nanos));
  }

  std::string out;
  bytesField(out, rx_report::kTeamId, team_id);
  bytesField(out, rx_report::kVehicleId, vehicle_id);
  tag(out, rx_report::kSeq, kVarint);
  varint(out, seq);
  bytesField(out, rx_report::kSentAt, stamp);
  bytesField(out, body_field, body);
  return out;
}

}  // namespace

std::string reportTopic(const std::string& team_id, const std::string& vehicle_id) {
  return "robocommand/robotx/" + team_id + "/" + vehicle_id + "/report";
}

std::string encodePipelineSurveyReport(const banyu_msgs::msg::PipelineSurveyReport& msg, const std::string& team_id,
                                       const std::string& vehicle_id, uint32_t seq, int64_t sent_at_ns) {
  std::string body;
  if (!std::isnan(msg.active_buoy_latitude) && !std::isnan(msg.active_buoy_longitude)) {
    std::string pos;
    doubleField(pos, lat_lng::kLatitude, msg.active_buoy_latitude);
    doubleField(pos, lat_lng::kLongitude, msg.active_buoy_longitude);
    bytesField(body, pipeline_survey::kActiveBuoyPosition, pos);
  }
  if (!msg.segment_states.empty()) {
    std::string packed;
    for (const uint8_t s : msg.segment_states)
      varint(packed, s == banyu_msgs::msg::PipelineSurveyReport::SEGMENT_DAMAGED ? kSegmentDamaged : kSegmentIntact);
    bytesField(body, pipeline_survey::kSegments, packed);
  }
  return envelope(team_id, vehicle_id, seq, sent_at_ns, rx_report::kPipelineSurvey, body);
}

std::string encodeResourceDeliveryRequest(const banyu_msgs::msg::ResourceDeliveryRequest& msg,
                                          const std::string& team_id, const std::string& vehicle_id, uint32_t seq,
                                          int64_t sent_at_ns) {
  std::string body;
  enumField(body, resource_delivery::kTask, msg.task);
  enumField(body, resource_delivery::kResourceColor, msg.resource_color);
  enumField(body, resource_delivery::kDeliveryCircleColor, msg.delivery_circle_color);
  return envelope(team_id, vehicle_id, seq, sent_at_ns, rx_report::kResourceDelivery, body);
}

}  // namespace banyu_robocommand
