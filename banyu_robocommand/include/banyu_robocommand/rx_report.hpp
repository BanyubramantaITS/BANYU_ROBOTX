// RoboCommand RxReport{pipeline_survey, resource_delivery} encoding (proto/robotx/rx_reports.proto).
//
// Encoded by hand to the protobuf wire format: RoboNation's generated C++ needs protobuf 35
// (edition 2024) and Ubuntu 24.04 ships 3.21. Field numbers and encodings follow the schema in
// proto/, and test/test_rx_report.cpp pins the output to bytes produced by their official classes.
// ponytail: these two messages only; switch to their generated code once the OCS has protobuf >= 35.
#ifndef BANYU_ROBOCOMMAND_RX_REPORT_HPP_
#define BANYU_ROBOCOMMAND_RX_REPORT_HPP_

#include <cstdint>
#include <string>

#include <banyu_msgs/msg/pipeline_survey_report.hpp>
#include <banyu_msgs/msg/resource_delivery_request.hpp>

namespace banyu_robocommand {

// MQTT topic an OCS publishes a vehicle's reports on (handbook 3.4.6).
std::string reportTopic(const std::string& team_id, const std::string& vehicle_id);

// Serialized RxReport carrying one PipelineSurveyReport. A NaN buoy latitude/longitude (no GPS
// fix) leaves active_buoy_position unset.
std::string encodePipelineSurveyReport(const banyu_msgs::msg::PipelineSurveyReport& msg, const std::string& team_id,
                                       const std::string& vehicle_id, uint32_t seq, int64_t sent_at_ns);

// Serialized RxReport carrying one ResourceDeliveryRequest (Task 2/3 Advanced and Disruptive).
std::string encodeResourceDeliveryRequest(const banyu_msgs::msg::ResourceDeliveryRequest& msg,
                                          const std::string& team_id, const std::string& vehicle_id, uint32_t seq,
                                          int64_t sent_at_ns);

}  // namespace banyu_robocommand

#endif  // BANYU_ROBOCOMMAND_RX_REPORT_HPP_
