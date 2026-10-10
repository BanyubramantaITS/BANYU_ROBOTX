#ifndef BANYU_MSGS__TASK2_SUBMISSION_HPP_
#define BANYU_MSGS__TASK2_SUBMISSION_HPP_

#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>

#include "banyu_msgs/msg/task2_submission.hpp"

namespace banyu_msgs::task2
{
inline bool validSubmission(const msg::Task2Submission & value)
{
  const bool colors = value.delivery_circle_color >= 1 && value.delivery_circle_color <= 3 &&
    ((value.tier == 3 && value.resource_color == 4) ||
    (value.tier == 4 && value.resource_color >= 1 && value.resource_color <= 3));
  if (value.schema_version != 1 || value.run_id == 0 || value.epoch == 0 ||
    value.report_id.empty() || value.report_id.size() > 128 || value.course_id.empty() ||
    value.course_id.size() > 128 || value.tier < 2 || value.tier > 4 ||
    (value.kind != msg::Task2Submission::SURVEY && value.kind != msg::Task2Submission::DELIVERY) ||
    !std::isfinite(value.buoy_latitude) || std::abs(value.buoy_latitude) > 90.0 ||
    !std::isfinite(value.buoy_longitude) || std::abs(value.buoy_longitude) > 180.0 ||
    (value.tier != 2 && !colors) || (value.kind == msg::Task2Submission::DELIVERY && !colors))
  {return false;}
  for (const char character : value.report_id) {
    if (!((character >= 'a' && character <= 'z') || (character >= 'A' && character <= 'Z') ||
      (character >= '0' && character <= '9') || character == '-' || character == '_' ||
      character == '.')) {return false;}
  }
  for (const auto condition : value.original_conditions) {
    if (condition != 1 && condition != 2) {return false;}
  }
  return true;
}

// Publication timestamps and authorization receipts do not change the immutable payload.
inline std::string canonicalPayload(const msg::Task2Submission & value)
{
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::setprecision(std::numeric_limits<double>::max_digits10) <<
    static_cast<unsigned>(value.schema_version) << ' ' << value.run_id << ' ' << value.epoch <<
    ' ' << std::quoted(value.report_id) << ' ' << static_cast<unsigned>(value.kind) << ' ' <<
    std::quoted(value.course_id) << ' ' << static_cast<unsigned>(value.tier) << ' ' <<
    value.buoy_latitude << ' ' << value.buoy_longitude;
  for (const auto condition : value.original_conditions) {
    stream << ' ' << static_cast<unsigned>(condition);
  }
  stream << ' ' << static_cast<unsigned>(value.resource_color) << ' ' <<
    static_cast<unsigned>(value.delivery_circle_color);
  return stream.str();
}

inline bool parseSubmission(const std::string & payload, msg::Task2Submission & submission)
{
  std::istringstream input(payload);
  input.imbue(std::locale::classic());
  uint64_t version = 0;
  uint64_t run = 0;
  uint64_t kind = 0;
  uint64_t tier = 0;
  if (!(input >> version >> run >> submission.epoch >> std::quoted(submission.report_id) >>
    kind >> std::quoted(submission.course_id) >> tier >> submission.buoy_latitude >>
    submission.buoy_longitude) || version > 255 || run > std::numeric_limits<uint32_t>::max() ||
    kind > 255 || tier > 255)
  {
    return false;
  }
  submission.schema_version = static_cast<uint8_t>(version);
  submission.run_id = static_cast<uint32_t>(run);
  submission.kind = static_cast<uint8_t>(kind);
  submission.tier = static_cast<uint8_t>(tier);
  for (auto & condition : submission.original_conditions) {
    uint64_t value = 0;
    if (!(input >> value) || value > 255) {
      return false;
    }
    condition = static_cast<uint8_t>(value);
  }
  uint64_t resource = 0;
  uint64_t delivery = 0;
  if (!(input >> resource >> delivery) || resource > 255 || delivery > 255) {
    return false;
  }
  submission.resource_color = static_cast<uint8_t>(resource);
  submission.delivery_circle_color = static_cast<uint8_t>(delivery);
  input >> std::ws;
  return input.eof() && validSubmission(submission) &&
         canonicalPayload(submission) == payload;
}
}  // namespace banyu_msgs::task2

#endif  // BANYU_MSGS__TASK2_SUBMISSION_HPP_
