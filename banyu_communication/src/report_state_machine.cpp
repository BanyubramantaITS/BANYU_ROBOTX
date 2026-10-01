#include "banyu_communication/report_state_machine.hpp"

#include <algorithm>

namespace banyu_communication
{

namespace
{
constexpr uint32_t kCrcPolynomial = 0xEDB88320U;

uint32_t crc_byte(uint32_t crc, uint8_t byte)
{
  crc ^= byte;
  for (int bit = 0; bit < 8; ++bit) {
    crc = (crc & 1U) != 0U ? (crc >> 1U) ^ kCrcPolynomial : crc >> 1U;
  }
  return crc;
}
}  // namespace

uint32_t mission_report_crc32(
  const std::string & mission_id,
  const std::vector<uint8_t> & ring_sequence)
{
  uint32_t crc = 0xFFFFFFFFU;
  for (const auto byte : mission_id) {
    crc = crc_byte(crc, static_cast<uint8_t>(byte));
  }
  for (const auto ring_id : ring_sequence) {
    crc = crc_byte(crc, ring_id);
  }
  return crc ^ 0xFFFFFFFFU;
}

ReportValidation validate_report(
  const std::string & mission_id,
  const std::vector<uint8_t> & ring_sequence, uint32_t supplied_crc32)
{
  ReportValidation validation;
  validation.calculated_crc32 = mission_report_crc32(mission_id, ring_sequence);

  if (mission_id.empty()) {
    validation.reason = "mission_id must not be empty";
    return validation;
  }
  if (ring_sequence.empty()) {
    validation.reason = "ring_sequence must not be empty";
    return validation;
  }
  if (supplied_crc32 != validation.calculated_crc32) {
    validation.reason = "crc32 does not match mission_id and ring_sequence";
    return validation;
  }

  validation.valid = true;
  validation.reason = "valid";
  return validation;
}

ReportRetryStateMachine::ReportRetryStateMachine(uint8_t configured_max_attempts)
: configured_max_attempts_(std::max<uint8_t>(configured_max_attempts, 1U)),
  max_attempts_(configured_max_attempts_)
{
}

void ReportRetryStateMachine::start(
  const banyu_msgs::msg::MissionReport & report,
  uint8_t requested_max_retries)
{
  report_ = report;
  const auto requested_max_attempts = requested_max_retries == 255U ?
    255U : static_cast<uint8_t>(requested_max_retries + 1U);
  max_attempts_ = std::min<uint8_t>(configured_max_attempts_, requested_max_attempts);
  max_attempts_ = std::max<uint8_t>(max_attempts_, 1U);
  attempts_ = 0;
  acknowledged_crc32_ = 0;
  state_ = State::WaitingForAck;
}

uint8_t ReportRetryStateMachine::record_publication()
{
  if (state_ != State::WaitingForAck || attempts_ >= max_attempts_) {
    return attempts_;
  }
  ++attempts_;
  return attempts_;
}

ReportRetryStateMachine::TimerEvent ReportRetryStateMachine::on_timer()
{
  if (state_ != State::WaitingForAck) {
    return TimerEvent::None;
  }
  if (attempts_ >= max_attempts_) {
    state_ = State::Exhausted;
    return TimerEvent::Exhausted;
  }
  return TimerEvent::Publish;
}

ReportRetryStateMachine::AckEvent ReportRetryStateMachine::on_ack(
  const banyu_msgs::msg::MissionReportAck & ack)
{
  if (state_ != State::WaitingForAck || !ack.acknowledged ||
    ack.mission_id != report_.mission_id || ack.crc32 != report_.crc32)
  {
    return AckEvent::Ignored;
  }

  acknowledged_crc32_ = ack.crc32;
  state_ = State::Acknowledged;
  return AckEvent::Accepted;
}

void ReportRetryStateMachine::cancel()
{
  if (state_ == State::WaitingForAck) {
    state_ = State::Canceled;
  }
}

ReportRetryStateMachine::State ReportRetryStateMachine::state() const
{
  return state_;
}

const banyu_msgs::msg::MissionReport & ReportRetryStateMachine::report() const
{
  return report_;
}

uint8_t ReportRetryStateMachine::attempts() const
{
  return attempts_;
}

uint8_t ReportRetryStateMachine::max_attempts() const
{
  return max_attempts_;
}

uint32_t ReportRetryStateMachine::acknowledged_crc32() const
{
  return acknowledged_crc32_;
}

}  // namespace banyu_communication
