#ifndef BANYU_COMMUNICATION__REPORT_STATE_MACHINE_HPP_
#define BANYU_COMMUNICATION__REPORT_STATE_MACHINE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "banyu_msgs/msg/mission_report.hpp"
#include "banyu_msgs/msg/mission_report_ack.hpp"

namespace banyu_communication
{

uint32_t mission_report_crc32(
  const std::string & mission_id,
  const std::vector<uint8_t> & ring_sequence);

struct ReportValidation
{
  bool valid{false};
  std::string reason;
  uint32_t calculated_crc32{0};
};

ReportValidation validate_report(
  const std::string & mission_id,
  const std::vector<uint8_t> & ring_sequence, uint32_t supplied_crc32);

class ReportRetryStateMachine
{
public:
  enum class State
  {
    Idle,
    WaitingForAck,
    Acknowledged,
    Exhausted,
    Canceled
  };

  enum class TimerEvent
  {
    None,
    Publish,
    Exhausted
  };

  enum class AckEvent
  {
    Ignored,
    Accepted
  };

  // configured_max_attempts is a hard cap on total publications.
  explicit ReportRetryStateMachine(uint8_t configured_max_attempts);

  void start(
    const banyu_msgs::msg::MissionReport & report,
    uint8_t requested_max_retries);
  uint8_t record_publication();
  TimerEvent on_timer();
  AckEvent on_ack(const banyu_msgs::msg::MissionReportAck & ack);
  void cancel();

  State state() const;
  const banyu_msgs::msg::MissionReport & report() const;
  uint8_t attempts() const;
  uint8_t max_attempts() const;
  uint32_t acknowledged_crc32() const;

private:
  banyu_msgs::msg::MissionReport report_;
  uint8_t configured_max_attempts_{1};
  uint8_t max_attempts_{1};
  uint8_t attempts_{0};
  uint32_t acknowledged_crc32_{0};
  State state_{State::Idle};
};

}  // namespace banyu_communication

#endif  // BANYU_COMMUNICATION__REPORT_STATE_MACHINE_HPP_
