#pragma once

#include <cstdint>
#include <string>

namespace banyu_teleop
{

// Wire protocol of the ESP32 kill-switch indicator (firmware indikator_banyu.ino).
// The board numbers its states differently from VehicleStatus.operational_mode:
//   board "its0" = MANUAL LED, "its1" = AUTO LED, "its2" = EMERGENCY LED.
// It accepts newline-terminated "itsN" lines, ignores its0/its1 while its hardware kill input
// is active, and reports its current state as "itsN\r\n" every 500 ms.

// operational_mode (0 kill, 1 manual, 2 autonomous) -> line to send.
inline std::string indicatorCommand(uint8_t operational_mode)
{
  switch (operational_mode) {
    case 1:
      return "its0\n";
    case 2:
      return "its1\n";
    default:
      return "its2\n";
  }
}

// Reported line -> operational_mode, or -1 when the line is not a state report.
inline int parseIndicatorReport(const std::string & line)
{
  const size_t first = line.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return -1;
  }
  const size_t last = line.find_last_not_of(" \t\r\n");
  const std::string trimmed = line.substr(first, last - first + 1);
  if (trimmed == "its0") {
    return 1;
  }
  if (trimmed == "its1") {
    return 2;
  }
  if (trimmed == "its2") {
    return 0;
  }
  return -1;
}

}   // namespace banyu_teleop
