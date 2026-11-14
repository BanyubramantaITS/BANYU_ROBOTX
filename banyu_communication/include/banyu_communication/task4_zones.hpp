#ifndef BANYU_COMMUNICATION_TASK4_ZONES_HPP_
#define BANYU_COMMUNICATION_TASK4_ZONES_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <banyu_msgs/msg/task4_zones.hpp>
#include <rx_msgs/msg/command.hpp>

namespace banyu_communication
{

// RoboCommand Task 4 keep-out state for the surfaced UUV (a surface vehicle is a USV): the static zones until ALL_CLEAR
// and the latest moving-object alert. Run-scoped (reset at prepare_autonomy, which opens a new run identity window).
class Task4ZoneTracker
{
public:
  struct Config
  {
    std::size_t max_static_zones{16};
    double max_radius_m{500.0};
    // A moving object that RoboCommand stopped updating is dropped after this long (the protocol has no all-clear for it).
    double moving_expiry_s{120.0};
    // The protocol has no ack for MOVING_OBJECT, the scoring awards one: off until settled with the organisers.
    bool ack_moving_object{false};
  };

  struct Result
  {
    // The command is a Task 4 zone command addressed to a vehicle this node represents (UUV, USV as the surface body, all).
    bool relevant{false};
    bool duplicate{false};
    bool rejected{false};
    // The zone state changed and must be published.
    bool changed{false};
    // Answer with an IncidentAck: only commands targeted at the UUV (or all vehicles), never the USV's own.
    bool ack{false};
    std::string event;
  };

  explicit Task4ZoneTracker(const Config & config);

  void reset();
  Result handle(const rx_msgs::msg::Command & command, double now_s);
  // Drops an expired moving object; true when the state changed.
  bool expire(double now_s);

  bool empty() const {return statics_.empty() && !moving_.has_value();}
  std::size_t staticCount() const {return statics_.size();}
  bool movingActive() const {return moving_.has_value();}
  double movingReceivedS() const {return moving_ ? moving_->received_s : 0.0;}
  // Fills the zone fields of the message (not the header or run identity).
  void fill(banyu_msgs::msg::Task4Zones & msg) const;

private:
  struct Static
  {
    double latitude;
    double longitude;
    double radius_m;
  };
  struct Moving
  {
    double latitude;
    double longitude;
    double heading_enu_rad;
    double speed_mps;
    double radius_m;
    double received_s;
    uint32_t seq;
  };

  Config config_;
  std::vector<Static> statics_;
  std::optional<Moving> moving_;
  std::set<std::pair<uint32_t, uint8_t>> seen_;
  uint32_t last_seq_{0};
  std::string last_event_;
};

}  // namespace banyu_communication

#endif  // BANYU_COMMUNICATION_TASK4_ZONES_HPP_
