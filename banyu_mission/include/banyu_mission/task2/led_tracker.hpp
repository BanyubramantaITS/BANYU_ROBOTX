// Counts pipeline light nodes without any position estimate: image-space tracks.
#ifndef BANYU_MISSION_TASK2_LED_TRACKER_HPP_
#define BANYU_MISSION_TASK2_LED_TRACKER_HPP_

#include <cstdint>
#include <vector>

#include <banyu_msgs/msg/light_node.hpp>

namespace banyu_mission::task2 {

class LedTracker {
 public:
  struct Track {
    float u, v;         // last image position, px
    double first_s;     // first sighting
    double seen_s;      // last sighting
    int n{0};
    int red_votes{0}, green_votes{0};
  };

  void reset() { tracks_.clear(); }

  // One frame's detections at time now_s.
  void update(const std::vector<banyu_msgs::msg::LightNode>& lights, double now_s);

  // LightNode colours of the real nodes, in pipe order. A real LED is held for its whole pass
  // under the camera while noise and ring fragments flicker; nodes enter the frame in pipe order.
  std::vector<uint8_t> nodes() const;

  const std::vector<Track>& tracks() const { return tracks_; }

 private:
  std::vector<Track> tracks_;
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_LED_TRACKER_HPP_
