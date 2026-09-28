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
    float v_min, v_max;  // rows it crossed: a real node sweeps the frame as the vehicle passes it
    int red_votes{0}, green_votes{0};
    bool confirmed{false};  // a node for certain, however briefly it was seen
    bool entered{true};     // first seen where nodes come into view (see update)
    bool at_start{false};   // already in view when recording (re)started
  };

  // A new recording. Whatever is in view the first moment is the node the survey starts over:
  // only partly seen, it counts on a few frames.
  void reset(double now_s) {
    tracks_.clear();
    start_s_ = now_s;
    resumed_ = false;
  }
  // Recording again after a pause (the repair). Whatever is in view the first moment was seen
  // before the pause and is never reported again.
  void resume(double now_s) {
    start_s_ = now_s;
    resumed_ = true;
  }

  // One frame's detections at time now_s, travelling nose first (direction 1) or tail first (-1)
  // over an image `height` rows tall. New nodes come into view from the travel side; a track
  // that starts anywhere else is a node seen again (re-found at the frame edge, or drifted back
  // over) and is never reported.
  void update(const std::vector<banyu_msgs::msg::LightNode>& lights, double now_s, int direction = -1,
              int height = 240);

  // Mark the track at (u, v) as a real node: the survey stopped on it to repair it, often only
  // a few frames after it came into view.
  void confirm(float u, float v);

  // LightNode colours of the real nodes, in pipe order. A real LED is held for its whole pass
  // under the camera, or sweeps a good part of the frame (the node the survey starts over),
  // while noise and ring fragments flicker in place; nodes enter the frame in pipe order.
  std::vector<uint8_t> nodes() const;

  const std::vector<Track>& tracks() const { return tracks_; }

 private:
  std::vector<Track> tracks_;
  double start_s_{0};
  bool resumed_{false};
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_LED_TRACKER_HPP_
