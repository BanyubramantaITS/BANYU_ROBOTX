#include "banyu_mission/task2/led_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace banyu_mission::task2 {

namespace {
constexpr float kGatePx = 40.0f;   // a ring is ~28 px wide at survey altitude; nodes are >= 200 px apart
constexpr double kTrackTtlS = 1.0;
}  // namespace

void LedTracker::update(const std::vector<banyu_msgs::msg::LightNode>& lights, double now_s) {
  for (const auto& l : lights) {
    Track* best = nullptr;
    float best_d = kGatePx;
    for (auto& t : tracks_) {
      if (now_s - t.seen_s >= kTrackTtlS) continue;
      const float d = std::hypot(t.u - l.x, t.v - l.y);
      if (d <= best_d) {
        best = &t;
        best_d = d;
      }
    }
    if (best == nullptr) {
      tracks_.push_back(Track{l.x, l.y, now_s, now_s});
      best = &tracks_.back();
    }
    best->u = l.x;
    best->v = l.y;
    best->seen_s = now_s;
    ++best->n;
    (l.colour == banyu_msgs::msg::LightNode::RED ? best->red_votes : best->green_votes)++;
  }
}

std::vector<uint8_t> LedTracker::nodes() const {
  int n_max = 0;
  for (const auto& t : tracks_) n_max = std::max(n_max, t.n);
  const double keep = std::max(10.0, 0.3 * n_max);

  std::vector<const Track*> kept;
  for (const auto& t : tracks_)
    if (t.n >= keep) kept.push_back(&t);
  std::sort(kept.begin(), kept.end(), [](const Track* a, const Track* b) { return a->first_s < b->first_s; });

  std::vector<uint8_t> out;
  for (const auto* t : kept)
    out.push_back(t->red_votes > t->green_votes ? banyu_msgs::msg::LightNode::RED : banyu_msgs::msg::LightNode::GREEN);
  return out;
}

}  // namespace banyu_mission::task2
