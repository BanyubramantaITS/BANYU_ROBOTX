#include "banyu_mission/task2/led_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace banyu_mission::task2 {

namespace {
constexpr float kGatePx = 40.0f;   // a ring is ~28 px wide at survey altitude; nodes are >= 200 px apart
constexpr double kTrackTtlS = 1.0;
constexpr float kSweepPx = 50.0f;
// Tail first, new nodes appear low in the frame (first sightings in sim: rows 160-237 of 240);
// nodes seen again (re-found at the edge, the repaired node still flashing as the vehicle climbs
// away) come back higher up.
constexpr float kEntryFraction = 0.55f;
constexpr double kStartS = 0.5;         // "in view when recording (re)started"
constexpr int kStartMinFrames = 5;  // re-acquired fragments at the frame edge move ~30 px
}  // namespace

void LedTracker::update(const std::vector<banyu_msgs::msg::LightNode>& lights, double now_s, int direction,
                        int height) {
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
      tracks_.push_back(Track{l.x, l.y, now_s, now_s, 0, l.y, l.y});
      best = &tracks_.back();
      best->at_start = now_s - start_s_ < kStartS;
      best->entered = best->at_start ? !resumed_
                                     : direction < 0 ? l.y >= kEntryFraction * height : l.y <= (1 - kEntryFraction) * height;
    }
    best->u = l.x;
    best->v = l.y;
    best->seen_s = now_s;
    best->v_min = std::min(best->v_min, l.y);
    best->v_max = std::max(best->v_max, l.y);
    ++best->n;
    (l.colour == banyu_msgs::msg::LightNode::RED ? best->red_votes : best->green_votes)++;
  }
}

void LedTracker::confirm(float u, float v) {
  Track* best = nullptr;
  float best_d = kGatePx;
  for (auto& t : tracks_) {
    const float d = std::hypot(t.u - u, t.v - v);
    if (d <= best_d) {
      best = &t;
      best_d = d;
    }
  }
  if (best) best->confirmed = true;
}

std::vector<uint8_t> LedTracker::nodes() const {
  int n_max = 0;
  for (const auto& t : tracks_) n_max = std::max(n_max, t.n);
  const double keep = std::max(10.0, 0.3 * n_max);

  std::vector<const Track*> kept;
  for (const auto& t : tracks_)
    if (t.confirmed ||
        (t.entered && (t.at_start ? t.n >= kStartMinFrames : t.n >= keep || t.v_max - t.v_min >= kSweepPx)))
      kept.push_back(&t);
  std::sort(kept.begin(), kept.end(), [](const Track* a, const Track* b) { return a->first_s < b->first_s; });

  std::vector<uint8_t> out;
  for (const auto* t : kept)
    out.push_back(t->red_votes > t->green_votes ? banyu_msgs::msg::LightNode::RED : banyu_msgs::msg::LightNode::GREEN);
  return out;
}

}  // namespace banyu_mission::task2
