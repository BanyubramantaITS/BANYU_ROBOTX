// Acoustic shadowing by the vehicle's hull, for HydrophoneArray. The hull is a set of solid boxes
// in the body frame; a hydrophone whose straight line to the pinger crosses one hears the wave
// diffracted around the edges instead: the first arrival follows the shortest path over the box
// edges, found on a visibility graph of points sampled along them.
// ponytail: boxes only, edges sampled at 1/6 of their length (a detour can come out up to half a
// sample spacing long). Use the mesh itself if the hull ever needs to be more than tubes and foam.
#ifndef BANYU_SIMULATION_ACOUSTIC_SHADOW_HPP_
#define BANYU_SIMULATION_ACOUSTIC_SHADOW_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <gz/math/Vector3.hh>

namespace banyu
{
struct Box
{
  gz::math::Vector3d lo, hi;
};

// True when segment p-q passes through the inside of the box (grazing a face or edge is clear).
inline bool SegmentHits(const gz::math::Vector3d &p, const gz::math::Vector3d &q, const Box &b)
{
  constexpr double kEps = 1e-4;
  double t0 = 0.0, t1 = 1.0;
  for (int i = 0; i < 3; ++i) {
    const double d = q[i] - p[i];
    const double lo = b.lo[i] + kEps, hi = b.hi[i] - kEps;
    if (std::abs(d) < 1e-12) {
      if (p[i] <= lo || p[i] >= hi)
        return false;
      continue;
    }
    double a = (lo - p[i]) / d, c = (hi - p[i]) / d;
    if (a > c)
      std::swap(a, c);
    t0 = std::max(t0, a);
    t1 = std::min(t1, c);
    if (t0 >= t1)
      return false;
  }
  return true;
}

class ShadowPaths
{
public:
  ShadowPaths() = default;

  // Receivers and boxes are fixed in the body frame, so everything that does not involve the
  // source is solved here once: the shortest path from each receiver to every edge point.
  ShadowPaths(std::vector<Box> boxes, const std::vector<gz::math::Vector3d> &receivers, int samples_per_edge = 6)
    : boxes_(std::move(boxes)), receivers_(receivers)
  {
    for (const auto &b : boxes_) {
      auto corner = [&b](int m) {
        return gz::math::Vector3d(m & 1 ? b.hi.X() : b.lo.X(), m & 2 ? b.hi.Y() : b.lo.Y(), m & 4 ? b.hi.Z() : b.lo.Z());
      };
      for (int m = 0; m < 8; ++m)
        for (int bit : {1, 2, 4})
          if (!(m & bit))
            for (int s = 0; s <= samples_per_edge; ++s)
              AddNode(corner(m) + (corner(m | bit) - corner(m)) * (static_cast<double>(s) / samples_per_edge));
    }

    const size_t n = nodes_.size();
    for (const auto &r : receivers_) {
      // Dense Dijkstra from the receiver; a few hundred nodes, run once at load.
      std::vector<double> dist(n, kInf);
      std::vector<bool> done(n, false);
      for (size_t j = 0; j < n; ++j)
        if (!Blocked(r, nodes_[j]))
          dist[j] = r.Distance(nodes_[j]);
      for (size_t k = 0; k < n; ++k) {
        size_t i = n;
        for (size_t j = 0; j < n; ++j)
          if (!done[j] && (i == n || dist[j] < dist[i]))
            i = j;
        if (i == n || dist[i] == kInf)
          break;
        done[i] = true;
        for (size_t j = 0; j < n; ++j) {
          const double nd = dist[i] + nodes_[i].Distance(nodes_[j]);
          if (!done[j] && nd < dist[j] && !Blocked(nodes_[i], nodes_[j]))
            dist[j] = nd;
        }
      }
      to_receiver_.push_back(std::move(dist));
    }
  }

  // First-arrival path length (m) from `source` to receiver i, both in the body frame.
  double Length(const gz::math::Vector3d &source, size_t i, bool *shadowed) const
  {
    const auto &r = receivers_[i];
    *shadowed = Blocked(source, r);
    if (!*shadowed)
      return source.Distance(r);
    double best = kInf;
    for (size_t j = 0; j < nodes_.size(); ++j)
      if (to_receiver_[i][j] < best && !Blocked(source, nodes_[j]))
        best = std::min(best, to_receiver_[i][j] + source.Distance(nodes_[j]));
    return best;
  }

private:
  static constexpr double kInf = std::numeric_limits<double>::infinity();

  bool Blocked(const gz::math::Vector3d &p, const gz::math::Vector3d &q) const
  {
    return std::any_of(boxes_.begin(), boxes_.end(), [&](const Box &b) { return SegmentHits(p, q, b); });
  }

  // Edge points buried inside another box can never be on a path.
  void AddNode(const gz::math::Vector3d &p)
  {
    for (const auto &b : boxes_)
      if (p.X() > b.lo.X() && p.X() < b.hi.X() && p.Y() > b.lo.Y() && p.Y() < b.hi.Y() && p.Z() > b.lo.Z() &&
          p.Z() < b.hi.Z())
        return;
    nodes_.push_back(p);
  }

  std::vector<Box> boxes_;
  std::vector<gz::math::Vector3d> receivers_;
  std::vector<gz::math::Vector3d> nodes_;
  std::vector<std::vector<double>> to_receiver_;
};
}  // namespace banyu

#endif  // BANYU_SIMULATION_ACOUSTIC_SHADOW_HPP_
