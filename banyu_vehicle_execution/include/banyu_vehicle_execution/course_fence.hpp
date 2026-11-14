#ifndef BANYU_VEHICLE_EXECUTION__COURSE_FENCE_HPP_
#define BANYU_VEHICLE_EXECUTION__COURSE_FENCE_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "banyu_vehicle_execution/task4_zones.hpp"

namespace banyu_vehicle_execution
{

// Course boundary from the GCS (rx_msgs/Course.boundary): a closed polygon the surfaced vehicle must stay inside,
// `margin_m` from its edge. ArduSub's fence only logs a breach, so this is the only enforcement.
struct CourseFence
{
  std::vector<double> latitude_deg;
  std::vector<double> longitude_deg;
  // GPS error plus vehicle size plus drift: the vehicle and every surface goal stay this far inside the boundary.
  double margin_m{3.0};
  // Outward speed fades to zero over this band outside the margin.
  double brake_m{3.0};
  // Inward speed when inside the margin band (or outside the boundary).
  double return_speed_mps{0.4};

  bool any() const {return std::min(latitude_deg.size(), longitude_deg.size()) >= 3;}
};

// Polygon vertices as ENU offsets from (ref_lat, ref_lon), without a repeated closing point.
inline std::vector<Vec2> fenceAround(const CourseFence & fence, double ref_lat, double ref_lon)
{
  std::vector<Vec2> poly;
  if (!fence.any()) {return poly;}
  const std::size_t count = std::min(fence.latitude_deg.size(), fence.longitude_deg.size());
  for (std::size_t i = 0; i < count; ++i) {
    poly.push_back(enuOffset(ref_lat, ref_lon, fence.latitude_deg[i], fence.longitude_deg[i]));
  }
  if (poly.size() > 1 && length(poly.front() - poly.back()) < 1e-3) {poly.pop_back();}
  return poly;
}

inline bool insidePolygon(Vec2 p, const std::vector<Vec2> & poly)
{
  bool inside = false;
  for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const Vec2 a = poly[i];
    const Vec2 b = poly[j];
    if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) {inside = !inside;}
  }
  return inside;
}

// Distance from `p` to the boundary, positive inside and negative outside, and the unit direction into the course.
struct FenceDistance
{
  double clearance_m{0.0};
  Vec2 inward{0.0, 0.0};
};

inline FenceDistance fenceDistance(Vec2 p, const std::vector<Vec2> & poly)
{
  double area2 = 0.0;
  for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    area2 += poly[j].x * poly[i].y - poly[i].x * poly[j].y;
  }
  const double side = area2 >= 0.0 ? 1.0 : -1.0;   // interior on the left of each edge when counter-clockwise
  double best = std::numeric_limits<double>::infinity();
  Vec2 nearest{};
  Vec2 edge_normal{};
  for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const Vec2 q = closestOnSegment(p, poly[j], poly[i]);
    const double d = length(p - q);
    if (d < best) {
      best = d;
      nearest = q;
      const Vec2 e = poly[i] - poly[j];
      const double len = std::max(length(e), 1e-9);
      edge_normal = Vec2{-e.y / len, e.x / len} * side;
    }
  }
  const bool inside = insidePolygon(p, poly);
  Vec2 inward = edge_normal;
  if (best > 1e-6) {inward = (inside ? p - nearest : nearest - p) * (1.0 / best);}
  return FenceDistance{inside ? best : -best, inward};
}

// `goal` moved to be at least `margin_m` inside the polygon (unchanged when it already is).
inline Vec2 pullInside(Vec2 goal, const std::vector<Vec2> & poly, double margin_m)
{
  Vec2 g = goal;
  for (int i = 0; i < 20; ++i) {
    const auto d = fenceDistance(g, poly);
    if (d.clearance_m >= margin_m - 1e-3) {break;}
    g = g + d.inward * (margin_m - d.clearance_m + 0.01);
  }
  return g;
}

// Horizontal ENU velocity limited by the fence: the outward part fades out over the brake band and is removed inside
// the margin, where an inward push is added. The part along the edge is kept, so the vehicle can slide along it.
inline Vec2 fenceVelocity(Vec2 v, const FenceDistance & d, const CourseFence & fence)
{
  const double toward_inside = dot(v, d.inward);
  double limited = toward_inside;
  if (d.clearance_m < fence.margin_m) {
    const double push = std::clamp(0.5 * (fence.margin_m - d.clearance_m), 0.2, std::max(0.2, fence.return_speed_mps));
    limited = std::max(toward_inside, push);
  } else if (toward_inside < 0.0) {
    const double scale = fence.brake_m > 0.0 ?
      std::clamp((d.clearance_m - fence.margin_m) / fence.brake_m, 0.0, 1.0) : 1.0;
    limited = toward_inside * scale;
  }
  return v + d.inward * (limited - toward_inside);
}

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__COURSE_FENCE_HPP_
