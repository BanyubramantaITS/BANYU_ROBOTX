#ifndef BANYU_VEHICLE_EXECUTION__TASK4_ZONES_HPP_
#define BANYU_VEHICLE_EXECUTION__TASK4_ZONES_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace banyu_vehicle_execution
{

// Task 4 keep-out geometry, planar ENU metres relative to the vehicle unless a function says otherwise.
// A zone is a capsule: the points within `radius` of the segment a-b. A static zone has a == b; a moving object
// is the segment it sweeps from its predicted position now to where it will be after the projection horizon.
struct Vec2
{
  double x{0.0};
  double y{0.0};
};

struct Capsule
{
  Vec2 a;
  Vec2 b;
  double radius{0.0};
  bool moving{false};
};

inline double dot(Vec2 p, Vec2 q) {return p.x * q.x + p.y * q.y;}
inline Vec2 operator+(Vec2 p, Vec2 q) {return {p.x + q.x, p.y + q.y};}
inline Vec2 operator-(Vec2 p, Vec2 q) {return {p.x - q.x, p.y - q.y};}
inline Vec2 operator*(Vec2 p, double s) {return {p.x * s, p.y * s};}
inline double length(Vec2 p) {return std::hypot(p.x, p.y);}

// Closest point to `p` on the segment a-b.
inline Vec2 closestOnSegment(Vec2 p, Vec2 a, Vec2 b)
{
  const Vec2 ab = b - a;
  const double len2 = dot(ab, ab);
  if (len2 < 1e-12) {return a;}
  const double t = std::clamp(dot(p - a, ab) / len2, 0.0, 1.0);
  return a + ab * t;
}

// Signed distance to the zone boundary: negative inside.
inline double signedDistance(Vec2 p, const Capsule & zone)
{
  return length(p - closestOnSegment(p, zone.a, zone.b)) - zone.radius;
}

inline bool insideAny(Vec2 p, const std::vector<Capsule> & zones)
{
  return std::any_of(zones.begin(), zones.end(), [&](const Capsule & z) {return signedDistance(p, z) < 0.0;});
}

// Smallest distance between the segments p-q and a-b.
inline double segmentSegmentDistance(Vec2 p, Vec2 q, Vec2 a, Vec2 b)
{
  const auto cross = [](Vec2 u, Vec2 v) {return u.x * v.y - u.y * v.x;};
  const Vec2 r = q - p;
  const Vec2 s = b - a;
  const double denom = cross(r, s);
  if (std::abs(denom) > 1e-12) {
    const double t = cross(a - p, s) / denom;
    const double u = cross(a - p, r) / denom;
    if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {return 0.0;}
  }
  return std::min(
    {length(p - closestOnSegment(p, a, b)), length(q - closestOnSegment(q, a, b)),
      length(a - closestOnSegment(a, p, q)), length(b - closestOnSegment(b, p, q))});
}

// True when the straight segment p-q passes through the zone.
inline bool segmentCrossesZone(Vec2 p, Vec2 q, const Capsule & zone)
{
  return segmentSegmentDistance(p, q, zone.a, zone.b) < zone.radius;
}

inline bool segmentClear(Vec2 p, Vec2 q, const std::vector<Capsule> & zones)
{
  return std::none_of(zones.begin(), zones.end(), [&](const Capsule & z) {return segmentCrossesZone(p, q, z);});
}

// Unit vector pointing out of the deepest zone containing `p`, with the distance to travel to be `clearance` outside it.
struct Exit
{
  Vec2 direction;
  double distance_m{0.0};
};

inline std::optional<Exit> exitVector(Vec2 p, const std::vector<Capsule> & zones, double clearance_m)
{
  const Capsule * deepest = nullptr;
  double depth = 0.0;
  for (const auto & z : zones) {
    const double d = signedDistance(p, z);
    if (d < depth) {
      depth = d;
      deepest = &z;
    }
  }
  if (!deepest) {return std::nullopt;}
  const Vec2 away = p - closestOnSegment(p, deepest->a, deepest->b);
  const double len = length(away);
  Vec2 direction{1.0, 0.0};
  if (len > 1e-6) {
    direction = away * (1.0 / len);
  } else {
    // On the axis of a moving zone (or the centre of a static one): leave sideways to its travel.
    const Vec2 axis = deepest->b - deepest->a;
    const double axis_len = length(axis);
    if (axis_len > 1e-6) {direction = Vec2{-axis.y / axis_len, axis.x / axis_len};}
  }
  return Exit{direction, -depth + clearance_m};
}

// Where to steer to reach `goal` from the origin without entering a zone: the goal when the straight line is clear, else the
// first waypoint of the shortest path through points around the zone ends and flanks (1.25 x radius out). Empty when the
// goal cannot be reached around the zones.
inline std::optional<Vec2> detourAim(Vec2 goal, const std::vector<Capsule> & zones)
{
  const Vec2 origin{0.0, 0.0};
  if (segmentClear(origin, goal, zones)) {return goal;}
  constexpr double kFlank = 1.25;
  const Vec2 path = goal - origin;
  const double path_len = length(path);
  const Vec2 path_normal = path_len > 1e-6 ? Vec2{-path.y / path_len, path.x / path_len} : Vec2{0.0, 1.0};
  // Node 0 is the origin; the goal is the last node.
  std::vector<Vec2> nodes{origin};
  for (const auto & z : zones) {
    const Vec2 axis = z.b - z.a;
    const double axis_len = length(axis);
    const Vec2 axis_dir = axis_len > 1e-6 ? axis * (1.0 / axis_len) : path_normal * -1.0;
    const Vec2 normal{-axis_dir.y, axis_dir.x};
    const double r = z.radius * kFlank;
    for (const double side : {-1.0, 1.0}) {
      for (const Vec2 c : {z.a + normal * (side * r), z.b + normal * (side * r),
          z.a - axis_dir * r + normal * (side * r), z.b + axis_dir * r + normal * (side * r),
          closestOnSegment(origin + path * 0.5, z.a, z.b) + path_normal * (side * r)})
      {
        if (!insideAny(c, zones)) {nodes.push_back(c);}
      }
    }
  }
  nodes.push_back(goal);
  const std::size_t n = nodes.size();
  std::vector<double> dist(n, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> prev(n, n);
  std::vector<bool> done(n, false);
  dist[0] = 0.0;
  for (std::size_t round = 0; round < n; ++round) {
    std::size_t u = n;
    for (std::size_t i = 0; i < n; ++i) {
      if (!done[i] && (u == n || dist[i] < dist[u])) {u = i;}
    }
    if (u == n || !std::isfinite(dist[u])) {break;}
    done[u] = true;
    for (std::size_t v = 0; v < n; ++v) {
      if (done[v] || !segmentClear(nodes[u], nodes[v], zones)) {continue;}
      const double d = dist[u] + length(nodes[v] - nodes[u]);
      if (d < dist[v]) {
        dist[v] = d;
        prev[v] = u;
      }
    }
  }
  if (!std::isfinite(dist[n - 1])) {return std::nullopt;}
  std::size_t step = n - 1;
  while (prev[step] != 0 && prev[step] != n) {step = prev[step];}
  return nodes[step];
}

// Dive point inside a static zone: the nearest point outside every zone and at least `buoy_clear_m` from each buoy,
// first along the buoy line (when given), then anywhere (rings around the goal). Unchanged when already outside.
struct DivePointShift
{
  Vec2 point;
  bool shifted{false};
  bool along_line{false};
};

inline std::optional<DivePointShift> shiftDivePoint(
  Vec2 goal, const std::vector<Capsule> & zones, const std::vector<Vec2> & buoys, double buoy_clear_m,
  std::optional<std::pair<Vec2, Vec2>> buoy_line)
{
  constexpr double kEps = 0.05;
  if (!insideAny(goal, zones)) {return DivePointShift{goal, false, false};}
  const auto valid = [&](Vec2 p) {
      for (const auto & z : zones) {
        if (signedDistance(p, z) < kEps) {return false;}
      }
      return std::none_of(buoys.begin(), buoys.end(), [&](Vec2 b) {return length(p - b) < buoy_clear_m;});
    };
  const auto nearest = [&](const std::vector<Vec2> & points) -> std::optional<Vec2> {
      std::optional<Vec2> best;
      double best_d = std::numeric_limits<double>::infinity();
      for (const auto & p : points) {
        const double d = length(p - goal);
        if (valid(p) && d < best_d) {
          best_d = d;
          best = p;
        }
      }
      return best;
    };
  if (buoy_line) {
    const Vec2 dir = buoy_line->second - buoy_line->first;
    const double len = length(dir);
    if (len > 1e-6) {
      const Vec2 u = dir * (1.0 / len);
      std::vector<Vec2> points;
      for (const auto & z : zones) {
        // Line parameter s from the goal where it meets the inflated circle of the zone's closest axis point.
        const Vec2 c = closestOnSegment(goal, z.a, z.b);
        const double along = dot(c - goal, u);
        const Vec2 foot = goal + u * along;
        const double offset = length(c - foot);
        const double r = z.radius + 2.0 * kEps;
        if (offset < r) {
          const double half = std::sqrt(r * r - offset * offset);
          points.push_back(goal + u * (along - half));
          points.push_back(goal + u * (along + half));
        }
      }
      if (const auto p = nearest(points)) {return DivePointShift{*p, true, true};}
    }
  }
  for (double ring = 0.5; ring <= 80.0; ring += 0.5) {
    std::vector<Vec2> points;
    for (int k = 0; k < 24; ++k) {
      const double ang = k * (M_PI / 12.0);
      points.push_back(goal + Vec2{std::cos(ang), std::sin(ang)} * ring);
    }
    if (const auto p = nearest(points)) {return DivePointShift{*p, true, false};}
  }
  return std::nullopt;
}

// First point on the segment origin-goal where it enters a zone (backed off by `standoff_m`), or nullopt when clear.
inline std::optional<Vec2> truncateAtZone(Vec2 goal, const std::vector<Capsule> & zones, double standoff_m)
{
  const double len = length(goal);
  if (len < 1e-6) {return std::nullopt;}
  const Vec2 u = goal * (1.0 / len);
  for (double s = 0.0; s <= len; s += 0.1) {
    if (insideAny(u * s, zones)) {return u * std::max(0.0, s - standoff_m);}
  }
  if (insideAny(goal, zones)) {return u * std::max(0.0, len - standoff_m);}
  return std::nullopt;
}

// Raw Task 4 zone state as published on /banyu/task4/zones (lat/lon), turned into capsules around a reference position.
struct MovingObject
{
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double heading_enu_rad{0.0};
  double speed_mps{0.0};
  double radius_m{0.0};
  double age_s{0.0};
};

// Position of the object `dt` seconds after its alert, planar ENU offset from its alert position.
inline Vec2 projectMoving(const MovingObject & object, double dt)
{
  return Vec2{std::cos(object.heading_enu_rad), std::sin(object.heading_enu_rad)} * (object.speed_mps * dt);
}

struct ZoneField
{
  bool active{false};
  std::vector<double> static_latitude_deg;
  std::vector<double> static_longitude_deg;
  std::vector<double> static_radius_m;
  std::optional<MovingObject> moving;
  // GPS error plus vehicle size, added to every radius.
  double margin_m{2.0};
  // How far ahead the moving object's travel is kept clear of.
  double horizon_s{30.0};
  // Leave a zone to this distance outside it.
  double exit_clearance_m{0.5};
  double exit_speed_mps{0.6};

  bool any() const {return active && (!static_latitude_deg.empty() || moving.has_value());}
};

// Planar offset (east, north) of a point from a reference, same flat-earth model as latLonToEnuOffsets.
inline Vec2 enuOffset(double ref_lat, double ref_lon, double lat, double lon)
{
  constexpr double kEarthRadius = 6378137.0;
  constexpr double kDegToRad = M_PI / 180.0;
  return Vec2{(lon - ref_lon) * kDegToRad * std::cos((lat + ref_lat) * 0.5 * kDegToRad) * kEarthRadius,
    (lat - ref_lat) * kDegToRad * kEarthRadius};
}

// The zones as capsules relative to (ref_lat, ref_lon). The moving object is predicted by its age and swept over the
// horizon, so both where it is and where it is heading are kept clear of.
inline std::vector<Capsule> zonesAround(const ZoneField & field, double ref_lat, double ref_lon, double extra_age_s = 0.0)
{
  std::vector<Capsule> zones;
  if (!field.any()) {return zones;}
  const std::size_t count = std::min({field.static_latitude_deg.size(), field.static_longitude_deg.size(),
        field.static_radius_m.size()});
  for (std::size_t i = 0; i < count; ++i) {
    const Vec2 c = enuOffset(ref_lat, ref_lon, field.static_latitude_deg[i], field.static_longitude_deg[i]);
    zones.push_back(Capsule{c, c, field.static_radius_m[i] + field.margin_m, false});
  }
  if (field.moving) {
    const auto & m = *field.moving;
    const Vec2 origin = enuOffset(ref_lat, ref_lon, m.latitude_deg, m.longitude_deg);
    const double age = std::max(0.0, m.age_s + extra_age_s);
    zones.push_back(Capsule{origin + projectMoving(m, age), origin + projectMoving(m, age + field.horizon_s),
        m.radius_m + field.margin_m, true});
  }
  return zones;
}

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__TASK4_ZONES_HPP_
