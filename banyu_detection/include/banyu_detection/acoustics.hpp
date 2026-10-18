// Hydrophone-array geometry: pinger bearing and range from first-arrival time differences.
#ifndef BANYU_DETECTION_ACOUSTICS_HPP_
#define BANYU_DETECTION_ACOUSTICS_HPP_

#include <array>
#include <optional>
#include <vector>

namespace banyu_detection
{

constexpr double kSoundSpeed = 1500.0;  // m/s

using Vec3 = std::array<double, 3>;

// Horizontal part of the unit vector toward the pinger, body frame (FLU). Its length is
// cos(elevation).
struct PingerVector
{
  double forward, left;
  double bearing() const;       // rad, > 0 = left of the nose
  double elevationCos() const;  // hypot(forward, left)
};

// Plane wave: t_i = const - (u . h_i) / c. The array is planar, so only u's horizontal part is
// observable. `hydrophones` are body-frame (FLU) positions matching `arrivals` one to one;
// nullopt if the array is degenerate or sizes differ.
std::optional<PingerVector> pingerVector(
  const std::vector<double> & arrivals,
  const std::vector<Vec3> & hydrophones);

// Horizontal distance to the pinger from cos(elevation) and the depth difference (clamped to
// 0.5 m so a surface vehicle still gets a finite, if rough, answer).
double horizontalRange(const PingerVector & u, double height_below_pinger);

}  // namespace banyu_detection

#endif  // BANYU_DETECTION_ACOUSTICS_HPP_
