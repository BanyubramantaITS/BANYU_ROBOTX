// Hydrophone-array geometry: pinger bearing from first-arrival time differences. The array only
// resolves azimuth; range and elevation are not observable.
#ifndef BANYU_DETECTION_ACOUSTICS_HPP_
#define BANYU_DETECTION_ACOUSTICS_HPP_

#include <array>
#include <optional>
#include <vector>

namespace banyu_detection
{

constexpr double kSoundSpeed = 1500.0;  // m/s

using Vec3 = std::array<double, 3>;

// Horizontal unit vector toward the pinger, body frame (FLU).
struct PingerVector
{
  double forward, left;
  double bearing() const;  // rad, > 0 = left of the nose
};

// Plane wave: t_i = const - (u . h_i) / c. The array is planar, so only u's horizontal direction is
// used. `hydrophones` are body-frame (FLU) positions matching `arrivals` one to one; nullopt if the
// array is degenerate, sizes differ, or the arrivals carry no direction.
std::optional<PingerVector> pingerVector(
  const std::vector<double> & arrivals,
  const std::vector<Vec3> & hydrophones);

}  // namespace banyu_detection

#endif  // BANYU_DETECTION_ACOUSTICS_HPP_
