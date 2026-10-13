#include "banyu_detection/acoustics.hpp"

#include <algorithm>
#include <cmath>

namespace banyu_detection {

double PingerVector::bearing() const { return std::atan2(left, forward); }
double PingerVector::elevationCos() const { return std::hypot(forward, left); }

std::optional<PingerVector> pingerVector(const std::vector<double>& arrivals, const std::vector<Vec3>& hydrophones) {
  const size_t n = hydrophones.size();
  if (n < 3 || arrivals.size() != n) return std::nullopt;

  double mx = 0, my = 0, mt = 0;
  for (size_t i = 0; i < n; ++i) {
    mx += hydrophones[i][0];
    my += hydrophones[i][1];
    mt += arrivals[i];
  }
  mx /= n;
  my /= n;
  mt /= n;

  // Least squares for u in  -(h_i - h_mean) . u / c = t_i - t_mean  (normal equations, 2x2).
  double sxx = 0, sxy = 0, syy = 0, sxt = 0, syt = 0;
  for (size_t i = 0; i < n; ++i) {
    const double x = hydrophones[i][0] - mx, y = hydrophones[i][1] - my, t = arrivals[i] - mt;
    sxx += x * x;
    sxy += x * y;
    syy += y * y;
    sxt += x * t;
    syt += y * t;
  }
  const double det = sxx * syy - sxy * sxy;
  if (std::fabs(det) < 1e-12) return std::nullopt;
  const double ux = (syy * sxt - sxy * syt) / det;
  const double uy = (sxx * syt - sxy * sxt) / det;
  return PingerVector{-kSoundSpeed * ux, -kSoundSpeed * uy};
}

double horizontalRange(const PingerVector& u, double height_below_pinger) {
  const double c = std::min(u.elevationCos(), 0.999);
  return std::max(std::fabs(height_below_pinger), 0.5) * c / std::sqrt(1 - c * c);
}

}  // namespace banyu_detection
