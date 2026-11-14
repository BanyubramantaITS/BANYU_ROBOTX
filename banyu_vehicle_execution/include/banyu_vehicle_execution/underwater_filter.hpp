#ifndef BANYU_VEHICLE_EXECUTION__UNDERWATER_FILTER_HPP_
#define BANYU_VEHICLE_EXECUTION__UNDERWATER_FILTER_HPP_

#include <cmath>

namespace banyu_vehicle_execution
{

// Planar position filter for a dive without DVL: commanded-velocity dead reckoning corrected by
// bearings to a pinger at a known point. State is (x, y), ENU metres from the dive origin.
class UnderwaterFilter
{
public:
  struct Params
  {
    double velocity_gain{1.0};       // achieved / commanded speed, calibrated in the water
    double speed_error_fraction{0.1};  // 1-sigma of the achieved speed, fraction of speed
    double drift_rate_mps{0.02};     // unmodelled drift (current), 1-sigma m/s
    double bearing_sigma_rad{0.17};  // hydrophone bearing 1-sigma
    double min_range_m{1.0};         // skip bearings from closer than this (geometry blows up)
    double gate_sigma{3.0};          // innovation gate in sigmas
  };

  explicit UnderwaterFilter(const Params & p)
  : p_(p) {}

  void reset(double initial_sigma_m)
  {
    x_ = y_ = 0.0;
    pxx_ = pyy_ = initial_sigma_m * initial_sigma_m;
    pxy_ = 0.0;
  }

  // Advance by commanded ENU velocity (m/s) over dt (s).
  void predict(double vx_cmd, double vy_cmd, double dt)
  {
    x_ += p_.velocity_gain * vx_cmd * dt;
    y_ += p_.velocity_gain * vy_cmd * dt;
    const double q = p_.drift_rate_mps * p_.drift_rate_mps * dt * dt;
    const double sx = p_.speed_error_fraction * p_.velocity_gain * vx_cmd * dt;
    const double sy = p_.speed_error_fraction * p_.velocity_gain * vy_cmd * dt;
    pxx_ += q + sx * sx;
    pyy_ += q + sy * sy;
    pxy_ += sx * sy;
  }

  // `world_bearing`: ENU azimuth (CCW from east) from the vehicle toward the pinger at (px, py).
  // Returns true if the bearing was fused.
  bool updateBearing(double world_bearing, double px, double py)
  {
    const double dx = px - x_;
    const double dy = py - y_;
    const double r2 = dx * dx + dy * dy;
    if (r2 < p_.min_range_m * p_.min_range_m) {return false;}

    const double innovation = wrap(world_bearing - std::atan2(dy, dx));
    // H = d(bearing)/d(x, y)
    const double hx = dy / r2;
    const double hy = -dx / r2;
    const double phx = pxx_ * hx + pxy_ * hy;
    const double phy = pxy_ * hx + pyy_ * hy;
    const double s = hx * phx + hy * phy + p_.bearing_sigma_rad * p_.bearing_sigma_rad;
    if (innovation * innovation > p_.gate_sigma * p_.gate_sigma * s) {return false;}

    const double kx = phx / s;
    const double ky = phy / s;
    x_ += kx * innovation;
    y_ += ky * innovation;
    // P = (I - K H) P
    const double nxx = pxx_ - kx * phx;
    const double nxy = pxy_ - kx * phy;
    const double nyy = pyy_ - ky * phy;
    pxx_ = nxx;
    pxy_ = nxy;
    pyy_ = nyy;
    return true;
  }

  double x() const {return x_;}
  double y() const {return y_;}
  double covXX() const {return pxx_;}
  double covXY() const {return pxy_;}
  double covYY() const {return pyy_;}

  static double wrap(double a)
  {
    while (a > M_PI) {a -= 2.0 * M_PI;}
    while (a < -M_PI) {a += 2.0 * M_PI;}
    return a;
  }

private:
  Params p_;
  double x_{0.0}, y_{0.0};
  double pxx_{1.0}, pyy_{1.0}, pxy_{0.0};
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__UNDERWATER_FILTER_HPP_
