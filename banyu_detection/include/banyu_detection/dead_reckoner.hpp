// Horizontal position with no GPS and no DVL under water: the last GPS-aided position at the surface, then the commanded
// velocity (ENU) integrated over time. The real Pixhawk has no position under water (EKF3 without aiding), and the sim
// EKF keeps a near-perfect one, so the ring order must not rest on it. Only the order and rough spacing are used.
#ifndef BANYU_DETECTION_DEAD_RECKONER_HPP_
#define BANYU_DETECTION_DEAD_RECKONER_HPP_

#include <algorithm>
#include <cmath>

namespace banyu_detection
{

class DeadReckoner
{
public:
  explicit DeadReckoner(double velocity_gain = 1.0, double command_timeout_s = 0.5)
  : gain_(velocity_gain), command_timeout_s_(command_timeout_s) {}

  // Aided position (GPS fix at the surface): taken as is.
  void reset(double x, double y, double t)
  {
    x_ = x;
    y_ = y;
    t_ = t;
    valid_ = true;
  }

  void setCommand(double vx_enu, double vy_enu, double t)
  {
    advance(t);
    vx_ = std::isfinite(vx_enu) ? vx_enu : 0.0;
    vy_ = std::isfinite(vy_enu) ? vy_enu : 0.0;
    command_t_ = t;
  }

  void advance(double t)
  {
    if (!valid_) {return;}
    const double dt = std::clamp(t - t_, 0.0, 1.0);
    if (t - command_t_ <= command_timeout_s_) {
      x_ += gain_ * vx_ * dt;
      y_ += gain_ * vy_ * dt;
    }
    t_ = std::max(t_, t);
  }

  bool valid() const {return valid_;}
  double x() const {return x_;}
  double y() const {return y_;}

private:
  double gain_;
  double command_timeout_s_;
  bool valid_{false};
  double x_{0.0}, y_{0.0}, t_{0.0};
  double vx_{0.0}, vy_{0.0}, command_t_{-1.0e9};
};

}  // namespace banyu_detection

#endif  // BANYU_DETECTION_DEAD_RECKONER_HPP_
