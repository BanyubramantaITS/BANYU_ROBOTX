// Hydrophone first-arrival times (s, any common offset; ros_gz_interfaces/Float32Array so the sim
// HydrophoneArray plugin bridges straight in) -> banyu_msgs/PingerObservation.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <ros_gz_interfaces/msg/float32_array.hpp>

#include <banyu_msgs/msg/pinger_observation.hpp>

#include "banyu_detection/acoustics.hpp"

class PingerLocalization : public rclcpp::Node {
 public:
  PingerLocalization() : Node("pinger_localization") {
    // Body-frame (FLU) hydrophone positions, x y z per hydrophone. Must match the vehicle
    // (sim: the HydrophoneArray plugin in bluerov2_heavy/model.xacro).
    // ponytail: square array under the hull; set to the real mount.
    const auto flat = declare_parameter<std::vector<double>>(
        "hydrophones", {0.15, 0, -0.1, -0.15, 0, -0.1, 0, 0.15, -0.1, 0, -0.15, -0.1});
    for (size_t i = 0; i + 2 < flat.size(); i += 3) hydrophones_.push_back({flat[i], flat[i + 1], flat[i + 2]});
    // The pinger hangs just under its buoy; its depth is a guess and only sharpens range.
    pinger_depth_ = declare_parameter<double>("pinger_depth", 1.0);

    pub_ = create_publisher<banyu_msgs::msg::PingerObservation>("/banyu/pinger/observation", 10);
    // Depth from the pressure sensor, never rel_alt (EKF output, garbage without horizontal aiding).
    // ponytail: fixed surface pressure and fresh water; sample at the surface on the vehicle.
    pressure_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
        "/mavros/imu/static_pressure", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::FluidPressure::ConstSharedPtr m) { depth_ = (m->fluid_pressure - 101325.0) / 9807.0; });
    arrivals_sub_ = create_subscription<ros_gz_interfaces::msg::Float32Array>(
        "/banyu/pinger/arrivals", 10, [this](ros_gz_interfaces::msg::Float32Array::ConstSharedPtr m) { onArrivals(*m); });
  }

 private:
  void onArrivals(const ros_gz_interfaces::msg::Float32Array& m) {
    const std::vector<double> t(m.data.begin(), m.data.end());
    const auto u = banyu_detection::pingerVector(t, hydrophones_);
    if (!u) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "got %zu arrivals for %zu hydrophones", t.size(),
                           hydrophones_.size());
      return;
    }
    banyu_msgs::msg::PingerObservation obs;
    obs.header.stamp = now();
    obs.forward = u->forward;
    obs.left = u->left;
    obs.bearing = u->bearing();
    obs.elevation_cos = u->elevationCos();
    obs.horizontal_range = banyu_detection::horizontalRange(*u, depth_ - pinger_depth_);
    pub_->publish(obs);
  }

  std::vector<banyu_detection::Vec3> hydrophones_;
  double pinger_depth_{1.0};
  double depth_{0.0};
  rclcpp::Publisher<banyu_msgs::msg::PingerObservation>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Float32Array>::SharedPtr arrivals_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PingerLocalization>());
  rclcpp::shutdown();
  return 0;
}
