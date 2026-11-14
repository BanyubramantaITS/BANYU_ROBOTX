#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "banyu_msgs/msg/pinger_observation.hpp"
#include "banyu_msgs/msg/task2_buoy_target.hpp"
#include "banyu_msgs/msg/water_depth.hpp"
#include "banyu_vehicle_execution/underwater_filter.hpp"
#include "banyu_vehicle_execution/vehicle_telemetry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace banyu_vehicle_execution
{

// Submerged odometry without a DVL. The origin (0, 0) is the last GPS fix before the dive;
// motion is dead-reckoned from the commanded velocity and corrected by pinger bearings.
class UnderwaterOdometryNode : public rclcpp::Node
{
public:
  UnderwaterOdometryNode()
  : Node("underwater_odometry_node"),
    filter_(loadFilterParams())
  {
    dive_depth_m_ = declare_parameter<double>("dive_depth_m", 0.5);
    surface_depth_m_ = declare_parameter<double>("surface_depth_m", 0.2);
    gps_sigma_m_ = declare_parameter<double>("gps_sigma_m", 2.0);
    setpoint_timeout_s_ = declare_parameter<double>("setpoint_timeout_s", 0.5);
    min_path_step_m_ = declare_parameter<double>("min_path_step_m", 0.05);
    max_poses_ = declare_parameter<int>("max_poses", 5000);
    frame_id_ = declare_parameter<std::string>("frame_id", "dive_origin");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
    // Pinger sits on the active buoy unless a fixed position is configured.
    pinger_lat_ = declare_parameter<double>("pinger.latitude", std::nan(""));
    pinger_lon_ = declare_parameter<double>("pinger.longitude", std::nan(""));

    const auto pose_topic = declare_parameter<std::string>(
      "topics.pose", "/mavros/local_position/pose");
    const auto fix_topic = declare_parameter<std::string>(
      "topics.gps", "/mavros/global_position/global");
    const auto depth_topic = declare_parameter<std::string>(
      "topics.water_depth", "/banyu/vehicle/water_depth");
    const auto setpoint_topic = declare_parameter<std::string>(
      "topics.setpoint", "/mavros/setpoint_raw/local");
    const auto pinger_topic = declare_parameter<std::string>(
      "topics.pinger", "/banyu/pinger/observation");
    const auto buoy_topic = declare_parameter<std::string>(
      "topics.buoy_target", "/banyu/task2/buoy_target");
    const auto odom_topic = declare_parameter<std::string>(
      "topics.odometry", "/banyu/dive/odometry");
    const auto path_topic = declare_parameter<std::string>(
      "topics.path", "/banyu/dive/path");

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, rclcpp::QoS(1).reliable());

    depth_sub_ = create_subscription<banyu_msgs::msg::WaterDepth>(
      depth_topic, 10, [this](banyu_msgs::msg::WaterDepth::ConstSharedPtr m) {
        if (m->valid) {depth_m_ = m->depth_m; depth_valid_ = true;}
      });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
        const auto & q = m->pose.orientation;
        yaw_ = getYawFromQuaternion(q.x, q.y, q.z, q.w);
        orientation_ = q;
        yaw_valid_ = true;
      });
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      fix_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) {
        if (!diving_ && std::isfinite(m->latitude) && std::isfinite(m->longitude)) {
          surface_fix_ = *m;
          has_fix_ = true;
        }
      });
    buoy_sub_ = create_subscription<banyu_msgs::msg::Task2BuoyTarget>(
      buoy_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](banyu_msgs::msg::Task2BuoyTarget::ConstSharedPtr m) {
        if (!diving_ && !std::isfinite(pinger_lat_param_())) {
          buoy_lat_ = m->latitude;
          buoy_lon_ = m->longitude;
          has_buoy_ = true;
        }
      });
    setpoint_sub_ = create_subscription<mavros_msgs::msg::PositionTarget>(
      setpoint_topic, 10, [this](mavros_msgs::msg::PositionTarget::ConstSharedPtr m) {
        cmd_vx_ = m->velocity.x;
        cmd_vy_ = m->velocity.y;
        last_setpoint_ = now();
        has_setpoint_ = true;
      });
    // No hydrophone on the competition vehicle: bearing corrections only when opted in.
    use_hydrophone_ = declare_parameter<bool>("use_hydrophone", false);
    if (use_hydrophone_) {
      pinger_sub_ = create_subscription<banyu_msgs::msg::PingerObservation>(
        pinger_topic, 10, std::bind(
          &UnderwaterOdometryNode::handlePinger, this, std::placeholders::_1));
    }

    last_tick_ = now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&UnderwaterOdometryNode::tick, this));
  }

private:
  UnderwaterFilter::Params loadFilterParams()
  {
    UnderwaterFilter::Params p;
    p.velocity_gain = declare_parameter<double>("velocity_gain", p.velocity_gain);
    p.speed_error_fraction = declare_parameter<double>(
      "speed_error_fraction", p.speed_error_fraction);
    p.drift_rate_mps = declare_parameter<double>("drift_rate_mps", p.drift_rate_mps);
    p.bearing_sigma_rad = declare_parameter<double>("bearing_sigma_rad", p.bearing_sigma_rad);
    p.min_range_m = declare_parameter<double>("min_range_m", p.min_range_m);
    p.gate_sigma = declare_parameter<double>("gate_sigma", p.gate_sigma);
    return p;
  }

  double pinger_lat_param_() const {return pinger_lat_;}

  void handlePinger(const banyu_msgs::msg::PingerObservation::ConstSharedPtr msg)
  {
    if (!diving_ || !yaw_valid_ || !has_pinger_xy_ || !std::isfinite(msg->bearing)) {return;}
    // Body bearing is CCW from the nose; yaw is CCW from east, so they add.
    filter_.updateBearing(
      UnderwaterFilter::wrap(yaw_ + msg->bearing), pinger_x_, pinger_y_);
  }

  void tick()
  {
    const auto t = now();
    const double dt = std::clamp((t - last_tick_).seconds(), 0.0, 1.0);
    last_tick_ = t;
    if (!depth_valid_) {return;}

    if (!diving_) {
      if (depth_m_ > dive_depth_m_ && has_fix_) {startDive();}
      return;
    }
    if (depth_m_ < surface_depth_m_) {
      diving_ = false;
      RCLCPP_INFO(get_logger(), "Surfaced: odometry stopped, path kept until the next dive");
      return;
    }

    const bool fresh = has_setpoint_ && (t - last_setpoint_).seconds() < setpoint_timeout_s_;
    filter_.predict(fresh ? cmd_vx_ : 0.0, fresh ? cmd_vy_ : 0.0, dt);
    publish(t);
  }

  void startDive()
  {
    diving_ = true;
    filter_.reset(gps_sigma_m_);
    path_.poses.clear();
    path_.header.frame_id = frame_id_;

    double lat = pinger_lat_;
    double lon = pinger_lon_;
    if (!std::isfinite(lat) && has_buoy_) {lat = buoy_lat_; lon = buoy_lon_;}
    has_pinger_xy_ = std::isfinite(lat) && std::isfinite(lon);
    if (has_pinger_xy_) {
      latLonToEnuOffsets(
        surface_fix_.latitude, surface_fix_.longitude, lat, lon, pinger_x_, pinger_y_);
    }
    RCLCPP_INFO(
      get_logger(), "Dive started: origin latched, %s",
      !use_hydrophone_ ? "no hydrophone (dead reckoning only)" :
      has_pinger_xy_ ? "pinger position known (bearing updates on)" :
      "pinger unknown (dead reckoning only)");
  }

  void publish(const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = filter_.x();
    odom.pose.pose.position.y = filter_.y();
    odom.pose.pose.position.z = -depth_m_;
    odom.pose.pose.orientation = orientation_;
    odom.pose.covariance[0] = filter_.covXX();
    odom.pose.covariance[1] = filter_.covXY();
    odom.pose.covariance[6] = filter_.covXY();
    odom.pose.covariance[7] = filter_.covYY();
    odom_pub_->publish(odom);

    geometry_msgs::msg::PoseStamped ps;
    ps.header = odom.header;
    ps.pose = odom.pose.pose;
    if (!path_.poses.empty()) {
      const auto & last = path_.poses.back().pose.position;
      if (std::hypot(ps.pose.position.x - last.x, ps.pose.position.y - last.y) <
        min_path_step_m_)
      {
        return;
      }
    }
    path_.poses.push_back(ps);
    if (static_cast<int>(path_.poses.size()) > max_poses_) {
      path_.poses.erase(path_.poses.begin());
    }
    path_.header.stamp = stamp;
    path_pub_->publish(path_);
  }

  UnderwaterFilter filter_;
  double dive_depth_m_{0.5}, surface_depth_m_{0.2}, gps_sigma_m_{2.0};
  double setpoint_timeout_s_{0.5}, min_path_step_m_{0.05};
  int max_poses_{5000};
  std::string frame_id_, child_frame_id_;
  double pinger_lat_{std::nan("")}, pinger_lon_{std::nan("")};

  bool diving_{false}, depth_valid_{false}, yaw_valid_{false}, has_fix_{false};
  bool has_buoy_{false}, has_setpoint_{false}, has_pinger_xy_{false}, use_hydrophone_{false};
  double depth_m_{0.0}, yaw_{0.0}, cmd_vx_{0.0}, cmd_vy_{0.0};
  double buoy_lat_{0.0}, buoy_lon_{0.0}, pinger_x_{0.0}, pinger_y_{0.0};
  geometry_msgs::msg::Quaternion orientation_;
  sensor_msgs::msg::NavSatFix surface_fix_;
  rclcpp::Time last_setpoint_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path path_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<banyu_msgs::msg::WaterDepth>::SharedPtr depth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<banyu_msgs::msg::Task2BuoyTarget>::SharedPtr buoy_sub_;
  rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<banyu_msgs::msg::PingerObservation>::SharedPtr pinger_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace banyu_vehicle_execution

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<banyu_vehicle_execution::UnderwaterOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
