// Draws the Task 4 keep-out zones and surface obstacles as RViz markers (read-only, no window of its own).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <string>

#include "banyu_msgs/msg/surface_obstacles.hpp"
#include "banyu_msgs/msg/task4_zones.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kDegToRad = M_PI / 180.0;
}  // namespace

class ZoneMarkersNode : public rclcpp::Node
{
public:
  ZoneMarkersNode()
  : Node("zone_markers")
  {
    frame_ = declare_parameter<std::string>("frame_id", "map");
    const double rate_hz = declare_parameter<double>("publish_rate_hz", 2.0);
    const auto latched = rclcpp::QoS(1).transient_local().reliable();

    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/banyu/viz/zones", latched);
    // Volatile subscriptions match either durability; both sources republish at 1 Hz.
    const auto reliable = rclcpp::QoS(5).reliable();
    zones_sub_ = create_subscription<banyu_msgs::msg::Task4Zones>(
      "/banyu/task4/zones", reliable,
      [this](banyu_msgs::msg::Task4Zones::SharedPtr m) {zones_ = *m; have_zones_ = true;});
    obstacles_sub_ = create_subscription<banyu_msgs::msg::SurfaceObstacles>(
      "/banyu/surface/obstacles", reliable,
      [this](banyu_msgs::msg::SurfaceObstacles::SharedPtr m) {obstacles_[m->source] = *m;});
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::NavSatFix::SharedPtr m) {fix_ = *m; have_fix_ = true;});
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {pose_ = *m; have_pose_ = true;});

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(rate_hz, 0.1)), [this]() {publish();});
  }

private:
  // Flat-earth offset of a lat/lon from the vehicle's current fix, added to its local pose.
  bool toLocal(double lat, double lon, double & x, double & y) const
  {
    if (!have_fix_ || !have_pose_) {return false;}
    const double lat0 = fix_.latitude * kDegToRad;
    const double east = (lon - fix_.longitude) * kDegToRad * kEarthRadiusM * std::cos(lat0);
    const double north = (lat - fix_.latitude) * kDegToRad * kEarthRadiusM;
    x = pose_.pose.position.x + east;
    y = pose_.pose.position.y + north;
    return true;
  }

  visualization_msgs::msg::Marker disc(
    const std::string & ns, int id, double x, double y, double radius, float r, float g, float b)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_;
    m.header.stamp = now();
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = 2.0 * radius;
    m.scale.z = 0.1;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 0.4F;
    return m;
  }

  void publish()
  {
    visualization_msgs::msg::MarkerArray out;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    out.markers.push_back(clear);

    double x = 0.0;
    double y = 0.0;
    if (have_zones_) {
      for (size_t i = 0; i < zones_.static_radius_m.size(); ++i) {
        if (toLocal(zones_.static_latitude_deg[i], zones_.static_longitude_deg[i], x, y)) {
          out.markers.push_back(
            disc("task4_static", static_cast<int>(i), x, y, zones_.static_radius_m[i], 1.0F, 0.1F, 0.1F));
        }
      }
      if (zones_.moving_active &&
        toLocal(zones_.moving_latitude_deg, zones_.moving_longitude_deg, x, y))
      {
        // Dead-reckon the alert from its receipt time along the reported heading.
        const rclcpp::Time stamp(zones_.moving_stamp, get_clock()->get_clock_type());
        const double travel = zones_.moving_speed_mps * std::max((now() - stamp).seconds(), 0.0);
        x += travel * std::cos(zones_.moving_heading_enu_rad);
        y += travel * std::sin(zones_.moving_heading_enu_rad);
        out.markers.push_back(disc("task4_moving", 0, x, y, zones_.moving_radius_m, 1.0F, 0.5F, 0.0F));

        auto arrow = disc("task4_moving_dir", 0, x, y, 0.0, 1.0F, 0.5F, 0.0F);
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.pose.orientation.z = std::sin(zones_.moving_heading_enu_rad / 2.0);
        arrow.pose.orientation.w = std::cos(zones_.moving_heading_enu_rad / 2.0);
        arrow.scale.x = std::max(zones_.moving_radius_m, 1.0);
        arrow.scale.y = 0.3;
        arrow.scale.z = 0.3;
        arrow.color.a = 1.0F;
        out.markers.push_back(arrow);
      }
    }
    int id = 0;
    for (const auto & [source, obs] : obstacles_) {
      for (size_t i = 0; i < obs.radius_m.size(); ++i) {
        if (toLocal(obs.latitude_deg[i], obs.longitude_deg[i], x, y)) {
          out.markers.push_back(
            disc("obstacle_" + source, id++, x, y, obs.radius_m[i], 1.0F, 0.9F, 0.1F));
        }
      }
    }
    pub_->publish(out);
  }

  std::string frame_;
  bool have_zones_{false};
  bool have_fix_{false};
  bool have_pose_{false};
  banyu_msgs::msg::Task4Zones zones_;
  std::map<std::string, banyu_msgs::msg::SurfaceObstacles> obstacles_;
  sensor_msgs::msg::NavSatFix fix_;
  geometry_msgs::msg::PoseStamped pose_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Subscription<banyu_msgs::msg::Task4Zones>::SharedPtr zones_sub_;
  rclcpp::Subscription<banyu_msgs::msg::SurfaceObstacles>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZoneMarkersNode>());
  rclcpp::shutdown();
  return 0;
}
