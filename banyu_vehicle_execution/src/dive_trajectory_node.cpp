#include <cmath>
#include <memory>
#include <string>

#include "banyu_msgs/msg/water_depth.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace banyu_vehicle_execution
{

// Publishes the submerged trajectory as a Path whose (0, 0) is the last surface (GPS-aided) pose.
class DiveTrajectoryNode : public rclcpp::Node
{
public:
  DiveTrajectoryNode()
  : Node("dive_trajectory_node")
  {
    dive_depth_m_ = declare_parameter<double>("dive_depth_m", 0.5);
    surface_depth_m_ = declare_parameter<double>("surface_depth_m", 0.2);
    max_poses_ = declare_parameter<int>("max_poses", 5000);
    min_step_m_ = declare_parameter<double>("min_step_m", 0.05);
    frame_id_ = declare_parameter<std::string>("frame_id", "dive_origin");
    const auto pose_topic = declare_parameter<std::string>(
      "topics.pose", "/mavros/local_position/pose");
    const auto depth_topic = declare_parameter<std::string>(
      "topics.water_depth", "/banyu/vehicle/water_depth");
    const auto path_topic = declare_parameter<std::string>(
      "topics.path", "/banyu/dive/path");

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, rclcpp::QoS(1).reliable());
    depth_sub_ = create_subscription<banyu_msgs::msg::WaterDepth>(
      depth_topic, 10, [this](banyu_msgs::msg::WaterDepth::ConstSharedPtr msg) {
        if (msg->valid) {depth_m_ = msg->depth_m; depth_valid_ = true;}
      });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, rclcpp::SensorDataQoS(),
      std::bind(&DiveTrajectoryNode::handlePose, this, std::placeholders::_1));
  }

private:
  void handlePose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!depth_valid_) {return;}

    if (!diving_) {
      surface_pose_ = *msg;
      if (depth_m_ > dive_depth_m_) {
        // Origin is the last pose seen while still at the surface.
        diving_ = true;
        path_.poses.clear();
        path_.header.frame_id = frame_id_;
        RCLCPP_INFO(get_logger(), "Dive started: trajectory origin latched");
      } else {
        return;
      }
    } else if (depth_m_ < surface_depth_m_) {
      diving_ = false;
      RCLCPP_INFO(get_logger(), "Surfaced: trajectory kept until next dive");
      return;
    }

    geometry_msgs::msg::PoseStamped rel = *msg;
    rel.header.frame_id = frame_id_;
    rel.pose.position.x -= surface_pose_.pose.position.x;
    rel.pose.position.y -= surface_pose_.pose.position.y;
    rel.pose.position.z = -depth_m_;

    if (!path_.poses.empty()) {
      const auto & last = path_.poses.back().pose.position;
      if (std::hypot(rel.pose.position.x - last.x, rel.pose.position.y - last.y) < min_step_m_) {
        return;
      }
    }
    path_.poses.push_back(rel);
    if (static_cast<int>(path_.poses.size()) > max_poses_) {
      path_.poses.erase(path_.poses.begin());
    }
    path_.header.stamp = msg->header.stamp;
    path_pub_->publish(path_);
  }

  double dive_depth_m_{0.5};
  double surface_depth_m_{0.2};
  double min_step_m_{0.05};
  int max_poses_{5000};
  std::string frame_id_;
  bool diving_{false};
  bool depth_valid_{false};
  double depth_m_{0.0};
  geometry_msgs::msg::PoseStamped surface_pose_;
  nav_msgs::msg::Path path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<banyu_msgs::msg::WaterDepth>::SharedPtr depth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

}  // namespace banyu_vehicle_execution

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<banyu_vehicle_execution::DiveTrajectoryNode>());
  rclcpp::shutdown();
  return 0;
}
