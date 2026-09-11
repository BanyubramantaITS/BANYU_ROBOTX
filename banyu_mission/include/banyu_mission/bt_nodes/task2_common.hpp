#ifndef BANYU_MISSION_BT_NODES_TASK2_COMMON_HPP_
#define BANYU_MISSION_BT_NODES_TASK2_COMMON_HPP_

#include <array>
#include <cstdint>
#include <cstddef>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>

#include "banyu_mission/mission_context.hpp"

namespace banyu_mission::bt_nodes::task2_common {

constexpr std::size_t kWaypointCount = 6;
constexpr std::size_t kRingCount = 5;

bool receiptIsFresh(const rclcpp::Time& now, const rclcpp::Time& received, double timeout_s);
bool finitePose(const geometry_msgs::msg::PoseStamped& pose);
bool task2RingColorKnown(uint8_t color);
std::string task2RingColorName(uint8_t color);

bool snapshotInputsLocked(
    const MissionContext& ctx, const rclcpp::Time& now, double route_timeout_s,
    double ring_timeout_s, double ring_confidence_min, double route_min_z, double route_max_z,
    double min_segment_spacing, double max_segment_spacing, double min_path_length,
    double max_path_length, std::array<geometry_msgs::msg::PoseStamped, kWaypointCount>& route,
    std::array<banyu_msgs::msg::PipelineRingState, kRingCount>& rings);

bool vehicleReady(const MissionContext& ctx, const rclcpp::Time& now, double pose_timeout_s,
                  double state_timeout_s, geometry_msgs::msg::PoseStamped& pose,
                  mavros_msgs::msg::State& state);

}  // namespace banyu_mission::bt_nodes::task2_common

#endif  // BANYU_MISSION_BT_NODES_TASK2_COMMON_HPP_
