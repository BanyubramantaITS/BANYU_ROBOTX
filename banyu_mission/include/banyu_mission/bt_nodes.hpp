#ifndef BANYU_MISSION_BT_NODES_HPP_
#define BANYU_MISSION_BT_NODES_HPP_

#include <array>
#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/condition_node.h>

#include <rclcpp/rclcpp.hpp>

#include "banyu_mission/mission_context.hpp"

namespace banyu_mission {

// SUCCESS when the mission is inactive or has been aborted. Placed as the
// first child of a ReactiveFallback so an abort halts the mission branch
// immediately and hands control back to teleop.
class AbortedOrTeleop : public BT::ConditionNode {
 public:
  AbortedOrTeleop(const std::string& name, const BT::NodeConfig& config,
                  std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  std::shared_ptr<MissionContext> ctx_;
};

// SUCCESS while a mission is running.
class MissionActive : public BT::ConditionNode {
 public:
  MissionActive(const std::string& name, const BT::NodeConfig& config,
                std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  std::shared_ptr<MissionContext> ctx_;
};

// Dive in place: closed-loop velocity navigation to the mission origin plus
// gate_offset_* NED offsets. ArduSub's AC_WPNav position control is
// unreliable in this sim, so this node emits velocity-only setpoints (ROS
// ENU components). SUCCESS on arrival or timeout.
class NavToGate : public BT::StatefulActionNode {
 public:
  NavToGate(const std::string& name, const BT::NodeConfig& config,
            const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  rclcpp::Time start_time_;
};

// Visual alignment base: uses camera offsets to center the target object in
// the front camera. Publishes body-frame velocity setpoints (0xDC7) and
// succeeds once the offsets stay inside the configured deadband. Object id
// and parameter prefix are provided by the derived classes.
class VisualAlign : public BT::StatefulActionNode {
 public:
  VisualAlign(const std::string& name, const BT::NodeConfig& config,
              const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx,
              int object_id, const std::string& prefix);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 protected:
  double param(const std::string& name) const;

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  int object_id_;
  std::string prefix_;
  rclcpp::Time start_time_;
  rclcpp::Time aligned_since_;
  bool alignment_started_{false};
};

class GateAlign : public VisualAlign {
 public:
  GateAlign(const std::string& name, const BT::NodeConfig& config,
            const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
      : VisualAlign(name, config, node, std::move(ctx), 0, "gate_align_") {}
};

class MarkerAlign : public VisualAlign {
 public:
  MarkerAlign(const std::string& name, const BT::NodeConfig& config,
              const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
      : VisualAlign(name, config, node, std::move(ctx), 1, "marker_align_") {}
};

// Drive forward (body frame) through the gate. Completes once the travelled
// distance exceeds `gate_pass_distance`, or once the gate leaves the camera
// view after a minimum distance (means the sub is through). SUCCESS or
// timeout.
class GatePass : public BT::StatefulActionNode {
 public:
  GatePass(const std::string& name, const BT::NodeConfig& config,
           const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  rclcpp::Time start_time_;
  geometry_msgs::msg::Point start_pose_;
};

// Sweep forward while searching for the yellow buoy marker (object_id=1).
// Once detected, keep approaching until the reported distance reaches the
// configured approach range, using marker_align_* gains to keep the buoy
// centered while closing in. If the detection is lost mid-approach, hold
// position and wait for re-acquisition instead of driving forward blindly.
class SearchMarker : public BT::StatefulActionNode {
 public:
  SearchMarker(const std::string& name, const BT::NodeConfig& config,
               const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  rclcpp::Time start_time_;
  bool seen_{false};
};

// Orbit the marker a full revolution: drive tangentially (body forward) while
// correcting the standoff range with lateral velocity. Completion is detected
// by integrating the vehicle yaw (|accumulated| >= orbit_angle_rad), not by
// time, so the orbit is a true 360 degrees regardless of radius/speed.
class MarkerOrbit : public BT::StatefulActionNode {
 public:
  MarkerOrbit(const std::string& name, const BT::NodeConfig& config,
              const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  rclcpp::Time start_time_;
  double start_yaw_{0.0};
  double last_yaw_{0.0};
  double accumulated_yaw_{0.0};
};

// Rotate the vehicle in place by `turn_angle_rad` (default 180 deg) using a
// yaw-rate setpoint, tracked by yaw integration. SUCCESS or timeout.
class TurnAround : public BT::StatefulActionNode {
 public:
  TurnAround(const std::string& name, const BT::NodeConfig& config,
             const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  rclcpp::Time start_time_;
  double start_yaw_{0.0};
  double last_yaw_{0.0};
  double accumulated_yaw_{0.0};
};

// Sweep forward (like SearchMarker) until a fresh gate detection arrives,
// then SUCCESS so GateAlign can center on it.
class SearchGate : public BT::StatefulActionNode {
 public:
  SearchGate(const std::string& name, const BT::NodeConfig& config,
             const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  rclcpp::Time start_time_;
};

// Vision-assisted drive through the gate on the return leg: forward at
// `gate_pass_speed` while keeping the gate centered with gate_align_* gains.
// Tracks the closest observed distance; once the gate is lost after being
// seen close (`gate_pass_lost_distance`), keep driving blindly for
// `gate_pass_after_loss_m` to clear the gate plane, then SUCCESS.
class ReturnGatePass : public BT::StatefulActionNode {
 public:
  ReturnGatePass(const std::string& name, const BT::NodeConfig& config,
                 const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  rclcpp::Time start_time_;
  double min_distance_{std::numeric_limits<double>::infinity()};
  double loss_distance_{0.0};
  bool after_loss_{false};
  geometry_msgs::msg::Point loss_start_pose_;
};

// Ascend to `surface_depth` (straight up), then SUCCESS.
class Surface : public BT::StatefulActionNode {
 public:
  Surface(const std::string& name, const BT::NodeConfig& config,
          const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  rclcpp::Time start_time_;
};

class Task2Preflight : public BT::StatefulActionNode {
 public:
  Task2Preflight(const std::string& name, const BT::NodeConfig& config,
                 const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  rclcpp::Time start_time_;
};

class FollowTask2Pipeline : public BT::StatefulActionNode {
 public:
  FollowTask2Pipeline(const std::string& name, const BT::NodeConfig& config,
                      const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  std::array<geometry_msgs::msg::PoseStamped, 6> route_;
  std::array<banyu_msgs::msg::PipelineRingState, 5> ring_states_;
  rclcpp::Time start_time_;
  rclcpp::Time segment_start_time_;
  rclcpp::Time hold_start_time_;
  std::size_t waypoint_index_{0};
  bool holding_{false};
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION_BT_NODES_HPP_
