#ifndef BANYU_MISSION_BT_NODES_HPP_
#define BANYU_MISSION_BT_NODES_HPP_

#include <memory>

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

// Stream a position-only setpoint (FRAME_LOCAL_NED) toward the gate. On the
// first tick it captures the current position as the mission origin and
// derives the absolute NED gate target. SUCCESS on arrival or timeout.
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

// Phase B/D stubs. Each holds a safe zero-velocity position for
// `stub_hold_time_s` (so the whole tree chain can be exercised end to end)
// and logs which phase will be implemented next.
class StubHold : public BT::StatefulActionNode {
 public:
  StubHold(const std::string& name, const BT::NodeConfig& config,
           const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx,
           const std::string& phase);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts() { return {}; }

 protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionContext> ctx_;
  std::string phase_;
  rclcpp::Time start_time_;
};

class GateAlign : public StubHold {
 public:
  GateAlign(const std::string& name, const BT::NodeConfig& config,
            const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
      : StubHold(name, config, node, std::move(ctx), "GateAlign") {}
};

class GatePass : public StubHold {
 public:
  GatePass(const std::string& name, const BT::NodeConfig& config,
           const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
      : StubHold(name, config, node, std::move(ctx), "GatePass") {}
};

class SearchMarker : public StubHold {
 public:
  SearchMarker(const std::string& name, const BT::NodeConfig& config,
               const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
      : StubHold(name, config, node, std::move(ctx), "SearchMarker") {}
};

class MarkerAlign : public StubHold {
 public:
  MarkerAlign(const std::string& name, const BT::NodeConfig& config,
              const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
      : StubHold(name, config, node, std::move(ctx), "MarkerAlign") {}
};

class MarkerOrbit : public StubHold {
 public:
  MarkerOrbit(const std::string& name, const BT::NodeConfig& config,
              const rclcpp::Node::SharedPtr& node, std::shared_ptr<MissionContext> ctx)
      : StubHold(name, config, node, std::move(ctx), "MarkerOrbit") {}
};

// Stream a position-only setpoint back to the captured mission origin, then
// SUCCESS (mission complete).
class ReturnToStart : public BT::StatefulActionNode {
 public:
  ReturnToStart(const std::string& name, const BT::NodeConfig& config,
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

}  // namespace banyu_mission

#endif  // BANYU_MISSION_BT_NODES_HPP_
