#ifndef BANYU_MISSION__MISSION_STATE_HPP_
#define BANYU_MISSION__MISSION_STATE_HPP_

#include <memory>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include "banyu_msgs/msg/pipeline_observation.hpp"
#include "banyu_msgs/msg/pipeline_ring_observation.hpp"
#include "banyu_msgs/msg/pipeline_ring_state.hpp"
#include "banyu_msgs/msg/pipeline_end_state.hpp"
#include "banyu_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace banyu_mission
{

class MissionState
{
public:
  enum class Lifecycle { Idle, Active, Succeeded, Failed, Interrupted };

  struct Snapshot
  {
    bool fcu_connected {false};
    bool armed {false};
    bool fcu_status_fresh {false};
    bool rc_fresh {false};
    bool operator_takeover {false};
    bool manual_interrupt {false};
    bool abort_requested {false};
    bool mission_started {false};
    bool gps_available {false};
    bool pipeline_fresh {false};
    bool pipeline_lost {true};
    bool pipeline_ended {false};
    bool red_pending {false};
    uint32_t red_record_index {0};
    bool ring_visible {false};
    uint8_t ring_color {0};
    std::string mode;
    std::string authority;
    uint32_t run_id {0};
    uint64_t epoch {0};
  };

  explicit MissionState(rclcpp::Node * node, bool subscribe = true);

  void configure(
    double gps_timeout_s, double pipeline_timeout_s, double state_timeout_s,
    int rc_interrupt_channel, int rc_interrupt_threshold,
    std::string autonomy_mode, std::string autonomy_authority = "",
    bool require_gps = true);
  bool configureMissionCoordinates(
    double start_latitude, double start_longitude,
    double end_latitude, double end_longitude,
    bool require_coordinates = true);
  bool resetForStart();
  void requestAbort();
  bool requestStart();
  bool acceptRun(uint32_t run_id, uint64_t epoch);
  void prepareAutonomy();
  void requireRunContext(bool required);
  void markSucceeded();
  void markFailed();
  void markInterrupted();
  void markAutonomyEngaged();
  bool autonomyEngaged() const;
  Snapshot snapshot() const;
  Lifecycle lifecycle() const;
  double missionHeading() const;

  bool autonomyInterrupted() const;
  bool vehicleReady() const;
  bool gpsAvailable() const;
  bool gpsRequired() const;
  bool pipelineLost() const;
  bool pipelineEnded() const;
  bool redPending() const;
  uint32_t pendingRedIndex() const;
  bool acknowledgeRed(uint32_t index);

  static bool validateMissionCoordinates(
    double start_latitude, double start_longitude,
    double end_latitude, double end_longitude);
  static bool deriveEnuInitialBearing(
    double start_latitude, double start_longitude,
    double end_latitude, double end_longitude, double & bearing);

private:
  void stateCallback(const banyu_msgs::msg::VehicleStatus::SharedPtr msg);
  void pipelineCallback(const banyu_msgs::msg::PipelineObservation::SharedPtr msg);
  void ringCallback(const banyu_msgs::msg::PipelineRingObservation::SharedPtr msg);
  void ringStateCallback(const banyu_msgs::msg::PipelineRingState::SharedPtr msg);
  void endCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void endStateCallback(const banyu_msgs::msg::PipelineEndState::SharedPtr msg);

  mutable std::mutex mutex_;
  rclcpp::Node * node_;
  bool fcu_connected_ {false};
  bool armed_ {false};
  bool require_run_context_ {false};
  uint32_t run_id_ {0};
  uint64_t epoch_ {0};
  rclcpp::Time run_started_at_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point state_received_at_{};
  bool fcu_status_fresh_ {false};
  bool rc_fresh_ {false};
  bool operator_takeover_ {false};
  bool manual_interrupt_ {false};
  bool abort_requested_ {false};
  bool mission_started_ {false};
  bool autonomy_engaged_ {false};
  bool gps_valid_ {false};
  bool pipeline_ended_ {false};
  bool pipeline_seen_ {false};
  bool pipeline_tracked_ {false};
  bool red_pending_ {false};
  uint32_t red_record_index_ {0};
  bool ring_visible_ {false};
  uint8_t ring_color_ {0};
  std::string mode_;
  std::string authority_;
  rclcpp::Time last_pipeline_;
  rclcpp::Time last_state_;
  double gps_timeout_s_ {2.0};
  double pipeline_timeout_s_ {1.0};
  double state_timeout_s_ {2.0};
  int rc_interrupt_channel_ {4};
  int rc_interrupt_threshold_ {1700};
  std::string autonomy_mode_ {"GUIDED"};
  std::string autonomy_authority_;
  Lifecycle lifecycle_ {Lifecycle::Idle};
  double start_latitude_ {0.0};
  double start_longitude_ {0.0};
  double end_latitude_ {0.0};
  double end_longitude_ {0.0};
  double mission_heading_ {0.0};
  bool mission_coordinates_valid_ {false};
  bool mission_coordinates_required_ {true};
  bool gps_required_ {true};
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__MISSION_STATE_HPP_
