#ifndef BANYU_DETECTION_RING_TRACKER_HPP_
#define BANYU_DETECTION_RING_TRACKER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <banyu_msgs/msg/pipeline_ring_observation.hpp>
#include <banyu_msgs/msg/pipeline_ring_state.hpp>
#include <banyu_msgs/msg/task2_ring_record.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace banyu_detection
{

enum class RingTrackerPhase
{
  IDLE,
  ENTRY,
  TRACKING,
  EXIT,
  DEBOUNCE,
};

struct RingTrackerConfig
{
  std::size_t entry_confirm_frames{3};
  std::size_t exit_confirm_frames{2};
  std::size_t debounce_frames{2};
  double minimum_confidence{0.2};
  double minimum_rearm_distance_m{0.25};
  double pose_timestamp_tolerance_s{0.05};
  uint64_t maximum_sequence_gap{1};
  bool reject_duplicate_red{true};
};

struct RingTrackerInput
{
  uint32_t run_id{0};
  uint64_t epoch{0};
  banyu_msgs::msg::PipelineRingObservation observation;
  geometry_msgs::msg::PoseStamped vehicle_pose_map;
  bool has_vehicle_pose{false};
  double path_distance_m{0.0};
  bool has_path_distance{false};
};

struct RingTrackerUpdate
{
  RingTrackerPhase phase{RingTrackerPhase::IDLE};
  bool accepted{false};
  bool exited{false};
  bool ignored{false};
  std::size_t consecutive_frames{0};
  std::size_t record_index{0};
  banyu_msgs::msg::Task2RingRecord record;
};

struct RingTrackerSnapshot
{
  RingTrackerPhase phase{RingTrackerPhase::IDLE};
  uint8_t active_color{
    banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN};
  std::size_t consecutive_frames{0};
  std::size_t invisible_frames{0};
  bool red_pending{false};
  bool repaired{false};
  std::vector<banyu_msgs::msg::Task2RingRecord> records;
  bool has_last_state{false};
  banyu_msgs::msg::PipelineRingState last_state;
};

class RingTracker
{
public:
  explicit RingTracker(RingTrackerConfig config = {});

  RingTrackerUpdate update(const RingTrackerInput & input);

  // Camera-only observations can advance temporal state but cannot append a
  // record without an aligned map pose.
  RingTrackerUpdate update(const banyu_msgs::msg::PipelineRingObservation & observation);

  // Repair is an explicit mission event; seeing a red ring again is not
  // treated as proof that the physical ring was repaired.
  bool markRedRepaired();
  void reset();

  const RingTrackerSnapshot & snapshot() const {return snapshot_;}
  const std::vector<banyu_msgs::msg::Task2RingRecord> & records() const
  {
    return snapshot_.records;
  }
  bool redPending() const {return snapshot_.red_pending;}
  bool repaired() const {return snapshot_.repaired;}

private:
  static bool isKnownColor(uint8_t color);
  enum class ObservationKind
  {
    VISIBLE,
    INVISIBLE,
    MALFORMED_OR_UNCERTAIN,
  };
  ObservationKind classifyObservation(
    const banyu_msgs::msg::PipelineRingObservation & observation) const;
  static bool sameColor(uint8_t first, uint8_t second);
  bool hasRecordedColor(uint8_t color) const;
  bool validPoseForObservation(const RingTrackerInput & input) const;
  bool rearmAllowed(const RingTrackerInput & input) const;

  void beginExit();
  void beginDebounce();
  void clearEntry();
  RingTrackerUpdate ignoredUpdate() const;
  RingTrackerUpdate accept(const RingTrackerInput & input);

  RingTrackerConfig config_;
  RingTrackerSnapshot snapshot_;
  bool has_sequence_{false};
  uint64_t last_sequence_{0};
  double last_record_path_distance_{0.0};
  bool has_last_record_path_distance_{false};
  geometry_msgs::msg::PoseStamped last_record_pose_;
  bool has_last_record_pose_{false};
  uint8_t pending_color_{
    banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN};
  bool has_pending_color_{false};
};

}  // namespace banyu_detection

#endif  // BANYU_DETECTION_RING_TRACKER_HPP_
