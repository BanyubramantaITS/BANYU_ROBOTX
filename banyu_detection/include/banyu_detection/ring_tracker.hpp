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
  double node_association_distance_m{0.8};
  // With four records, a fifth distinct ring merges the two records closest in progress, only if they are closer
  // than this (one node recorded twice); otherwise the fifth ring is ignored. 0 disables the merge.
  double min_node_spacing_m{1.0};
  // Bottom camera geometry for estimating where a ring is along the pipe; disabled when
  // camera_height_m is 0 (ring position then equals vehicle position).
  double camera_height_m{0.0};
  // With both set, the height of the camera above the ring comes from the ring's apparent size (observation.size,
  // fraction of the image width) instead of camera_height_m, which only holds at the survey altitude. lid_diameter_m is
  // the lit lid, focal_ratio is fx over the image width.
  double lid_diameter_m{0.0};
  double focal_ratio{0.0};
  double camera_forward_m{0.0};
  double camera_tilt_from_vertical_rad{0.0};
  double camera_half_vfov_rad{0.0};
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
  // Camera height above the pipe measured now (from the pipe's apparent width); 0 = use camera_height_m.
  double camera_height_m{0.0};
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
  // Set when two records were merged to make room for this one: the surviving record and the superseded one (both
  // to publish before this record).
  bool merged{false};
  banyu_msgs::msg::Task2RingRecord merged_record;
  banyu_msgs::msg::Task2RingRecord superseded_record;
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

  void setCenterline(const std::vector<std::pair<double, double>> & waypoints);
  bool hasCenterline() const;
  double projectOntoCenterline(double x, double y) const;
  int findAssociatedRecord(
    double progress_m,
    const geometry_msgs::msg::PoseStamped & pose) const;

  const RingTrackerSnapshot & snapshot() const {return snapshot_;}
  const std::vector<banyu_msgs::msg::Task2RingRecord> & records() const
  {
    return snapshot_.records;
  }
  bool redPending() const {return snapshot_.red_pending;}
  // Records that are nodes (not superseded by a merge).
  std::size_t liveRecordCount() const;
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
  void beginNewRingEntry();
  bool cameraModelEnabled() const;
  bool ringProgress(const RingTrackerInput & input, double & progress) const;
  static bool kind_is_visible(const banyu_msgs::msg::PipelineRingObservation & observation);

  void beginExit();
  void beginDebounce();
  void clearEntry();
  RingTrackerUpdate ignoredUpdate() const;
  RingTrackerUpdate accept(const RingTrackerInput & input);
  // Merges the two live records closest in progress into one (RED wins) and marks the other superseded; false if they
  // lack progress or are not closer than min_node_spacing_m.
  bool mergeClosestRecords(std::size_t & survivor, std::size_t & superseded);

  RingTrackerConfig config_;
  RingTrackerSnapshot snapshot_;
  bool has_sequence_{false};
  uint64_t last_sequence_{0};
  double last_record_path_distance_{0.0};
  bool has_last_record_path_distance_{false};
  // Which way the progress runs in the order the rings are met: +1 from the active buoy end, -1 when the pipe is first met
  // from the other end. The second ring fixes it; later rings must keep going the same way.
  int traverse_direction_{0};
  geometry_msgs::msg::PoseStamped last_record_pose_;
  bool has_last_record_pose_{false};
  bool rearmed_{false};
  bool beyond_last_node_{false};
  uint8_t pending_color_{
    banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN};
  bool has_pending_color_{false};
  std::vector<std::pair<double, double>> centerline_waypoints_;
};

}  // namespace banyu_detection

#endif  // BANYU_DETECTION_RING_TRACKER_HPP_
