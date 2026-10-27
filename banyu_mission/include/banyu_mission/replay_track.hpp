#ifndef BANYU_MISSION__REPLAY_TRACK_HPP_
#define BANYU_MISSION__REPLAY_TRACK_HPP_

#include <string>
#include <vector>

namespace banyu_mission
{

// A teach track written by tools/scripts/por_teach_extract.py (one row per recorded sample).
struct ReplayTrack
{
  std::string name;
  std::string path;
  std::vector<double> time_s;
  std::vector<double> forward_pwm;
  std::vector<double> lateral_pwm;
  std::vector<double> yaw_pwm;
  std::vector<double> heading_rad;
  std::vector<double> depth_m;

  double durationS() const {return time_s.empty() ? 0.0 : time_s.back();}
  double maxDepthM() const;
};

// $BANYU_REPLAY_DIR, else ~/banyu_replays.
std::string replayTrackDirectory();

// "latest" -> newest .csv in the replay directory; a bare name -> that file in the directory
// (".csv" optional); an absolute path is used as is. Throws when nothing matches.
std::string resolveReplayTrackPath(const std::string & spec);

// Parses and validates a track CSV; throws std::invalid_argument with the reason.
ReplayTrack loadReplayTrack(const std::string & path);

}  // namespace banyu_mission

#endif  // BANYU_MISSION__REPLAY_TRACK_HPP_
