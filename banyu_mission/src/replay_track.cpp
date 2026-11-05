#include "banyu_mission/replay_track.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace banyu_mission
{
namespace fs = std::filesystem;

namespace
{
const char * const kColumns[] = {
  "time_s", "forward_pwm", "lateral_pwm", "yaw_pwm", "heading_rad", "depth_m"};

std::vector<std::string> splitCsv(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    field.erase(0, field.find_first_not_of(" \t\r"));
    field.erase(field.find_last_not_of(" \t\r") + 1);
    fields.push_back(field);
  }
  return fields;
}
}  // namespace

double ReplayTrack::maxDepthM() const
{
  return depth_m.empty() ? 0.0 : *std::max_element(depth_m.begin(), depth_m.end());
}

std::string replayTrackDirectory()
{
  if (const char * dir = std::getenv("BANYU_REPLAY_DIR"); dir != nullptr && *dir != '\0') {
    return dir;
  }
  const char * home = std::getenv("HOME");
  return std::string(home != nullptr ? home : ".") + "/banyu_replays";
}

std::string resolveReplayTrackPath(const std::string & spec)
{
  if (spec.empty()) {throw std::invalid_argument("replay_track is empty");}
  if (spec.front() == '/') {return spec;}
  const fs::path dir(replayTrackDirectory());
  if (spec == "latest") {
    std::error_code error;
    fs::path newest;
    fs::file_time_type newest_time{};
    for (const auto & entry : fs::directory_iterator(dir, error)) {
      if (!entry.is_regular_file() || entry.path().extension() != ".csv") {continue;}
      const auto time = entry.last_write_time();
      if (newest.empty() || time > newest_time) {
        newest = entry.path();
        newest_time = time;
      }
    }
    if (newest.empty()) {
      throw std::invalid_argument("no replay track (.csv) in " + dir.string());
    }
    return newest.string();
  }
  const auto path = dir / (fs::path(spec).extension() == ".csv" ? spec : spec + ".csv");
  if (!fs::exists(path)) {throw std::invalid_argument("replay track not found: " + path.string());}
  return path.string();
}

ReplayTrack loadReplayTrack(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {throw std::invalid_argument("cannot open replay track " + path);}
  ReplayTrack track;
  track.path = path;
  track.name = fs::path(path).stem().string();
  std::vector<std::vector<double> *> columns = {
    &track.time_s, &track.forward_pwm, &track.lateral_pwm,
    &track.yaw_pwm, &track.heading_rad, &track.depth_m};

  std::vector<int> index;  // CSV column of each kColumns entry
  std::size_t width = 0;
  std::string line;
  int line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    if (line.find_first_not_of(" \t\r") == std::string::npos || line.front() == '#') {continue;}
    const auto fields = splitCsv(line);
    const auto where = path + ":" + std::to_string(line_number);
    if (index.empty()) {
      std::map<std::string, int> header;
      for (std::size_t i = 0; i < fields.size(); ++i) {header[fields[i]] = static_cast<int>(i);}
      for (const char * column : kColumns) {
        if (header.count(column) == 0) {
          throw std::invalid_argument(where + ": header has no '" + column + "' column");
        }
        index.push_back(header[column]);
      }
      width = fields.size();
      continue;
    }
    if (fields.size() != width) {
      throw std::invalid_argument(where + ": expected " + std::to_string(width) + " fields");
    }
    for (std::size_t c = 0; c < columns.size(); ++c) {
      double value = 0.0;
      try {
        std::size_t used = 0;
        value = std::stod(fields[index[c]], &used);
        if (used != fields[index[c]].size()) {throw std::invalid_argument("trailing text");}
      } catch (const std::exception &) {
        throw std::invalid_argument(where + ": '" + fields[index[c]] + "' is not a number");
      }
      if (!std::isfinite(value)) {throw std::invalid_argument(where + ": non-finite value");}
      columns[c]->push_back(value);
    }
  }

  if (track.time_s.size() < 2) {
    throw std::invalid_argument(path + ": a replay track needs at least two samples");
  }
  if (std::abs(track.time_s.front()) > 1e-6) {
    throw std::invalid_argument(path + ": time_s must start at 0");
  }
  for (std::size_t i = 1; i < track.time_s.size(); ++i) {
    if (track.time_s[i] <= track.time_s[i - 1]) {
      throw std::invalid_argument(path + ": time_s must strictly increase");
    }
  }
  for (const auto * stick : {&track.forward_pwm, &track.lateral_pwm, &track.yaw_pwm}) {
    for (const double value : *stick) {
      if (std::abs(value) > 500.0) {
        throw std::invalid_argument(path + ": stick deflections must be within +/-500 PWM");
      }
    }
  }
  return track;
}

}  // namespace banyu_mission
