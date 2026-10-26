#include "banyu_detection/resource_code_decoder.hpp"

#include <algorithm>
#include <map>
#include <utility>

namespace banyu_detection
{
namespace
{

struct Run
{
  int colour;
  double first;   // time of the first sample
  double last;    // time of the last sample
};

bool isCode(int colour) {return colour >= 1 && colour <= 3;}

}  // namespace

ResourceCodeDecoder::ResourceCodeDecoder(const ResourceCodeConfig & config)
: config_(config) {}

void ResourceCodeDecoder::reset() {samples_.clear();}

void ResourceCodeDecoder::add(double t, int colour)
{
  samples_.push_back({t, isCode(colour) ? colour : 0});
}

ResourceCodeResult ResourceCodeDecoder::decode() const
{
  ResourceCodeResult result;
  if (samples_.size() < 2) {return result;}

  std::vector<CodeSample> s = samples_;
  std::stable_sort(s.begin(), s.end(), [](const CodeSample & a, const CodeSample & b) {
      return a.t < b.t;
    });

  // Frame period, so a run that is one frame long still has a duration.
  std::vector<double> dts;
  for (std::size_t i = 1; i < s.size(); ++i) {
    const double dt = s[i].t - s[i - 1].t;
    if (dt > 0.0) {dts.push_back(dt);}
  }
  if (dts.empty()) {return result;}
  std::nth_element(dts.begin(), dts.begin() + dts.size() / 2, dts.end());
  const double frame = dts[dts.size() / 2];

  // Runs of one colour. Dark samples split runs but are not runs themselves.
  std::vector<Run> runs;
  for (const auto & x : s) {
    if (!isCode(x.colour)) {continue;}
    if (!runs.empty() && runs.back().colour == x.colour && x.t - runs.back().last <= 1.5 * frame) {
      runs.back().last = x.t;
    } else {
      runs.push_back({x.colour, x.t, x.t});
    }
  }
  // A short dropout inside one colour does not end the flash.
  std::vector<Run> merged;
  for (const auto & r : runs) {
    if (!merged.empty() && merged.back().colour == r.colour &&
      r.first - merged.back().last <= config_.merge_gap_s)
    {
      merged.back().last = r.last;
    } else {
      merged.push_back(r);
    }
  }

  // Flashes are the runs of about one second. Longer runs are the steady green after the repair.
  struct Flash
  {
    int colour;
    double first;
    double last;
    double gap_before;   // dark time since the previous run; a negative value means none before
    bool after_steady;   // directly follows a steady run
  };
  std::vector<Flash> flashes;
  for (std::size_t i = 0; i < merged.size(); ++i) {
    const double duration = merged[i].last - merged[i].first + frame;
    const bool has_prev = i > 0;
    const double gap = has_prev ? merged[i].first - merged[i - 1].last - frame : -1.0;
    bool steady_prev = false;
    if (has_prev) {
      const double pd = merged[i - 1].last - merged[i - 1].first + frame;
      steady_prev = pd > config_.steady_on_s;
    }
    if (duration < config_.min_on_s || duration > config_.max_on_s) {continue;}
    flashes.push_back({merged[i].colour, merged[i].first, merged[i].last, gap, steady_prev});
  }

  // A node only turns green once the magnetic switch is activated, so a long green run is the proof.
  for (const auto & r : merged) {
    if (r.colour == 2 && r.last - r.first + frame > config_.steady_on_s) {
      result.repair_verified = true;
      break;
    }
  }

  std::map<std::pair<int, int>, int> votes;
  int seen = 0;
  if (config_.code_length <= 1) {
    for (const auto & f : flashes) {
      ++votes[{0, f.colour}];
      ++seen;
    }
  } else {
    for (std::size_t i = 0; i + 1 < flashes.size(); ++i) {
      const auto & a = flashes[i];
      const auto & b = flashes[i + 1];
      // Colour 1 starts a cycle: it follows the long dark gap, or the steady green.
      const bool starts_cycle = a.after_steady || a.gap_before >= config_.long_gap_s;
      const double between = b.first - a.last - frame;
      // Colour 2 comes after the short 1 s gap.
      const bool is_pair = between > 0.4 * config_.long_gap_s && between < config_.long_gap_s;
      if (starts_cycle && is_pair) {
        ++votes[{a.colour, b.colour}];
        ++seen;
        ++i;   // b belongs to this cycle
      }
    }
  }

  result.cycles_seen = seen;
  if (votes.empty()) {return result;}
  const auto best = std::max_element(
    votes.begin(), votes.end(), [](const auto & x, const auto & y) {return x.second < y.second;});
  result.tin = best->first.first;
  result.circle = best->first.second;
  result.cycles = best->second;
  result.confidence = static_cast<float>(best->second) / static_cast<float>(seen);
  result.complete = best->second >= config_.min_cycles;
  return result;
}

}  // namespace banyu_detection
