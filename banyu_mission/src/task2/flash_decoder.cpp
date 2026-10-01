#include "banyu_mission/task2/flash_decoder.hpp"

namespace banyu_mission::task2 {

namespace {
constexpr double kDropoutS = 0.3;   // detector misses shorter than this do not end a run
constexpr double kMinRunS = 0.4;    // a flash is 1 s; shorter runs are misclassified frames
constexpr double kLongGapS = 1.5;   // Disruptive: 2 s dark before colour 1, 1 s before colour 2
}  // namespace

void FlashDecoder::update(std::optional<uint8_t> colour, double now_s) {
  if (!colour) {
    if (lit_ && now_s - last_lit_s_ > kDropoutS) closeRun();
    return;
  }
  if (lit_ != colour) {
    if (lit_) closeRun();
    lit_ = colour;
    run_start_s_ = now_s;
  }
  last_lit_s_ = now_s;
}

void FlashDecoder::closeRun() {
  if (last_lit_s_ - run_start_s_ >= kMinRunS) runs_.push_back({*lit_, run_start_s_, last_lit_s_});
  lit_.reset();
}

std::optional<uint8_t> FlashDecoder::colour() const {
  // runs_[0] is the steady GREEN; the flashes follow.
  for (size_t i = 2; i < runs_.size(); ++i)
    if (runs_[i].colour == runs_[i - 1].colour) return runs_[i].colour;
  return std::nullopt;
}

std::optional<std::pair<uint8_t, uint8_t>> FlashDecoder::pair() const {
  std::optional<std::pair<uint8_t, uint8_t>> last;
  for (size_t i = 1; i + 1 < runs_.size(); ++i) {
    if (runs_[i].start_s - runs_[i - 1].end_s < kLongGapS) continue;  // not the start of a cycle
    const std::pair<uint8_t, uint8_t> cycle{runs_[i].colour, runs_[i + 1].colour};
    if (last == cycle) return cycle;
    last = cycle;
  }
  return std::nullopt;
}

}  // namespace banyu_mission::task2
