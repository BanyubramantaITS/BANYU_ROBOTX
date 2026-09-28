// Reads what the repaired light node flashes (handbook 3.3.3). It turns GREEN for 5 s, then:
//   Advanced:   one colour, 1 s on / 1 s off, for 30 s (the delivery circle).
//   Disruptive: colour 1 for 1 s, off 1 s, colour 2 for 1 s, off 2 s, for 30 s (the tin to fetch,
//               then the circle to deliver it to); the long gap says which comes first.
// Fed the colour seen at the probe each frame (or nothing), it splits the stream into lit runs.
// The first run is the steady GREEN (a GREEN flash straight after merges into it); every later
// run is a flash.
#ifndef BANYU_MISSION_TASK2_FLASH_DECODER_HPP_
#define BANYU_MISSION_TASK2_FLASH_DECODER_HPP_

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace banyu_mission::task2 {

class FlashDecoder {
 public:
  // LightNode colour seen this frame, or nullopt when the light is dark.
  void update(std::optional<uint8_t> colour, double now_s);

  // Advanced: the flash colour (LightNode value) once two flashes agree.
  std::optional<uint8_t> colour() const;

  // Disruptive: (first, second) colour once two whole cycles agree.
  std::optional<std::pair<uint8_t, uint8_t>> pair() const;

 private:
  struct Run {
    uint8_t colour;
    double start_s, end_s;
  };
  void closeRun();

  std::optional<uint8_t> lit_;  // colour of the run in progress
  double run_start_s_{0}, last_lit_s_{0};
  std::vector<Run> runs_;       // finished runs, in order
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_FLASH_DECODER_HPP_
