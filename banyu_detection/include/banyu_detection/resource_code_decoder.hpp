// Reads the post-repair flash code on a Task 2 light node from a time series of ring colours.
//
// Advanced tier: after the repair the node shows steady GREEN for 5 s, then flashes one colour
// (the delivery circle) 1 s on / 1 s off for 30 s. Disruptive tier: same, but a 5 s cycle of
// colour 1 (the tin) 1 s, off 1 s, colour 2 (the circle) 1 s, dark 2 s.
#ifndef BANYU_DETECTION_RESOURCE_CODE_DECODER_HPP_
#define BANYU_DETECTION_RESOURCE_CODE_DECODER_HPP_

#include <vector>

namespace banyu_detection
{

// Colour codes match banyu_msgs/PipelineRingObservation and rx_msgs/Color: 0 dark or unknown,
// 1 red, 2 green, 3 blue.
struct CodeSample
{
  double t;      // seconds
  int colour;
};

struct ResourceCodeConfig
{
  int code_length {2};            // 1 = Advanced (circle only), 2 = Disruptive (tin, circle)
  double min_on_s {0.5};          // a flash lasts about 1 s
  double max_on_s {1.7};
  double steady_on_s {2.5};       // a longer run is the steady post-repair green, not a flash
  double long_gap_s {1.6};        // dark time between two cycles (nominally 2 s)
  double merge_gap_s {0.25};      // a dropout this short inside one colour does not split the run
  int min_cycles {2};             // matching cycles needed before the code is called complete
};

struct ResourceCodeResult
{
  bool complete {false};
  bool repair_verified {false};   // the node showed steady green for longer than steady_on_s
  int tin {0};                    // 0 when code_length is 1
  int circle {0};
  int cycles {0};                 // cycles that agree with the winning code
  int cycles_seen {0};            // cycles read at all
  float confidence {0.0f};        // agreeing cycles / cycles read
};

class ResourceCodeDecoder
{
public:
  explicit ResourceCodeDecoder(const ResourceCodeConfig & config = {});

  void reset();
  void add(double t, int colour);
  std::size_t size() const {return samples_.size();}

  // Decode everything added so far. Samples may arrive slightly out of order.
  ResourceCodeResult decode() const;

private:
  ResourceCodeConfig config_;
  std::vector<CodeSample> samples_;
};

}  // namespace banyu_detection

#endif  // BANYU_DETECTION_RESOURCE_CODE_DECODER_HPP_
