// Bottom-camera detection of the Task 2 pipeline and its light nodes in turbid water.
#ifndef BANYU_DETECTION_PIPELINE_VISION_HPP_
#define BANYU_DETECTION_PIPELINE_VISION_HPP_

#include <optional>
#include <vector>

#include <opencv2/core.hpp>

namespace banyu_detection
{

struct PipeFit
{
  float angle;    // rad; > 0 = pipe leans right further up the image
  float lateral;  // px right of the image centre, at the reference row
};

struct Light
{
  enum Colour { kGreen, kRed, kBlue } colour;  // blue only shows in the post-repair flash
  float x, y;     // px
  float size {0.0f};  // px, longest side of the lit blob
};

// Image rows the fit uses: the stretch ahead in the direction of travel.
struct Band
{
  int begin, end;
};
Band aheadBand(int height);    // moving nose first
Band behindBand(int height);   // moving tail first
inline int referenceRow(int height) {return height / 2;}

// Pipe pixels (CV_8U 0/1). Found as pairs of parallel edges a pipe's width apart with one solid
// body between (whichever way its contrast runs; thin frames and stand bars fail it), then grown
// along the pipe over a faint-contrast mask (smoothed image against a wide median background),
// keeping only the parts the edge pairs touch. `ignore` is where the repair probe rod sits in the
// image (empty: no probe).
cv::Mat pipeMask(const cv::Mat & bgr, const cv::Rect & ignore = {});

// Line fit of the pipe inside `band`; fitting only the stretch ahead takes a zigzag one leg at
// a time. nullopt when too little pipe is in the band.
std::optional<PipeFit> fitPipe(const cv::Mat & mask, Band band);

// Light nodes on the pipe. They are small bright dots (a ring of emitters), not bright areas -
// the vehicle's lights make the pipe itself bright green - so a top-hat keeps features under
// ~11 px, a 3x3 blur stops single noise pixels, and blobs off the pipe are dropped.
std::vector<Light> detectLights(const cv::Mat & bgr, const cv::Mat & mask);

// True when the pipe no longer reaches past the vehicle in the travel direction
// (+1 nose first, -1 tail first).
bool pipeEndsHere(const cv::Mat & mask, int direction);

// Centroid of the pipe pixels, nullopt when too few are in view to hold station on.
std::optional<cv::Point2f> pipeCentroid(const cv::Mat & mask);

cv::Mat drawDebug(
  const cv::Mat & bgr, const cv::Mat & mask, const std::optional<PipeFit> & fit,
  const std::vector<Light> & lights);

}  // namespace banyu_detection

#endif  // BANYU_DETECTION_PIPELINE_VISION_HPP_
