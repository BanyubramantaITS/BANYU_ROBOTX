// Bottom-camera detection of the Task 2 pipeline and its light nodes in turbid water.
#ifndef BANYU_DETECTION_PIPELINE_VISION_HPP_
#define BANYU_DETECTION_PIPELINE_VISION_HPP_

#include <optional>
#include <vector>

#include <opencv2/core.hpp>

namespace banyu_detection {

struct PipeFit {
  float angle;    // rad; > 0 = pipe leans right further up the image
  float lateral;  // px right of the image centre, at the reference row
};

struct Light {
  bool red;
  float x, y;     // px
};

// Image rows the fit uses: the stretch ahead in the direction of travel.
struct Band {
  int begin, end;
};
Band aheadBand(int height);    // moving nose first
Band behindBand(int height);   // moving tail first
inline int referenceRow(int height) { return height / 2; }

// Pipe pixels (CV_8U 0/1): a faint ridge above the water veil. Contrast is only 5-8 DN, about
// sensor noise, so it is thresholded on a 9x9-smoothed copy against a 51 px median background.
cv::Mat pipeMask(const cv::Mat& bgr);

// Line fit of the pipe inside `band`; fitting only the stretch ahead takes a zigzag one leg at
// a time. nullopt when too little pipe is in the band.
std::optional<PipeFit> fitPipe(const cv::Mat& mask, Band band);

// Light nodes on the pipe. They are small bright dots (a ring of emitters), not bright areas -
// the vehicle's lights make the pipe itself bright green - so a top-hat keeps features under
// ~11 px, a 3x3 blur stops single noise pixels, and blobs off the pipe are dropped.
std::vector<Light> detectLights(const cv::Mat& bgr, const cv::Mat& mask);

// True when the pipe no longer reaches past the vehicle in the travel direction
// (+1 nose first, -1 tail first).
bool pipeEndsHere(const cv::Mat& mask, int direction);

// Centroid of the pipe pixels, nullopt when too few are in view to hold station on.
std::optional<cv::Point2f> pipeCentroid(const cv::Mat& mask);

cv::Mat drawDebug(const cv::Mat& bgr, const cv::Mat& mask, const std::optional<PipeFit>& fit,
                  const std::vector<Light>& lights);

}  // namespace banyu_detection

#endif  // BANYU_DETECTION_PIPELINE_VISION_HPP_
