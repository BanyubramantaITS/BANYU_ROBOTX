#include "banyu_detection/pipeline_vision.hpp"

#include <cmath>

#include <opencv2/imgproc.hpp>

namespace banyu_detection {

namespace {
constexpr int kMinPipeBlobPx = 150;
constexpr int kMinFitPx = 80;
constexpr int kMinHoldPx = 50;
constexpr int kMinLightArea = 30;
}  // namespace

Band aheadBand(int height) { return {40, referenceRow(height) + 20}; }
Band behindBand(int height) { return {referenceRow(height) - 20, height - 40}; }

cv::Mat pipeMask(const cv::Mat& bgr) {
  cv::Mat green, bg, smooth;
  cv::extractChannel(bgr, green, 1);
  cv::medianBlur(green, bg, 51);
  cv::GaussianBlur(green, smooth, cv::Size(9, 9), 0);
  cv::Mat diff;
  cv::subtract(smooth, bg, diff, cv::noArray(), CV_16S);
  cv::Mat mask = diff >= 3;
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat::ones(5, 5, CV_8U));

  cv::Mat labels, stats, centroids;
  const int n = cv::connectedComponentsWithStats(mask, labels, stats, centroids);
  cv::Mat keep = cv::Mat::zeros(mask.size(), CV_8U);
  for (int i = 1; i < n; ++i) {
    if (stats.at<int>(i, cv::CC_STAT_AREA) >= kMinPipeBlobPx) keep.setTo(1, labels == i);
  }
  return keep;
}

std::optional<PipeFit> fitPipe(const cv::Mat& mask, Band band) {
  std::vector<cv::Point> pts;
  cv::findNonZero(mask.rowRange(band.begin, band.end), pts);
  if (static_cast<int>(pts.size()) < kMinFitPx) return std::nullopt;
  for (auto& p : pts) p.y += band.begin;
  cv::Vec4f line;
  cv::fitLine(pts, line, cv::DIST_HUBER, 0, 0.01, 0.01);
  float vx = line[0], vy = line[1];
  if (vy > 0) {  // point up the image
    vx = -vx;
    vy = -vy;
  }
  const float row = static_cast<float>(referenceRow(mask.rows));
  const float lateral = std::fabs(vy) > 1e-3f ? line[2] + (row - line[3]) * vx / vy - mask.cols / 2.0f : 0.0f;
  return PipeFit{std::atan2(vx, -vy), lateral};
}

std::vector<Light> detectLights(const cv::Mat& bgr, const cv::Mat& mask) {
  static const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11));
  cv::Mat near_pipe, sm;
  cv::dilate(mask, near_pipe, cv::Mat::ones(31, 31, CV_8U));
  cv::GaussianBlur(bgr, sm, cv::Size(3, 3), 0);
  cv::Mat ch[3];
  cv::split(sm, ch);
  cv::Mat tophat_g, tophat_r;
  cv::morphologyEx(ch[1], tophat_g, cv::MORPH_TOPHAT, kernel);
  cv::morphologyEx(ch[2], tophat_r, cv::MORPH_TOPHAT, kernel);
  cv::Mat g16, r16;
  ch[1].convertTo(g16, CV_16S);
  ch[2].convertTo(r16, CV_16S);

  const cv::Mat red = (tophat_r > 20) & (r16 > g16);
  const cv::Mat green = (tophat_g > 25) & (g16 > 2 * r16);

  std::vector<Light> lights;
  for (const auto& [is_red, m] : {std::pair<bool, cv::Mat>{true, red}, {false, green}}) {
    cv::Mat grown, labels, stats, centroids;
    cv::dilate(m, grown, cv::Mat::ones(5, 5, CV_8U));
    const int n = cv::connectedComponentsWithStats(grown, labels, stats, centroids);
    for (int i = 1; i < n; ++i) {
      const float x = static_cast<float>(centroids.at<double>(i, 0));
      const float y = static_cast<float>(centroids.at<double>(i, 1));
      if (stats.at<int>(i, cv::CC_STAT_AREA) >= kMinLightArea &&
          near_pipe.at<uchar>(static_cast<int>(y), static_cast<int>(x))) {
        lights.push_back({is_red, x, y});
      }
    }
  }
  return lights;
}

bool pipeEndsHere(const cv::Mat& mask, int direction) {
  std::vector<cv::Point> pts;
  cv::findNonZero(mask, pts);
  if (static_cast<int>(pts.size()) <= kMinPipeBlobPx) return false;
  int top = mask.rows, bottom = 0;
  for (const auto& p : pts) {
    top = std::min(top, p.y);
    bottom = std::max(bottom, p.y);
  }
  const int row = referenceRow(mask.rows);
  return direction > 0 ? top > row - 30 : bottom < row + 30;
}

std::optional<cv::Point2f> pipeCentroid(const cv::Mat& mask) {
  const cv::Moments m = cv::moments(mask, true);
  if (m.m00 <= kMinHoldPx) return std::nullopt;
  return cv::Point2f(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
}

cv::Mat drawDebug(const cv::Mat& bgr, const cv::Mat& mask, const std::optional<PipeFit>& fit,
                  const std::vector<Light>& lights) {
  cv::Mat vis;
  cv::normalize(bgr, vis, 0, 255, cv::NORM_MINMAX);
  cv::Mat tint(vis.size(), vis.type(), cv::Scalar(255, 0, 255));
  cv::Mat blended;
  cv::addWeighted(vis, 0.5, tint, 0.5, 0, blended);
  blended.copyTo(vis, mask);
  const int row = referenceRow(bgr.rows);
  if (fit) {
    const cv::Point a(static_cast<int>(bgr.cols / 2 + fit->lateral), row);
    const cv::Point b(static_cast<int>(a.x + 80 * std::sin(fit->angle)), static_cast<int>(row - 80 * std::cos(fit->angle)));
    cv::line(vis, a, b, cv::Scalar(0, 255, 255), 2);
  }
  for (const auto& l : lights) {
    cv::circle(vis, cv::Point(static_cast<int>(l.x), static_cast<int>(l.y)), 8,
               l.red ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 1);
  }
  return vis;
}

}  // namespace banyu_detection
