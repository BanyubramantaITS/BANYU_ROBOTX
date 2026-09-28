#include "banyu_detection/pipeline_vision.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace banyu_detection {

namespace {
constexpr int kMinPipeBlobPx = 150;
constexpr int kMinFitPx = 80;
constexpr int kMinHoldPx = 50;
constexpr int kMinLightArea = 30;
constexpr int kCloseLightArea = 150;
}  // namespace

Band aheadBand(int height) { return {40, referenceRow(height) + 20}; }
Band behindBand(int height) { return {referenceRow(height) - 20, height - 40}; }

namespace {

// Pairs of parallel straight edges a pipe's width apart with one solid body between them: the
// pipe, whichever way its contrast runs. Node frames and stand bases are bars too, but thin, and
// the gap between two bars is water like the water outside them. Returns the filled pairs and
// the pipe's width in px (0: none found).
std::pair<cv::Mat, float> edgePairs(const cv::Mat& green, const cv::Mat& rod) {
  const float s = green.cols / 320.0f;  // tuned on 320x240
  // Blur before the local contrast stretch, or it stretches sensor noise into edges everywhere.
  cv::Mat blurred, eq, edges, smooth;
  cv::GaussianBlur(green, blurred, cv::Size(), 2.0 * s);
  cv::createCLAHE(2.0, cv::Size(8, 8))->apply(blurred, eq);
  cv::GaussianBlur(eq, eq, cv::Size(5, 5), 0);
  cv::Canny(eq, edges, 8, 24);
  edges.setTo(0, rod);
  cv::GaussianBlur(blurred, smooth, cv::Size(5, 5), 0);
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 20, 15 * s, 20 * s);

  // Median grey and edge fraction over a strip beside segment (p, d): offsets o0..o1 along n.
  auto strip = [&](cv::Point2f p, cv::Point2f d, cv::Point2f n, float len, float o0, float o1, float& median,
                   float& edge_frac) {
    std::vector<uchar> vals;
    int on_edge = 0;
    for (int i = 0; i < 12; ++i)
      for (int k = 0; k < 5; ++k) {
        const cv::Point2f q = p + d * (len * (0.15f + 0.7f * i / 11)) + n * (o0 + (o1 - o0) * k / 4);
        const int x = cvRound(q.x), y = cvRound(q.y);
        if (x < 0 || y < 0 || x >= green.cols || y >= green.rows) continue;
        vals.push_back(smooth.at<uchar>(y, x));
        on_edge += edges.at<uchar>(y, x) > 0;
      }
    if (vals.empty()) return 0;
    std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
    median = vals[vals.size() / 2];
    edge_frac = static_cast<float>(on_edge) / vals.size();
    return static_cast<int>(vals.size());
  };

  struct Pair {
    float overlap, width;
    int i, j;
  };
  std::vector<Pair> pairs;
  // ponytail: O(n^2) over Hough segments, fine for the few dozen a frame gives
  for (size_t i = 0; i < lines.size(); ++i) {
    const cv::Point2f p(lines[i][0], lines[i][1]), e(lines[i][2], lines[i][3]);
    const float len = cv::norm(e - p);
    const cv::Point2f d = (e - p) / len;
    for (size_t j = i + 1; j < lines.size(); ++j) {
      const cv::Point2f q1(lines[j][0], lines[j][1]), q2(lines[j][2], lines[j][3]);
      const cv::Point2f dj = (q2 - q1) / cv::norm(q2 - q1);
      if (std::fabs(d.dot(dj)) < std::cos(6 * CV_PI / 180)) continue;
      cv::Point2f n(-d.y, d.x);
      float off = ((q1 + q2) / 2 - p).dot(n);
      if (off < 0) {
        n = -n;
        off = -off;
      }
      if (off < 8 * s || off > 90 * s) continue;
      const float t1 = (q1 - p).dot(d), t2 = (q2 - p).dot(d);
      const float overlap = std::min(len, std::max(t1, t2)) - std::max(0.0f, std::min(t1, t2));
      if (overlap < 25 * s) continue;
      float in, left, right, in_edges, unused;
      if (strip(p, d, n, len, 0.25f * off, 0.75f * off, in, in_edges) < 20 ||
          strip(p, d, n, len, -1.2f * off, -0.7f * off, left, unused) < 10 ||
          strip(p, d, n, len, 1.7f * off, 2.2f * off, right, unused) < 10)
        continue;
      const float dl = in - left, dr = in - right;
      if (dl * dr > 0 && std::min(std::fabs(dl), std::fabs(dr)) >= 3 && in_edges < 0.12f)
        pairs.push_back({overlap, off, static_cast<int>(i), static_cast<int>(j)});
    }
  }
  cv::Mat seed = cv::Mat::zeros(green.size(), CV_8U);
  if (pairs.empty()) return {seed, 0.0f};
  const float width = std::max_element(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) {
                        return a.overlap < b.overlap;
                      })->width;
  for (const auto& pr : pairs) {
    if (std::fabs(pr.width - width) > 0.35f * width) continue;
    const auto& a = lines[pr.i];
    const auto& b = lines[pr.j];
    std::vector<cv::Point> quad{{a[0], a[1]}, {a[2], a[3]}, {b[0], b[1]}, {b[2], b[3]}}, hull;
    cv::convexHull(quad, hull);
    cv::fillConvexPoly(seed, hull, 1);
  }
  return {seed, width};
}

}  // namespace

cv::Mat pipeMask(const cv::Mat& bgr, const cv::Rect& ignore) {
  cv::Mat green;
  cv::extractChannel(bgr, green, 1);
  // The probe rod: fill it with the water's level so it is neither pipe nor a dark hole that
  // skews the background around it.
  cv::Mat rod;
  cv::dilate(green < 0.3 * cv::mean(green)[0], rod, cv::Mat::ones(5, 5, CV_8U));
  const cv::Rect area = ignore & cv::Rect(0, 0, bgr.cols, bgr.rows);
  if (area.area() > 0) rod(area).setTo(1);
  const auto [seed, width] = edgePairs(green, rod);
  if (width <= 0) return seed;
  green.setTo(cv::mean(green, rod == 0)[0], rod);
  // Background from a half-size copy: the window is then ~100 px, wider than the pipe even as the
  // vehicle lowers onto a light node.
  cv::Mat small, bg, smooth;
  cv::resize(green, small, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
  cv::medianBlur(small, small, 51);
  cv::resize(small, bg, green.size(), 0, 0, cv::INTER_LINEAR);
  cv::GaussianBlur(green, smooth, cv::Size(9, 9), 0);
  cv::Mat diff;
  cv::subtract(smooth, bg, diff, cv::noArray(), CV_16S);
  // Either way round: under the vehicle's lights the course's dark grey pipe is a silhouette
  // against the lit water where the light misses it and bright where it catches it.
  cv::Mat mask = ((diff >= 3) | (diff <= -3)) & (rod == 0);
  // Opening at most of the pipe's width strips the thin node frames and stand bars off it.
  const int k = std::max(3, static_cast<int>(0.6f * width));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k)));

  // The edge pairs are sure but short (edges break where frames cross the pipe); the contrast
  // mask runs the pipe's whole length but also picks up junk. Keep what the pairs vouch for.
  cv::Mat labels;
  const int n = cv::connectedComponents(mask, labels);
  std::vector<bool> vouched(n, false);
  for (int y = 0; y < labels.rows; ++y)
    for (int x = 0; x < labels.cols; ++x)
      if (seed.at<uchar>(y, x)) vouched[labels.at<int>(y, x)] = true;
  cv::Mat keep = seed.clone();
  for (int i = 1; i < n; ++i)
    if (vouched[i]) keep.setTo(1, labels == i);
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
  cv::Mat tophat_b, tophat_g, tophat_r;
  cv::morphologyEx(ch[0], tophat_b, cv::MORPH_TOPHAT, kernel);
  cv::morphologyEx(ch[1], tophat_g, cv::MORPH_TOPHAT, kernel);
  cv::morphologyEx(ch[2], tophat_r, cv::MORPH_TOPHAT, kernel);
  cv::Mat b16, g16, r16;
  ch[0].convertTo(b16, CV_16S);
  ch[1].convertTo(g16, CV_16S);
  ch[2].convertTo(r16, CV_16S);

  const cv::Mat red = (tophat_r > 20) & (r16 > g16);
  const cv::Mat green = (tophat_g > 25) & (g16 > 2 * r16) & (g16 > b16);
  const cv::Mat blue = (tophat_b > 20) & (b16 > g16) & (b16 > r16);

  // The repair probe is a black rod in view: where it covers the lit pipe it leaves a narrow bright
  // sliver, and bits of pipe edge beside its tip, that the top-hat takes for lights. A blob
  // touching anything far darker than the water only counts as a big round one: the ring up
  // close, crossed by the rod.
  cv::Mat rod;
  cv::dilate(ch[1] < 0.3 * cv::mean(ch[1])[0], rod, cv::Mat::ones(5, 5, CV_8U));

  std::vector<Light> lights;
  for (const auto& [colour, m] : {std::pair<Light::Colour, cv::Mat>{Light::kRed, red}, {Light::kGreen, green},
                                  {Light::kBlue, blue}}) {
    cv::Mat grown, labels, stats, centroids;
    cv::dilate(m, grown, cv::Mat::ones(5, 5, CV_8U));
    const int n = cv::connectedComponentsWithStats(grown, labels, stats, centroids);
    for (int i = 1; i < n; ++i) {
      // Box centre, not centroid: up close the probe rod hides a slice of the ring and would pull
      // the centroid off the switch at its centre.
      const float x = stats.at<int>(i, cv::CC_STAT_LEFT) + stats.at<int>(i, cv::CC_STAT_WIDTH) / 2.0f;
      const float y = stats.at<int>(i, cv::CC_STAT_TOP) + stats.at<int>(i, cv::CC_STAT_HEIGHT) / 2.0f;
      const int area = stats.at<int>(i, cv::CC_STAT_AREA);
      const int w = stats.at<int>(i, cv::CC_STAT_WIDTH), h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
      // Up close (repair) the pipe is wider than pipeMask's background window and drops out, but
      // the ring is then a big clear blob no noise makes.
      const bool on_pipe = near_pipe.at<uchar>(static_cast<int>(y), static_cast<int>(x)) || area >= kCloseLightArea;
      const bool sliver = (area < kCloseLightArea || std::max(w, h) > 2.5 * std::min(w, h)) &&
                          cv::countNonZero((labels == i) & rod) > 0;
      if (area >= kMinLightArea && on_pipe && !sliver) {
        lights.push_back({colour, x, y});
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
               l.colour == Light::kRed    ? cv::Scalar(0, 0, 255)
               : l.colour == Light::kBlue ? cv::Scalar(255, 0, 0)
                                          : cv::Scalar(0, 255, 0),
               1);
  }
  return vis;
}

}  // namespace banyu_detection
