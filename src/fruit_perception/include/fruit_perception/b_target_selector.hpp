#ifndef FRUIT_PERCEPTION__B_TARGET_SELECTOR_HPP_
#define FRUIT_PERCEPTION__B_TARGET_SELECTOR_HPP_

#include <opencv2/core.hpp>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace fruit_perception
{

struct BTargetCandidate
{
  std::string type;
  cv::Point3d point;
  cv::Point2d pixel_center;
  std::size_t image_width;
  double confidence;
  double maturity_confidence{1.0};
  cv::Rect roi{};
  bool has_point{true};
  bool bad{false};
};

struct BTargetSelectorConfig
{
  std::size_t acquisition_frames;
  double association_radius_m;
  double outlier_sigma;
  double maturity_confidence_threshold{0.70};
};

struct BTargetResult
{
  std::string type;
  cv::Point3d point{};
  cv::Point2d pixel_center{};
  std::size_t image_width{0U};
  std::size_t observation_count{0U};
  std::size_t inlier_count{0U};
  double confidence{0.0};
  bool has_point{false};
};

struct BTargetUpdate
{
  bool ready{false};
  std::vector<BTargetResult> targets;
};

class BTargetSelector
{
public:
  explicit BTargetSelector(BTargetSelectorConfig config);

  // Shared A/B/C voting. Defects use original-image pixel coordinates.
  BTargetUpdate update(
    const std::vector<BTargetCandidate> & candidates,
    const std::vector<cv::Rect> & defects = {});
  void reset();

private:
  struct Observation
  {
    std::string type;
    cv::Point3d point;
    cv::Point2d pixel_center;
    std::size_t image_width;
    double confidence;
    double maturity_confidence;
    std::size_t frame;
    cv::Rect roi;
    bool has_point;
  };

  struct Track
  {
    std::vector<Observation> observations;
    cv::Point3d point_sum{};
    std::size_t last_seen_frame{0U};
    bool bad{false};
  };

  void add_frame(const std::vector<BTargetCandidate> & candidates);
  std::vector<BTargetResult> select_tracks() const;
  static cv::Point3d mean_point(const Track & track);

  BTargetSelectorConfig config_;
  std::size_t frame_count_{0U};
  std::vector<Track> tracks_;
  bool ready_{false};
  std::vector<BTargetResult> locked_targets_;
  std::vector<cv::Rect> defects_;
};

bool defect_belongs_to_fruit(const cv::Rect & defect, const cv::Rect & fruit);

std::optional<BTargetResult> select_primary_target(
  const std::vector<BTargetResult> & targets, double confidence_weight,
  double score_close_threshold);

}  // namespace fruit_perception

#endif  // FRUIT_PERCEPTION__B_TARGET_SELECTOR_HPP_
