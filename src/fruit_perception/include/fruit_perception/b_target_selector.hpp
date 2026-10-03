#ifndef FRUIT_PERCEPTION__B_TARGET_SELECTOR_HPP_
#define FRUIT_PERCEPTION__B_TARGET_SELECTOR_HPP_

#include <opencv2/core.hpp>

#include <cstddef>
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
};

struct BTargetSelectorConfig
{
  std::size_t acquisition_frames;
  std::size_t max_targets;
  double association_radius_m;
  double outlier_sigma;
  std::size_t min_observations{6U};
  double type_support_ratio{0.75};
  std::size_t max_missed_frames{2U};
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

  BTargetUpdate update(const std::vector<BTargetCandidate> & candidates);
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
  };

  struct Track
  {
    std::vector<Observation> observations;
    cv::Point3d point_sum{};
    std::size_t last_seen_frame{0U};
  };

  struct TrackStatistics
  {
    BTargetResult result;
    double confidence{0.0};
  };

  void add_frame(const std::vector<BTargetCandidate> & candidates);
  std::vector<TrackStatistics> select_tracks() const;
  static cv::Point3d mean_point(const Track & track);

  BTargetSelectorConfig config_;
  std::size_t frame_count_{0U};
  std::size_t last_candidate_frame_{0U};
  std::vector<Track> tracks_;
  bool ready_{false};
  std::vector<BTargetResult> locked_targets_;
};

}  // namespace fruit_perception

#endif  // FRUIT_PERCEPTION__B_TARGET_SELECTOR_HPP_
