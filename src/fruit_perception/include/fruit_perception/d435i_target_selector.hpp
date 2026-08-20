#ifndef FRUIT_PERCEPTION__D435I_TARGET_SELECTOR_HPP_
#define FRUIT_PERCEPTION__D435I_TARGET_SELECTOR_HPP_

#include <opencv2/core.hpp>

#include <cstddef>
#include <optional>
#include <vector>

namespace fruit_perception
{

struct D435iTargetCandidate
{
  cv::Point3d point;
  double confidence;
  double horizontal_centrality;
};

struct D435iTargetSelectorConfig
{
  std::size_t acquisition_frames;
  double association_radius_m;
  double outlier_sigma;
  std::size_t lock_miss_frames;
  double confidence_weight;
  double score_close_threshold;
};

struct D435iTargetUpdate
{
  std::optional<cv::Point3d> point;
  bool lock_acquired{false};
  bool lock_lost{false};
  std::size_t observation_count{0U};
  std::size_t inlier_count{0U};
  double score{0.0};
  double confidence{0.0};
  double horizontal_centrality{0.0};
  cv::Point3d reference_point{};
  cv::Point3d stddev{};
};

class D435iTargetSelector
{
public:
  explicit D435iTargetSelector(D435iTargetSelectorConfig config);

  D435iTargetUpdate update(const std::vector<D435iTargetCandidate> & candidates);
  void reset();

private:
  struct Observation
  {
    cv::Point3d point;
    double confidence;
    double horizontal_centrality;
  };

  struct Track
  {
    std::vector<Observation> observations;
    cv::Point3d point_sum{};
    std::size_t last_seen_frame{0U};
  };

  struct TrackStatistics
  {
    const Track * track;
    cv::Point3d mean;
    cv::Point3d stddev;
    std::size_t inlier_count;
    double confidence;
    double horizontal_centrality;
    double score;
  };

  void add_acquisition_frame(const std::vector<D435iTargetCandidate> & candidates);
  std::optional<TrackStatistics> select_track() const;
  std::optional<D435iTargetCandidate> match_locked_target(
    const std::vector<D435iTargetCandidate> & candidates) const;
  static cv::Point3d mean_point(const Track & track);

  D435iTargetSelectorConfig config_;
  std::size_t acquisition_frame_count_{0U};
  std::vector<Track> tracks_;
  std::optional<cv::Point3d> locked_point_;
  std::size_t missed_frame_count_{0U};
};

}  // namespace fruit_perception

#endif  // FRUIT_PERCEPTION__D435I_TARGET_SELECTOR_HPP_
