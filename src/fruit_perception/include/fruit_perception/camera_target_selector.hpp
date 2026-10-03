#ifndef FRUIT_PERCEPTION__CAMERA_TARGET_SELECTOR_HPP_
#define FRUIT_PERCEPTION__CAMERA_TARGET_SELECTOR_HPP_

#include <opencv2/core.hpp>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace fruit_perception
{

struct CameraTargetCandidate
{
  cv::Point3d point;
  double confidence;
  double horizontal_centrality;
  std::string type;
  double maturity_confidence{1.0};
};

struct CameraTargetSelectorConfig
{
  std::size_t acquisition_frames;
  double association_radius_m;
  double outlier_sigma;
  std::size_t lock_miss_frames;
  double confidence_weight;
  double score_close_threshold;
  std::size_t min_observations{6U};
  double type_support_ratio{0.75};
  std::size_t max_missed_frames{2U};
  double maturity_confidence_threshold{0.70};
};

struct CameraTargetUpdate
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

class CameraTargetSelector
{
public:
  explicit CameraTargetSelector(CameraTargetSelectorConfig config);

  CameraTargetUpdate update(
    const std::vector<CameraTargetCandidate> & candidates,
    const std::string & expected_type = "");
  void reset();

private:
  struct Observation
  {
    cv::Point3d point;
    double confidence;
    double horizontal_centrality;
    std::string type;
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
    const Track * track;
    cv::Point3d mean;
    cv::Point3d stddev;
    std::size_t inlier_count;
    double confidence;
    double horizontal_centrality;
    double score;
    std::string type;
  };

  void add_acquisition_frame(const std::vector<CameraTargetCandidate> & candidates);
  std::optional<TrackStatistics> select_track(const std::string & expected_type) const;
  std::optional<CameraTargetCandidate> match_locked_target(
    const std::vector<CameraTargetCandidate> & candidates) const;
  static cv::Point3d mean_point(const Track & track);

  CameraTargetSelectorConfig config_;
  std::size_t acquisition_frame_count_{0U};
  std::vector<Track> tracks_;
  std::optional<cv::Point3d> locked_point_;
  std::string locked_type_;
  std::size_t missed_frame_count_{0U};
};

}  // namespace fruit_perception

#endif  // FRUIT_PERCEPTION__CAMERA_TARGET_SELECTOR_HPP_
