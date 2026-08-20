#ifndef FRUIT_PERCEPTION__SIDE_TARGET_SELECTOR_HPP_
#define FRUIT_PERCEPTION__SIDE_TARGET_SELECTOR_HPP_

#include <opencv2/core.hpp>

#include <chrono>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace fruit_perception
{

struct SideTargetCandidate
{
  std::string type;
  cv::Rect roi;
  double confidence;
  double horizontal_centrality;
};

struct SideTargetSelectorConfig
{
  double acquisition_duration_sec;
  double association_iou_threshold;
  std::size_t lock_miss_frames;
  double confidence_weight;
  double score_close_threshold;
};

struct SideTargetUpdate
{
  std::optional<std::string> type;
  bool lock_acquired{false};
  bool lock_lost{false};
  std::size_t observation_count{0U};
  double score{0.0};
  double confidence{0.0};
  double horizontal_centrality{0.0};
  cv::Rect2d reference_roi{};
};

class SideTargetSelector
{
public:
  explicit SideTargetSelector(SideTargetSelectorConfig config);

  SideTargetUpdate update(const std::vector<SideTargetCandidate> & candidates);
  void reset();

private:
  struct Observation
  {
    cv::Rect2d roi;
    double confidence;
    double horizontal_centrality;
  };

  struct Track
  {
    std::string type;
    std::vector<Observation> observations;
    cv::Vec4d roi_sum{};
    std::size_t last_seen_frame{0U};
  };

  struct TrackStatistics
  {
    const Track * track;
    cv::Rect2d mean_roi;
    double confidence;
    double horizontal_centrality;
    double score;
  };

  void add_acquisition_frame(const std::vector<SideTargetCandidate> & candidates);
  std::optional<TrackStatistics> select_track() const;
  std::optional<SideTargetCandidate> match_locked_target(
    const std::vector<SideTargetCandidate> & candidates) const;
  static cv::Rect2d mean_roi(const Track & track);

  SideTargetSelectorConfig config_;
  std::optional<std::chrono::steady_clock::time_point> acquisition_started_at_;
  std::size_t acquisition_frame_count_{0U};
  std::vector<Track> tracks_;
  std::optional<std::string> locked_type_;
  cv::Rect2d locked_roi_{};
  std::size_t missed_frame_count_{0U};
};

}  // namespace fruit_perception

#endif  // FRUIT_PERCEPTION__SIDE_TARGET_SELECTOR_HPP_
