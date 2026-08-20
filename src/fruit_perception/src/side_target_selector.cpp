#include "fruit_perception/side_target_selector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace fruit_perception
{
namespace
{

double intersection_over_union(const cv::Rect2d & left, const cv::Rect2d & right)
{
  const cv::Rect2d intersection = left & right;
  const double union_area = left.area() + right.area() - intersection.area();
  return union_area > 0.0 ? intersection.area() / union_area : 0.0;
}

double median(std::vector<double> values)
{
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2U);
  std::nth_element(values.begin(), middle, values.end());
  if (values.size() % 2U != 0U) {
    return *middle;
  }
  const double upper = *middle;
  const double lower = *std::max_element(values.begin(), middle);
  return (lower + upper) * 0.5;
}

void validate_config(const SideTargetSelectorConfig & config)
{
  if (!std::isfinite(config.acquisition_duration_sec) ||
    config.acquisition_duration_sec <= 0.0)
  {
    throw std::invalid_argument("side_target_acquisition_sec must be positive and finite");
  }
  if (!std::isfinite(config.association_iou_threshold) ||
    config.association_iou_threshold <= 0.0 || config.association_iou_threshold > 1.0)
  {
    throw std::invalid_argument(
            "side_target_association_iou_threshold must be in (0, 1]");
  }
  if (config.lock_miss_frames == 0U) {
    throw std::invalid_argument("side_target_lock_miss_frames must be positive");
  }
  if (!std::isfinite(config.confidence_weight) ||
    config.confidence_weight < 0.0 || config.confidence_weight > 1.0)
  {
    throw std::invalid_argument("side_target_confidence_weight must be between 0 and 1");
  }
  if (!std::isfinite(config.score_close_threshold) ||
    config.score_close_threshold < 0.0 || config.score_close_threshold > 1.0)
  {
    throw std::invalid_argument("side_target_score_close_threshold must be between 0 and 1");
  }
}

}  // namespace

SideTargetSelector::SideTargetSelector(SideTargetSelectorConfig config)
: config_(std::move(config))
{
  validate_config(config_);
}

SideTargetUpdate SideTargetSelector::update(
  const std::vector<SideTargetCandidate> & candidates)
{
  SideTargetUpdate result;
  if (locked_type_) {
    const auto matched = match_locked_target(candidates);
    if (matched) {
      locked_roi_ = matched->roi;
      missed_frame_count_ = 0U;
      result.type = *locked_type_;
      return result;
    }

    ++missed_frame_count_;
    if (missed_frame_count_ < config_.lock_miss_frames) {
      return result;
    }
    result.lock_lost = true;
    reset();
  }

  const auto now = std::chrono::steady_clock::now();
  if (!acquisition_started_at_) {
    acquisition_started_at_ = now;
    add_acquisition_frame(candidates);
    return result;
  }
  if (std::chrono::duration<double>(now - *acquisition_started_at_).count() <
    config_.acquisition_duration_sec)
  {
    add_acquisition_frame(candidates);
    return result;
  }

  const auto selected = select_track();
  if (!selected) {
    reset();
    return result;
  }

  locked_type_ = selected->track->type;
  locked_roi_ = selected->track->observations.back().roi;
  missed_frame_count_ = acquisition_frame_count_ - selected->track->last_seen_frame;
  result.lock_acquired = true;
  result.observation_count = selected->track->observations.size();
  result.score = selected->score;
  result.confidence = selected->confidence;
  result.horizontal_centrality = selected->horizontal_centrality;
  result.reference_roi = selected->mean_roi;
  result.type = *locked_type_;
  tracks_.clear();
  acquisition_started_at_.reset();
  acquisition_frame_count_ = 0U;
  return result;
}

void SideTargetSelector::reset()
{
  acquisition_started_at_.reset();
  acquisition_frame_count_ = 0U;
  tracks_.clear();
  locked_type_.reset();
  locked_roi_ = {};
  missed_frame_count_ = 0U;
}

cv::Rect2d SideTargetSelector::mean_roi(const Track & track)
{
  const cv::Vec4d mean =
    track.roi_sum * (1.0 / static_cast<double>(track.observations.size()));
  return cv::Rect2d(mean[0], mean[1], mean[2], mean[3]);
}

void SideTargetSelector::add_acquisition_frame(
  const std::vector<SideTargetCandidate> & candidates)
{
  ++acquisition_frame_count_;
  for (const auto & candidate : candidates) {
    const cv::Rect2d candidate_roi = candidate.roi;
    Track * best_track = nullptr;
    double best_iou = config_.association_iou_threshold;
    for (auto & track : tracks_) {
      if (track.type != candidate.type ||
        track.last_seen_frame == acquisition_frame_count_)
      {
        continue;
      }
      const double iou = intersection_over_union(candidate_roi, mean_roi(track));
      if (iou >= config_.association_iou_threshold &&
        (best_track == nullptr || iou > best_iou))
      {
        best_track = &track;
        best_iou = iou;
      }
    }

    const Observation observation{
      candidate_roi, candidate.confidence, candidate.horizontal_centrality};
    if (best_track == nullptr) {
      Track track;
      track.type = candidate.type;
      track.observations.push_back(observation);
      track.roi_sum = cv::Vec4d(
        candidate_roi.x, candidate_roi.y, candidate_roi.width, candidate_roi.height);
      track.last_seen_frame = acquisition_frame_count_;
      tracks_.push_back(std::move(track));
      continue;
    }

    best_track->observations.push_back(observation);
    best_track->roi_sum += cv::Vec4d(
      candidate_roi.x, candidate_roi.y, candidate_roi.width, candidate_roi.height);
    best_track->last_seen_frame = acquisition_frame_count_;
  }
}

std::optional<SideTargetSelector::TrackStatistics> SideTargetSelector::select_track() const
{
  std::vector<TrackStatistics> eligible;
  for (const auto & track : tracks_) {
    std::vector<double> confidences;
    std::vector<double> centralities;
    confidences.reserve(track.observations.size());
    centralities.reserve(track.observations.size());
    for (const auto & observation : track.observations) {
      confidences.push_back(observation.confidence);
      centralities.push_back(observation.horizontal_centrality);
    }
    const double confidence = median(std::move(confidences));
    const double centrality = median(std::move(centralities));
    const double score =
      config_.confidence_weight * confidence +
      (1.0 - config_.confidence_weight) * centrality;
    eligible.push_back(
      TrackStatistics{&track, mean_roi(track), confidence, centrality, score});
  }
  if (eligible.empty()) {
    return std::nullopt;
  }

  const double best_score = std::max_element(
    eligible.begin(), eligible.end(),
    [](const auto & left, const auto & right) {return left.score < right.score;})->score;
  const TrackStatistics * selected = nullptr;
  for (const auto & candidate : eligible) {
    if (best_score - candidate.score > config_.score_close_threshold) {
      continue;
    }
    if (selected == nullptr || candidate.confidence > selected->confidence) {
      selected = &candidate;
    }
  }
  return *selected;
}

std::optional<SideTargetCandidate> SideTargetSelector::match_locked_target(
  const std::vector<SideTargetCandidate> & candidates) const
{
  const SideTargetCandidate * best = nullptr;
  double best_iou = config_.association_iou_threshold;
  for (const auto & candidate : candidates) {
    if (candidate.type != *locked_type_) {
      continue;
    }
    const double iou = intersection_over_union(candidate.roi, locked_roi_);
    if (iou >= config_.association_iou_threshold &&
      (best == nullptr || iou > best_iou))
    {
      best = &candidate;
      best_iou = iou;
    }
  }
  if (best == nullptr) {
    return std::nullopt;
  }
  return *best;
}

}  // namespace fruit_perception
