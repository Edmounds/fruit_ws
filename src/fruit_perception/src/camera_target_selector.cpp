#include "fruit_perception/camera_target_selector.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace fruit_perception
{
namespace
{

double squared_distance(const cv::Point3d & left, const cv::Point3d & right)
{
  const cv::Point3d difference = left - right;
  return difference.dot(difference);
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

void validate_config(const CameraTargetSelectorConfig & config)
{
  if (config.acquisition_frames == 0U) {
    throw std::invalid_argument("camera_target_acquisition_frames must be positive");
  }
  if (!std::isfinite(config.association_radius_m) || config.association_radius_m <= 0.0) {
    throw std::invalid_argument(
            "camera_target_association_radius_m must be positive and finite");
  }
  if (!std::isfinite(config.outlier_sigma) || config.outlier_sigma <= 0.0) {
    throw std::invalid_argument("camera_target_outlier_sigma must be positive and finite");
  }
  if (config.lock_miss_frames == 0U) {
    throw std::invalid_argument("camera_target_lock_miss_frames must be positive");
  }
  if (config.min_observations == 0U || config.min_observations > config.acquisition_frames ||
    !std::isfinite(config.type_support_ratio) || config.type_support_ratio <= 0.5 ||
    config.type_support_ratio > 1.0 || config.max_missed_frames >= config.acquisition_frames ||
    !std::isfinite(config.maturity_confidence_threshold) ||
    config.maturity_confidence_threshold < 0.5 || config.maturity_confidence_threshold > 1.0)
  {
    throw std::invalid_argument("Invalid camera target confirmation configuration");
  }
  if (!std::isfinite(config.confidence_weight) ||
    config.confidence_weight < 0.0 || config.confidence_weight > 1.0)
  {
    throw std::invalid_argument("camera_target_confidence_weight must be between 0 and 1");
  }
  if (!std::isfinite(config.score_close_threshold) ||
    config.score_close_threshold < 0.0 || config.score_close_threshold > 1.0)
  {
    throw std::invalid_argument("camera_target_score_close_threshold must be between 0 and 1");
  }
}

}  // namespace

CameraTargetSelector::CameraTargetSelector(CameraTargetSelectorConfig config)
: config_(std::move(config))
{
  validate_config(config_);
}

CameraTargetUpdate CameraTargetSelector::update(
  const std::vector<CameraTargetCandidate> & candidates,
  const std::string & expected_type)
{
  CameraTargetUpdate result;
  if (locked_point_) {
    const auto matched = match_locked_target(candidates);
    if (matched) {
      missed_frame_count_ = 0U;
      result.point = locked_point_;
      return result;
    }

    ++missed_frame_count_;
    if (missed_frame_count_ < config_.lock_miss_frames) {
      return result;
    }
    result.lock_lost = true;
    reset();
  }

  add_acquisition_frame(candidates);
  const auto selected = select_track(expected_type);
  if (!selected) {
    return result;
  }
  result.observation_count = selected->track->observations.size();
  result.inlier_count = selected->inlier_count;
  result.score = selected->score;
  result.confidence = selected->confidence;
  result.horizontal_centrality = selected->horizontal_centrality;
  result.reference_point = selected->mean;
  result.stddev = selected->stddev;
  locked_point_ = selected->mean;
  locked_type_ = selected->type;
  missed_frame_count_ = acquisition_frame_count_ - selected->track->last_seen_frame;
  result.lock_acquired = true;
  result.point = selected->mean;
  tracks_.clear();
  acquisition_frame_count_ = 0U;
  return result;
}

cv::Point3d CameraTargetSelector::mean_point(const Track & track)
{
  return track.point_sum * (1.0 / static_cast<double>(track.observations.size()));
}

void CameraTargetSelector::reset()
{
  acquisition_frame_count_ = 0U;
  tracks_.clear();
  locked_point_.reset();
  locked_type_.clear();
  missed_frame_count_ = 0U;
}

void CameraTargetSelector::add_acquisition_frame(
  const std::vector<CameraTargetCandidate> & candidates)
{
  ++acquisition_frame_count_;
  for (auto & track : tracks_) {
    auto & observations = track.observations;
    observations.erase(
      std::remove_if(
        observations.begin(), observations.end(), [this](const auto & observation) {
          return acquisition_frame_count_ - observation.frame >= config_.acquisition_frames;
        }), observations.end());
    track.point_sum = {};
    for (const auto & observation : observations) {
      track.point_sum += observation.point;
    }
  }
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(), [this](const auto & track) {
        return track.observations.empty() ||
        acquisition_frame_count_ - track.last_seen_frame > config_.max_missed_frames + 1U;
      }), tracks_.end());
  const double radius_squared = config_.association_radius_m * config_.association_radius_m;
  for (const auto & candidate : candidates) {
    Track * closest = nullptr;
    double closest_distance = radius_squared;
    for (auto & track : tracks_) {
      if (track.last_seen_frame == acquisition_frame_count_) {
        continue;
      }
      const double distance = squared_distance(candidate.point, mean_point(track));
      if (distance <= radius_squared &&
        (closest == nullptr || distance < closest_distance))
      {
        closest = &track;
        closest_distance = distance;
      }
    }

    const Observation observation{
      candidate.point, candidate.confidence, candidate.horizontal_centrality,
      candidate.type, candidate.maturity_confidence, acquisition_frame_count_};
    if (closest == nullptr) {
      Track track;
      track.observations.push_back(observation);
      track.point_sum = candidate.point;
      track.last_seen_frame = acquisition_frame_count_;
      tracks_.push_back(std::move(track));
      continue;
    }
    closest->observations.push_back(observation);
    closest->point_sum += candidate.point;
    closest->last_seen_frame = acquisition_frame_count_;
  }
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(), [this](const auto & track) {
        return acquisition_frame_count_ - track.last_seen_frame > config_.max_missed_frames;
      }), tracks_.end());
}

std::optional<CameraTargetSelector::TrackStatistics> CameraTargetSelector::select_track(
  const std::string & expected_type) const
{
  std::vector<TrackStatistics> eligible;
  for (const auto & track : tracks_) {
    if (track.last_seen_frame != acquisition_frame_count_) {
      continue;
    }
    std::map<std::string, std::size_t> votes;
    for (const auto & observation : track.observations) {
      if (observation.maturity_confidence >= config_.maturity_confidence_threshold) {
        ++votes[observation.type];
      }
    }
    if (votes.empty()) {
      continue;
    }
    const auto best_type = std::max_element(
      votes.begin(), votes.end(),
      [](const auto & left, const auto & right) {return left.second < right.second;});
    const auto & type = best_type->first;
    if (best_type->second < config_.min_observations ||
      static_cast<double>(best_type->second) / track.observations.size() <
      config_.type_support_ratio || type.size() < 5U ||
      type.compare(type.size() - 5U, 5U, "_ripe") != 0 ||
      (!expected_type.empty() && expected_type != "ripe" && type != expected_type))
    {
      continue;
    }
    const auto & latest = track.observations.back();
    if (latest.type != type ||
      latest.maturity_confidence < config_.maturity_confidence_threshold)
    {
      continue;
    }
    std::vector<const Observation *> supporting;
    cv::Point3d supporting_sum{};
    for (const auto & observation : track.observations) {
      if (observation.type == type &&
        observation.maturity_confidence >= config_.maturity_confidence_threshold)
      {
        supporting.push_back(&observation);
        supporting_sum += observation.point;
      }
    }
    const cv::Point3d initial_mean =
      supporting_sum * (1.0 / static_cast<double>(supporting.size()));
    cv::Point3d variance{};
    std::vector<double> confidences;
    std::vector<double> centralities;
    confidences.reserve(track.observations.size());
    centralities.reserve(track.observations.size());
    for (const auto * item : supporting) {
      const auto & observation = *item;
      const cv::Point3d difference = observation.point - initial_mean;
      variance.x += difference.x * difference.x;
      variance.y += difference.y * difference.y;
      variance.z += difference.z * difference.z;
      confidences.push_back(observation.confidence);
      centralities.push_back(observation.horizontal_centrality);
    }
    const double divisor = 1.0 / static_cast<double>(supporting.size());
    const cv::Point3d initial_stddev{
      std::sqrt(variance.x * divisor),
      std::sqrt(variance.y * divisor),
      std::sqrt(variance.z * divisor)};

    std::vector<const Observation *> inliers;
    inliers.reserve(track.observations.size());
    cv::Point3d inlier_sum{};
    for (const auto * item : supporting) {
      const auto & observation = *item;
      const cv::Point3d difference = observation.point - initial_mean;
      if (std::abs(difference.x) <= config_.outlier_sigma * initial_stddev.x &&
        std::abs(difference.y) <= config_.outlier_sigma * initial_stddev.y &&
        std::abs(difference.z) <= config_.outlier_sigma * initial_stddev.z)
      {
        inliers.push_back(&observation);
        inlier_sum += observation.point;
      }
    }
    if (inliers.empty()) {
      for (const auto * observation : supporting) {
        inliers.push_back(observation);
        inlier_sum += observation->point;
      }
    }

    const cv::Point3d mean =
      inlier_sum * (1.0 / static_cast<double>(inliers.size()));
    variance = {};
    for (const auto * observation : inliers) {
      const cv::Point3d difference = observation->point - mean;
      variance.x += difference.x * difference.x;
      variance.y += difference.y * difference.y;
      variance.z += difference.z * difference.z;
    }
    const double inlier_divisor = 1.0 / static_cast<double>(inliers.size());
    const cv::Point3d stddev{
      std::sqrt(variance.x * inlier_divisor),
      std::sqrt(variance.y * inlier_divisor),
      std::sqrt(variance.z * inlier_divisor)};
    const double confidence = median(std::move(confidences));
    const double centrality = median(std::move(centralities));
    const double score =
      config_.confidence_weight * confidence +
      (1.0 - config_.confidence_weight) * centrality;
    eligible.push_back(
      TrackStatistics{
        &track, mean, stddev, inliers.size(), confidence, centrality, score, type});
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

std::optional<CameraTargetCandidate> CameraTargetSelector::match_locked_target(
  const std::vector<CameraTargetCandidate> & candidates) const
{
  const double radius_squared = config_.association_radius_m * config_.association_radius_m;
  const CameraTargetCandidate * closest = nullptr;
  double closest_distance = radius_squared;
  for (const auto & candidate : candidates) {
    if (candidate.type != locked_type_ ||
      candidate.maturity_confidence < config_.maturity_confidence_threshold)
    {
      continue;
    }
    const double distance = squared_distance(candidate.point, *locked_point_);
    if (distance <= radius_squared &&
      (closest == nullptr || distance < closest_distance))
    {
      closest = &candidate;
      closest_distance = distance;
    }
  }
  if (closest == nullptr) {
    return std::nullopt;
  }
  return *closest;
}

}  // namespace fruit_perception
