#include "fruit_perception/b_target_selector.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>

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

std::string fruit_species(const std::string & type)
{
  return type.substr(0, type.find('_'));
}

void validate_config(const BTargetSelectorConfig & config)
{
  if (config.acquisition_frames == 0U) {
    throw std::invalid_argument("d435i_b_target_acquisition_frames must be positive");
  }
  if (config.max_targets == 0U || config.max_targets > 2U) {
    throw std::invalid_argument("d435i_b_target_max_count must be between 1 and 2");
  }
  if (!std::isfinite(config.association_radius_m) || config.association_radius_m <= 0.0) {
    throw std::invalid_argument(
            "d435i_b_target_association_radius_m must be positive and finite");
  }
  if (!std::isfinite(config.outlier_sigma) || config.outlier_sigma <= 0.0) {
    throw std::invalid_argument(
            "d435i_b_target_outlier_sigma must be positive and finite");
  }
}

}  // namespace

BTargetSelector::BTargetSelector(BTargetSelectorConfig config)
: config_(std::move(config))
{
  validate_config(config_);
}

BTargetUpdate BTargetSelector::update(const std::vector<BTargetCandidate> & candidates)
{
  if (ready_) {
    return BTargetUpdate{true, locked_targets_};
  }

  add_frame(candidates);
  if (frame_count_ < config_.acquisition_frames) {
    return {};
  }

  auto statistics = select_tracks();
  std::stable_sort(
    statistics.begin(), statistics.end(),
    [](const auto & left, const auto & right) {
      if (left.confidence != right.confidence) {
        return left.confidence > right.confidence;
      }
      return left.result.observation_count > right.result.observation_count;
    });
  if (!statistics.empty()) {
    // B-zone trees contain one fruit species; the strongest stable track defines it.
    const auto dominant_species = fruit_species(statistics.front().result.type);
    statistics.erase(
      std::remove_if(
        statistics.begin(), statistics.end(),
        [&dominant_species](const auto & statistic) {
          return fruit_species(statistic.result.type) != dominant_species;
        }),
      statistics.end());
  }
  if (statistics.size() > config_.max_targets) {
    statistics.resize(config_.max_targets);
  }

  locked_targets_.reserve(statistics.size());
  for (const auto & statistic : statistics) {
    locked_targets_.push_back(statistic.result);
  }
  ready_ = true;
  return BTargetUpdate{true, locked_targets_};
}

void BTargetSelector::reset()
{
  frame_count_ = 0U;
  tracks_.clear();
  ready_ = false;
  locked_targets_.clear();
}

cv::Point3d BTargetSelector::mean_point(const Track & track)
{
  return track.point_sum * (1.0 / static_cast<double>(track.observations.size()));
}

void BTargetSelector::add_frame(const std::vector<BTargetCandidate> & candidates)
{
  ++frame_count_;
  const double radius_squared =
    config_.association_radius_m * config_.association_radius_m;
  for (const auto & candidate : candidates) {
    Track * closest = nullptr;
    double closest_distance = radius_squared;
    for (auto & track : tracks_) {
      if (track.last_seen_frame == frame_count_) {
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

    const Observation observation{candidate.type, candidate.point, candidate.confidence};
    if (closest == nullptr) {
      Track track;
      track.observations.push_back(observation);
      track.point_sum = candidate.point;
      track.last_seen_frame = frame_count_;
      tracks_.push_back(std::move(track));
      continue;
    }
    closest->observations.push_back(observation);
    closest->point_sum += candidate.point;
    closest->last_seen_frame = frame_count_;
  }
}

std::vector<BTargetSelector::TrackStatistics> BTargetSelector::select_tracks() const
{
  std::vector<TrackStatistics> statistics;
  statistics.reserve(tracks_.size());
  for (const auto & track : tracks_) {
    if (track.observations.empty()) {
      continue;
    }

    const cv::Point3d initial_mean = mean_point(track);
    cv::Point3d variance{};
    std::vector<double> confidences;
    std::map<std::string, std::vector<double>> type_confidences;
    confidences.reserve(track.observations.size());
    for (const auto & observation : track.observations) {
      const cv::Point3d difference = observation.point - initial_mean;
      variance.x += difference.x * difference.x;
      variance.y += difference.y * difference.y;
      variance.z += difference.z * difference.z;
      confidences.push_back(observation.confidence);
      type_confidences[observation.type].push_back(observation.confidence);
    }
    const double divisor = 1.0 / static_cast<double>(track.observations.size());
    const cv::Point3d initial_stddev{
      std::sqrt(variance.x * divisor),
      std::sqrt(variance.y * divisor),
      std::sqrt(variance.z * divisor)};

    std::vector<const Observation *> inliers;
    cv::Point3d inlier_sum{};
    for (const auto & observation : track.observations) {
      const cv::Point3d difference = observation.point - initial_mean;
      const bool within_x = initial_stddev.x == 0.0 ||
        std::abs(difference.x) <= config_.outlier_sigma * initial_stddev.x;
      const bool within_y = initial_stddev.y == 0.0 ||
        std::abs(difference.y) <= config_.outlier_sigma * initial_stddev.y;
      const bool within_z = initial_stddev.z == 0.0 ||
        std::abs(difference.z) <= config_.outlier_sigma * initial_stddev.z;
      if (within_x && within_y && within_z) {
        inliers.push_back(&observation);
        inlier_sum += observation.point;
      }
    }
    if (inliers.empty()) {
      for (const auto & observation : track.observations) {
        inliers.push_back(&observation);
        inlier_sum += observation.point;
      }
    }

    const auto best_type = std::max_element(
      type_confidences.begin(), type_confidences.end(),
      [](const auto & left, const auto & right) {
        const double left_confidence = median(left.second);
        const double right_confidence = median(right.second);
        if (left_confidence != right_confidence) {
          return left_confidence < right_confidence;
        }
        return left.first > right.first;
      });
    const double confidence = median(std::move(confidences));
    statistics.push_back(
      TrackStatistics{
        BTargetResult{
          best_type->first,
          inlier_sum * (1.0 / static_cast<double>(inliers.size())),
          track.observations.size(),
          inliers.size(),
          confidence},
        confidence});
  }
  return statistics;
}

}  // namespace fruit_perception
