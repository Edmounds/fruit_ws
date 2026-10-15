#include "fruit_perception/b_target_selector.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>

namespace fruit_perception
{

bool defect_belongs_to_fruit(const cv::Rect & defect, const cv::Rect & fruit)
{
  if (defect.empty() || fruit.empty()) {
    return false;
  }
  const cv::Point2d center{
    defect.x + defect.width * 0.5, defect.y + defect.height * 0.5};
  return center.x >= fruit.x && center.x < fruit.x + fruit.width &&
         center.y >= fruit.y && center.y < fruit.y + fruit.height &&
         static_cast<double>((defect & fruit).area()) / defect.area() >= 0.5;
}

std::optional<BTargetResult> select_primary_target(
  const std::vector<BTargetResult> & targets, double confidence_weight,
  double score_close_threshold)
{
  if (targets.empty()) {
    return std::nullopt;
  }
  const auto score = [confidence_weight](const BTargetResult & target) {
      const double half_width = target.image_width * 0.5;
      const double centrality = half_width > 0.0 ? 1.0 - std::clamp(
        std::abs(target.pixel_center.x - half_width) / half_width, 0.0, 1.0) : 0.0;
      return confidence_weight * target.confidence + (1.0 - confidence_weight) * centrality;
    };
  const double best_score = score(
    *std::max_element(
      targets.begin(), targets.end(), [&score](const auto & left, const auto & right) {
        return score(left) < score(right);
      }));
  const BTargetResult * primary = nullptr;
  for (const auto & target : targets) {
    if (best_score - score(target) <= score_close_threshold &&
      (primary == nullptr || target.confidence > primary->confidence))
    {
      primary = &target;
    }
  }
  return *primary;
}

BTargetSelector::BTargetSelector(BTargetSelectorConfig config)
: config_(std::move(config))
{
  if (config_.acquisition_frames == 0U ||
    !std::isfinite(config_.association_radius_m) || config_.association_radius_m <= 0.0 ||
    !std::isfinite(config_.outlier_sigma) || config_.outlier_sigma <= 0.0 ||
    !std::isfinite(config_.maturity_confidence_threshold) ||
    config_.maturity_confidence_threshold < 0.5 || config_.maturity_confidence_threshold > 1.0)
  {
    throw std::invalid_argument("Invalid target vote configuration");
  }
}

BTargetUpdate BTargetSelector::update(
  const std::vector<BTargetCandidate> & candidates, const std::vector<cv::Rect> & defects)
{
  if (ready_) {
    return {true, locked_targets_};
  }
  defects_.insert(defects_.end(), defects.begin(), defects.end());
  add_frame(candidates);
  for (auto & track : tracks_) {
    for (const auto & defect : defects_) {
      if (defect_belongs_to_fruit(defect, track.observations.back().roi)) {
        track.bad = true;
      }
    }
  }
  if (frame_count_ < config_.acquisition_frames) {
    return {};
  }
  locked_targets_ = select_tracks();
  ready_ = true;
  return {true, locked_targets_};
}

void BTargetSelector::reset()
{
  frame_count_ = 0U;
  tracks_.clear();
  defects_.clear();
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
  for (const auto & candidate : candidates) {
    if (!std::isfinite(candidate.confidence) || candidate.confidence < 0.0 ||
      candidate.confidence > 1.0 || !std::isfinite(candidate.maturity_confidence) ||
      candidate.maturity_confidence < 0.0 || candidate.maturity_confidence > 1.0)
    {
      continue;
    }
    Track * closest = nullptr;
    double best_overlap = 0.3;
    double closest_distance = config_.association_radius_m * config_.association_radius_m;
    for (auto & track : tracks_) {
      if (track.last_seen_frame == frame_count_) {
        continue;
      }
      const auto & previous = track.observations.back();
      if (!candidate.roi.empty() && !previous.roi.empty()) {
        const double intersection = (candidate.roi & previous.roi).area();
        const double overlap = intersection /
          (candidate.roi.area() + previous.roi.area() - intersection);
        if (overlap > best_overlap) {
          closest = &track;
          best_overlap = overlap;
        }
      } else if (candidate.has_point && previous.has_point) {
        const cv::Point3d difference = candidate.point - mean_point(track);
        const double distance = difference.dot(difference);
        if (distance <= closest_distance) {
          closest = &track;
          closest_distance = distance;
        }
      }
    }
    const Observation observation{
      candidate.type, candidate.point, candidate.pixel_center,
      candidate.image_width, candidate.confidence, candidate.maturity_confidence, frame_count_,
      candidate.roi, candidate.has_point};
    if (closest == nullptr) {
      tracks_.push_back(Track{{observation}, candidate.point, frame_count_, candidate.bad});
    } else {
      closest->observations.push_back(observation);
      closest->point_sum += candidate.point;
      closest->last_seen_frame = frame_count_;
      closest->bad = closest->bad || candidate.bad;
    }
  }
}

std::vector<BTargetResult> BTargetSelector::select_tracks() const
{
  std::vector<BTargetResult> statistics;
  for (const auto & track : tracks_) {
    // Keep all tracks until the window closes, including bad or temporarily missing fruit.
    std::map<std::string, std::pair<std::size_t, double>> votes;
    double confidence_sum = 0.0;
    cv::Point2d pixel_sum{};
    for (const auto & observation : track.observations) {
      confidence_sum += observation.confidence;
      pixel_sum += observation.pixel_center;
      if (observation.maturity_confidence >= config_.maturity_confidence_threshold) {
        auto & vote = votes[observation.type];
        ++vote.first;
        vote.second += observation.confidence;
      }
    }
    std::string winner;
    std::size_t best_count = 0U;
    double best_confidence = 0.0;
    bool tied = false;
    for (const auto & item : votes) {
      const auto count = item.second.first;
      const double confidence = item.second.second / count;
      if (count > best_count || (count == best_count && confidence > best_confidence)) {
        winner = item.first;
        best_count = count;
        best_confidence = confidence;
        tied = false;
      } else if (count == best_count && confidence == best_confidence) {
        tied = true;
      }
    }
    if (tied) {
      winner.clear();
    }
    BTargetResult result;
    result.type = track.bad ? "bad" : (winner.empty() ? "unknown" : winner);
    result.confidence = confidence_sum / track.observations.size();
    result.pixel_center = pixel_sum * (1.0 / track.observations.size());
    result.image_width = track.observations.front().image_width;
    result.observation_count = best_count;

    std::vector<cv::Point3d> points;
    cv::Point3d sum{};
    for (const auto & observation : track.observations) {
      if (observation.type == winner &&
        observation.maturity_confidence >= config_.maturity_confidence_threshold &&
        observation.has_point && std::isfinite(observation.point.x) &&
        std::isfinite(observation.point.y) && std::isfinite(observation.point.z) &&
        observation.point.z > 0.0)
      {
        points.push_back(observation.point);
        sum += observation.point;
      }
    }
    if (!points.empty()) {
      const auto mean = sum * (1.0 / points.size());
      cv::Point3d variance{};
      for (const auto & point : points) {
        const auto difference = point - mean;
        variance.x += difference.x * difference.x;
        variance.y += difference.y * difference.y;
        variance.z += difference.z * difference.z;
      }
      const cv::Point3d deviation{
        std::sqrt(variance.x / points.size()), std::sqrt(variance.y / points.size()),
        std::sqrt(variance.z / points.size())};
      cv::Point3d inlier_sum{};
      for (const auto & point : points) {
        const auto difference = point - mean;
        if (std::abs(difference.x) <= config_.outlier_sigma * deviation.x &&
          std::abs(difference.y) <= config_.outlier_sigma * deviation.y &&
          std::abs(difference.z) <= config_.outlier_sigma * deviation.z)
        {
          inlier_sum += point;
          ++result.inlier_count;
        }
      }
      if (result.inlier_count == 0U) {
        result.point = mean;
        result.inlier_count = points.size();
      } else {
        result.point = inlier_sum * (1.0 / result.inlier_count);
      }
      result.has_point = !track.bad;
    }
    statistics.push_back(result);
  }
  return statistics;
}

}  // namespace fruit_perception
