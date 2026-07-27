#ifndef PB_GICP_LOCALIZATION__REGISTRATION_VALIDATION_HPP_
#define PB_GICP_LOCALIZATION__REGISTRATION_VALIDATION_HPP_

#include <cmath>
#include <cstddef>
#include <string>

#include <Eigen/Geometry>

namespace pb_gicp_localization
{

struct RegistrationMetrics
{
  bool converged{};
  std::size_t source_points{};
  std::size_t inliers{};
  double total_error{};
  Eigen::Isometry3d candidate{Eigen::Isometry3d::Identity()};
};

struct ValidationThresholds
{
  std::size_t min_inliers{500};
  double min_inlier_ratio{0.2};
  double max_mean_error{1.0};
  double max_translation_jump{0.5};
  double max_rotation_jump{0.35};
};

inline double rotationDistance(
  const Eigen::Isometry3d & lhs, const Eigen::Isometry3d & rhs)
{
  const Eigen::Matrix3d delta = lhs.rotation().transpose() * rhs.rotation();
  return Eigen::AngleAxisd(delta).angle();
}

inline bool validateRegistration(
  const RegistrationMetrics & metrics, const Eigen::Isometry3d & previous,
  const ValidationThresholds & thresholds, std::string & reason)
{
  if (!metrics.converged) {
    reason = "optimizer did not converge";
    return false;
  }
  if (!metrics.candidate.matrix().allFinite()) {
    reason = "candidate transform is not finite";
    return false;
  }
  if (metrics.source_points == 0 || metrics.inliers < thresholds.min_inliers) {
    reason = "too few inliers";
    return false;
  }
  const double ratio = static_cast<double>(metrics.inliers) / metrics.source_points;
  if (ratio < thresholds.min_inlier_ratio) {
    reason = "inlier ratio below threshold";
    return false;
  }
  const double mean_error = metrics.total_error / metrics.inliers;
  if (!std::isfinite(mean_error) || mean_error > thresholds.max_mean_error) {
    reason = "mean GICP error above threshold";
    return false;
  }
  if ((metrics.candidate.translation() - previous.translation()).norm() >
    thresholds.max_translation_jump)
  {
    reason = "translation jump above threshold";
    return false;
  }
  if (rotationDistance(previous, metrics.candidate) > thresholds.max_rotation_jump) {
    reason = "rotation jump above threshold";
    return false;
  }
  return true;
}

}  // namespace pb_gicp_localization

#endif  // PB_GICP_LOCALIZATION__REGISTRATION_VALIDATION_HPP_
