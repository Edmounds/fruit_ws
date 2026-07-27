#include <limits>

#include <gtest/gtest.h>

#include "pb_gicp_localization/registration_validation.hpp"

using pb_gicp_localization::RegistrationMetrics;
using pb_gicp_localization::ValidationThresholds;

TEST(RegistrationValidation, AcceptsGoodResult)
{
  RegistrationMetrics metrics;
  metrics.converged = true;
  metrics.source_points = 2000;
  metrics.inliers = 1000;
  metrics.total_error = 500.0;
  metrics.candidate.translation().x() = 0.2;
  std::string reason;
  EXPECT_TRUE(pb_gicp_localization::validateRegistration(
    metrics, Eigen::Isometry3d::Identity(), ValidationThresholds{}, reason));
}

TEST(RegistrationValidation, RejectsEverySafetyGate)
{
  RegistrationMetrics metrics;
  metrics.converged = true;
  metrics.source_points = 2000;
  metrics.inliers = 1000;
  metrics.total_error = 500.0;
  std::string reason;
  ValidationThresholds thresholds;

  metrics.inliers = 499;
  EXPECT_FALSE(pb_gicp_localization::validateRegistration(
    metrics, Eigen::Isometry3d::Identity(), thresholds, reason));
  metrics.inliers = 500;
  metrics.source_points = 3000;
  EXPECT_FALSE(pb_gicp_localization::validateRegistration(
    metrics, Eigen::Isometry3d::Identity(), thresholds, reason));
  metrics.source_points = 2000;
  metrics.total_error = 501.0;
  EXPECT_FALSE(pb_gicp_localization::validateRegistration(
    metrics, Eigen::Isometry3d::Identity(), thresholds, reason));
  metrics.total_error = 500.0;
  metrics.candidate.translation().x() = 0.51;
  EXPECT_FALSE(pb_gicp_localization::validateRegistration(
    metrics, Eigen::Isometry3d::Identity(), thresholds, reason));
  metrics.candidate = Eigen::Isometry3d::Identity();
  metrics.candidate.linear() = Eigen::AngleAxisd(0.36, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_FALSE(pb_gicp_localization::validateRegistration(
    metrics, Eigen::Isometry3d::Identity(), thresholds, reason));
  metrics.converged = false;
  metrics.candidate = Eigen::Isometry3d::Identity();
  EXPECT_FALSE(pb_gicp_localization::validateRegistration(
    metrics, Eigen::Isometry3d::Identity(), thresholds, reason));

  metrics.converged = true;
  metrics.candidate.matrix()(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(pb_gicp_localization::validateRegistration(
    metrics, Eigen::Isometry3d::Identity(), thresholds, reason));
}
