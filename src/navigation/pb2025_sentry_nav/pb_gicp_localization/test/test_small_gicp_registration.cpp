#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

TEST(SmallGicpRegistration, RecoversSyntheticTransform)
{
  small_gicp::PointCloud target;
  for (int x = -10; x <= 10; ++x) {
    for (int y = -10; y <= 10; ++y) {
      const double xd = 0.2 * x;
      const double yd = 0.2 * y;
      target.points.emplace_back(xd, yd, 0.1 * std::sin(xd) + 0.05 * std::cos(yd), 1.0);
    }
  }

  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.translation() << 0.2, -0.1, 0.05;
  expected.linear() = Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  small_gicp::PointCloud source;
  source.resize(target.size());
  for (std::size_t i = 0; i < target.size(); ++i) {
    source.point(i) = expected.inverse() * target.point(i);
  }

  small_gicp::estimate_covariances_omp(target, 20, 4);
  small_gicp::estimate_covariances_omp(source, 20, 4);
  auto target_ptr = std::make_shared<small_gicp::PointCloud>(target);
  small_gicp::KdTree<small_gicp::PointCloud> target_tree(
    target_ptr, small_gicp::KdTreeBuilderOMP(4));

  small_gicp::Registration<
    small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
  registration.reduction.num_threads = 4;
  registration.rejector.max_dist_sq = 3.0;
  const auto result = registration.align(target, source, target_tree);

  ASSERT_TRUE(result.converged);
  EXPECT_GT(result.num_inliers, 300U);
  EXPECT_LT((result.T_target_source.translation() - expected.translation()).norm(), 0.03);
  EXPECT_LT(
    Eigen::AngleAxisd(expected.rotation().transpose() * result.T_target_source.rotation()).angle(),
    0.03);
}
