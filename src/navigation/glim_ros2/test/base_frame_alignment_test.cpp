#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <sstream>

#include "glim_ros/base_frame_alignment.hpp"

TEST(BaseFrameAlignment, MakesInitialBasePoseIdentity)
{
  Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
  T_world_imu.translation() << 1.0, -2.0, 0.4;
  T_world_imu.linear() =
    (Eigen::AngleAxisd(-0.5, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ())).toRotationMatrix();

  Eigen::Isometry3d T_imu_base = Eigen::Isometry3d::Identity();
  T_imu_base.translation() << -0.1, 0.2, 0.05;
  T_imu_base.linear() =
    Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitY()).toRotationMatrix();

  const Eigen::Isometry3d T_map_world =
    glim::baseFrameWorldAlignment(T_world_imu, T_imu_base);
  const Eigen::Isometry3d T_map_base = T_map_world * T_world_imu * T_imu_base;

  EXPECT_NEAR(T_map_base.translation().norm(), 0.0, 1e-12);
  EXPECT_NEAR(
    Eigen::AngleAxisd(T_map_base.linear()).angle(),
    0.0,
    1e-12);
}

TEST(BaseFrameAlignment, PreservesMotionRelativeToTheInitialBasePose)
{
  Eigen::Isometry3d T_world_imu_initial = Eigen::Isometry3d::Identity();
  T_world_imu_initial.linear() =
    Eigen::AngleAxisd(-0.5, Eigen::Vector3d::UnitX()).toRotationMatrix();

  Eigen::Isometry3d T_imu_base = Eigen::Isometry3d::Identity();
  T_imu_base.translation() << -0.1, 0.0, 0.05;
  T_imu_base.linear() =
    Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitX()).toRotationMatrix();

  Eigen::Isometry3d T_world_imu_moved = T_world_imu_initial;
  T_world_imu_moved.translation() << 2.0, -1.0, 0.0;

  const Eigen::Isometry3d T_map_world =
    glim::baseFrameWorldAlignment(T_world_imu_initial, T_imu_base);
  const Eigen::Isometry3d T_map_base_moved =
    T_map_world * T_world_imu_moved * T_imu_base;

  EXPECT_NEAR(T_map_base_moved.translation().x(), 2.0, 1e-12);
  EXPECT_NEAR(T_map_base_moved.translation().y(), -1.0, 1e-12);
  EXPECT_NEAR(T_map_base_moved.translation().z(), 0.0, 1e-12);
  EXPECT_NEAR(
    Eigen::AngleAxisd(T_map_base_moved.linear()).angle(),
    0.0,
    1e-12);
}

TEST(BaseFrameAlignment, RoundTripsDumpMetadata)
{
  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.translation() << 1.25, -0.5, 3.0;
  expected.linear() =
    (Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitZ())).toRotationMatrix();

  std::stringstream stream;
  ASSERT_TRUE(glim::writeBaseFrameAlignment(stream, expected));

  const auto actual = glim::readBaseFrameAlignment(stream);
  ASSERT_TRUE(actual);
  EXPECT_TRUE(actual->matrix().isApprox(expected.matrix(), 1e-12));
}

TEST(BaseFrameAlignment, RejectsMetadataThatIsNotARigidTransform)
{
  std::stringstream stream(
    "2 0 0 0 "
    "0 1 0 0 "
    "0 0 1 0 "
    "0 0 0 1");

  EXPECT_FALSE(glim::readBaseFrameAlignment(stream));
}
