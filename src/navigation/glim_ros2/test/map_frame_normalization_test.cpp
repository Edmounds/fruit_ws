#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <sstream>

#include "glim_ros/map_frame_normalization.hpp"

namespace {

Eigen::Vector3d rpy(const Eigen::Matrix3d& rotation)
{
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double pitch = std::asin(-rotation(2, 0));
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  return {roll, pitch, yaw};
}

Eigen::Matrix3d rotation(double roll, double pitch, double yaw)
{
  return (
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
    .toRotationMatrix();
}

}  // namespace

TEST(MapFrameNormalization, ZerosInitialBaseTranslationAndYaw)
{
  Eigen::Isometry3d T_native_base = Eigen::Isometry3d::Identity();
  T_native_base.translation() << 1.2, -0.8, 0.35;
  T_native_base.linear() = rotation(-0.08, 0.04, M_PI_2);

  const Eigen::Isometry3d A = glim::planarMapFrameNormalization(T_native_base);
  const Eigen::Isometry3d T_odom_base = A * T_native_base;
  const Eigen::Vector3d normalized_rpy = rpy(T_odom_base.linear());

  EXPECT_NEAR(T_odom_base.translation().norm(), 0.0, 1e-12);
  EXPECT_NEAR(normalized_rpy.x(), -0.08, 1e-12);
  EXPECT_NEAR(normalized_rpy.y(), 0.04, 1e-12);
  EXPECT_NEAR(normalized_rpy.z(), 0.0, 1e-12);
  EXPECT_NEAR(rpy(A.linear()).x(), 0.0, 1e-12);
  EXPECT_NEAR(rpy(A.linear()).y(), 0.0, 1e-12);
}

TEST(MapFrameNormalization, PreservesRelativeMotion)
{
  Eigen::Isometry3d T_native_base0 = Eigen::Isometry3d::Identity();
  T_native_base0.translation() << 2.0, -1.0, 0.2;
  T_native_base0.linear() = rotation(-0.03, 0.02, 1.4);

  Eigen::Isometry3d T_base0_base1 = Eigen::Isometry3d::Identity();
  T_base0_base1.translation() << 0.7, -0.2, 0.05;
  T_base0_base1.linear() = rotation(0.01, -0.02, 0.25);
  const Eigen::Isometry3d T_native_base1 = T_native_base0 * T_base0_base1;

  const Eigen::Isometry3d A = glim::planarMapFrameNormalization(T_native_base0);
  const Eigen::Isometry3d T_odom_base0 = A * T_native_base0;
  const Eigen::Isometry3d T_odom_base1 = A * T_native_base1;

  EXPECT_TRUE(
    (T_odom_base0.inverse() * T_odom_base1).matrix().isApprox(
      T_base0_base1.matrix(), 1e-12));
}

TEST(MapFrameNormalization, ConjugatesMapToOdomTransform)
{
  Eigen::Isometry3d T_native_base0 = Eigen::Isometry3d::Identity();
  T_native_base0.translation() << 1.0, 2.0, 0.3;
  T_native_base0.linear() = rotation(0.0, 0.0, M_PI_2);
  const Eigen::Isometry3d A = glim::planarMapFrameNormalization(T_native_base0);

  Eigen::Isometry3d T_world_native_odom = Eigen::Isometry3d::Identity();
  T_world_native_odom.translation() << 0.2, -0.1, 0.0;
  T_world_native_odom.linear() = rotation(0.0, 0.0, 0.1);

  Eigen::Isometry3d T_native_odom_base = Eigen::Isometry3d::Identity();
  T_native_odom_base.translation() << 1.5, 2.4, 0.3;
  T_native_odom_base.linear() = rotation(0.0, 0.0, 1.7);

  const Eigen::Isometry3d T_map_odom =
    glim::normalizedMapToOdom(A, T_world_native_odom);
  const Eigen::Isometry3d T_odom_base = A * T_native_odom_base;
  const Eigen::Isometry3d T_map_base = A * T_world_native_odom * T_native_odom_base;

  EXPECT_TRUE((T_map_odom * T_odom_base).matrix().isApprox(T_map_base.matrix(), 1e-12));
}

TEST(MapFrameNormalization, RoundTripsDumpMetadata)
{
  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.translation() << 1.25, -0.5, 3.0;
  expected.linear() = rotation(0.0, 0.0, -0.2);

  std::stringstream stream;
  ASSERT_TRUE(glim::writeMapFrameNormalization(stream, expected));

  const auto actual = glim::readMapFrameNormalization(stream);
  ASSERT_TRUE(actual);
  EXPECT_TRUE(actual->matrix().isApprox(expected.matrix(), 1e-12));
}

TEST(MapFrameNormalization, RejectsMetadataThatIsNotARigidTransform)
{
  std::stringstream stream(
    "2 0 0 0 "
    "0 1 0 0 "
    "0 0 1 0 "
    "0 0 0 1");

  EXPECT_FALSE(glim::readMapFrameNormalization(stream));
}
