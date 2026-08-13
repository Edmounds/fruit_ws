// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "loam_interface/extrinsic_math.hpp"

TEST(ExtrinsicMath, BuildsLidarInImuFromPointLioExtrinsic)
{
  const auto lidar_in_imu = loam_interface::makeLidarInImuTransform(
    {-0.011, -0.02329, 0.04412},
    {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  EXPECT_NEAR(lidar_in_imu.getOrigin().x(), -0.011, 1e-9);
  EXPECT_NEAR(lidar_in_imu.getOrigin().y(), -0.02329, 1e-9);
  EXPECT_NEAR(lidar_in_imu.getOrigin().z(), 0.04412, 1e-9);
}

TEST(ExtrinsicMath, OdomToCameraInitUsesBaseToImuNotBaseToLidar)
{
  tf2::Transform base_to_lidar;
  base_to_lidar.setIdentity();
  base_to_lidar.setOrigin(tf2::Vector3(0.2, 0.0, 0.0));

  const auto lidar_in_imu = loam_interface::makeLidarInImuTransform(
    {0.05, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  const auto odom_to_camera = loam_interface::odomToCameraInit(base_to_lidar, lidar_in_imu);

  // base->imu = base->lidar * lidar->imu = (0.2) * inv(0.05) = 0.15 along x
  EXPECT_NEAR(odom_to_camera.getOrigin().x(), 0.15, 1e-9);
  EXPECT_NEAR(odom_to_camera.getOrigin().y(), 0.0, 1e-9);
}

TEST(ExtrinsicMath, OdomToLidarAppliesBodyThenLidarExtrinsic)
{
  tf2::Transform odom_to_camera;
  odom_to_camera.setIdentity();
  odom_to_camera.setOrigin(tf2::Vector3(0.15, 0.0, 0.0));

  tf2::Transform camera_to_body;
  camera_to_body.setIdentity();
  camera_to_body.setOrigin(tf2::Vector3(1.0, 0.0, 0.0));

  const auto lidar_in_imu = loam_interface::makeLidarInImuTransform(
    {0.05, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});

  const auto odom_to_lidar =
    loam_interface::odomToLidar(odom_to_camera, camera_to_body, lidar_in_imu);
  EXPECT_NEAR(odom_to_lidar.getOrigin().x(), 1.20, 1e-9);

  // Legacy bug: base_to_lidar * camera_to_body would yield 1.05 when base_to_lidar.x=0.2
  tf2::Transform legacy;
  legacy.setIdentity();
  legacy.setOrigin(tf2::Vector3(0.2, 0.0, 0.0));
  const tf2::Transform legacy_result = legacy * camera_to_body;
  EXPECT_NEAR(legacy_result.getOrigin().x(), 1.05, 1e-9);
  EXPECT_GT(std::abs(odom_to_lidar.getOrigin().x() - legacy_result.getOrigin().x()), 1e-3);
}

TEST(ExtrinsicMath, RejectsInvalidExtrinsicSizes)
{
  EXPECT_THROW(
    loam_interface::makeLidarInImuTransform({0.0, 0.0}, {1, 0, 0, 0, 1, 0, 0, 0, 1}),
    std::invalid_argument);
  EXPECT_THROW(
    loam_interface::makeLidarInImuTransform({0.0, 0.0, 0.0}, {1, 0, 0}),
    std::invalid_argument);
}
