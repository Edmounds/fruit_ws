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

TEST(ExtrinsicMath, OdomToCameraInitCompensatesInitialPointLioPose)
{
  tf2::Transform base_to_lidar;
  base_to_lidar.setIdentity();
  base_to_lidar.setOrigin(tf2::Vector3(0.2, 0.0, 0.0));

  tf2::Transform camera_to_body;
  camera_to_body.setIdentity();
  camera_to_body.setOrigin(tf2::Vector3(1.0, 0.0, 0.0));

  const auto lidar_in_imu = loam_interface::makeLidarInImuTransform(
    {0.05, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  const auto odom_to_camera =
    loam_interface::odomToCameraInit(base_to_lidar, camera_to_body, lidar_in_imu);
  const auto initial_odom_to_lidar =
    loam_interface::odomToLidar(odom_to_camera, camera_to_body, lidar_in_imu);

  EXPECT_NEAR(odom_to_camera.getOrigin().x(), -0.85, 1e-9);
  EXPECT_NEAR(odom_to_camera.getOrigin().y(), 0.0, 1e-9);
  EXPECT_NEAR(initial_odom_to_lidar.getOrigin().x(), 0.2, 1e-9);
}

TEST(ExtrinsicMath, OdomToCameraInitCompensatesInitialPointLioRotation)
{
  tf2::Transform base_to_lidar;
  base_to_lidar.setIdentity();
  tf2::Quaternion base_rotation;
  base_rotation.setRPY(0.3, -0.2, 0.1);
  base_to_lidar.setRotation(base_rotation);

  tf2::Transform camera_to_body;
  camera_to_body.setIdentity();
  tf2::Quaternion initial_body_rotation;
  initial_body_rotation.setRPY(-0.5, 0.25, -0.4);
  camera_to_body.setRotation(initial_body_rotation);

  tf2::Transform lidar_in_imu;
  lidar_in_imu.setIdentity();
  const auto odom_to_camera =
    loam_interface::odomToCameraInit(base_to_lidar, camera_to_body, lidar_in_imu);
  const auto initial_odom_to_lidar =
    loam_interface::odomToLidar(odom_to_camera, camera_to_body, lidar_in_imu);

  EXPECT_NEAR(initial_odom_to_lidar.getRotation().angleShortestPath(base_rotation), 0.0, 1e-9);
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
