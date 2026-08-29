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

#ifndef LOAM_INTERFACE__EXTRINSIC_MATH_HPP_
#define LOAM_INTERFACE__EXTRINSIC_MATH_HPP_

#include "tf2/LinearMath/Transform.h"

namespace loam_interface
{

// Align the first LiDAR pose with the calibrated base->LiDAR transform.
inline tf2::Transform odomToCameraInit(
  const tf2::Transform & base_to_lidar, const tf2::Transform & camera_init_to_body,
  const tf2::Transform & lidar_in_imu)
{
  return base_to_lidar * (camera_init_to_body * lidar_in_imu).inverse();
}

// Point-LIO odometry is camera_init -> body(IMU). Convert to odom -> lidar.
inline tf2::Transform odomToLidar(
  const tf2::Transform & odom_to_camera_init, const tf2::Transform & camera_init_to_body,
  const tf2::Transform & lidar_in_imu)
{
  return odom_to_camera_init * camera_init_to_body * lidar_in_imu;
}

}  // namespace loam_interface

#endif  // LOAM_INTERFACE__EXTRINSIC_MATH_HPP_
