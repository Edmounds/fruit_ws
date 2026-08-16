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

#include <cmath>
#include <stdexcept>
#include <vector>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"

namespace loam_interface
{

// Point-LIO extrinsic: LiDAR pose expressed in the IMU (body) frame.
inline tf2::Transform makeLidarInImuTransform(
  const std::vector<double> & extrinsic_t, const std::vector<double> & extrinsic_r)
{
  if (extrinsic_t.size() != 3) {
    throw std::invalid_argument("extrinsic_T must contain 3 values");
  }
  if (extrinsic_r.size() != 9) {
    throw std::invalid_argument("extrinsic_R must contain 9 row-major values");
  }
  for (double value : extrinsic_t) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("extrinsic_T values must be finite");
    }
  }
  for (double value : extrinsic_r) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("extrinsic_R values must be finite");
    }
  }

  tf2::Matrix3x3 rotation(
    extrinsic_r[0], extrinsic_r[1], extrinsic_r[2],
    extrinsic_r[3], extrinsic_r[4], extrinsic_r[5],
    extrinsic_r[6], extrinsic_r[7], extrinsic_r[8]);
  tf2::Transform lidar_in_imu;
  lidar_in_imu.setBasis(rotation);
  lidar_in_imu.setOrigin(tf2::Vector3(extrinsic_t[0], extrinsic_t[1], extrinsic_t[2]));
  return lidar_in_imu;
}

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
