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

#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"

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

// Convert odom -> lidar to odom -> base_link using the calibrated base -> lidar transform.
inline tf2::Transform odomToBase(
  const tf2::Transform & odom_to_lidar, const tf2::Transform & base_to_lidar)
{
  return odom_to_lidar * base_to_lidar.inverse();
}

inline geometry_msgs::msg::Twist calculateBodyTwist(
  const tf2::Transform & current, const tf2::Transform & previous, double dt)
{
  geometry_msgs::msg::Twist twist;
  if (!std::isfinite(dt) || dt <= 0.0) {
    return twist;
  }

  const tf2::Matrix3x3 parent_to_child = current.getBasis().transpose();
  const tf2::Vector3 linear_parent = (current.getOrigin() - previous.getOrigin()) / dt;
  const tf2::Vector3 linear_child = parent_to_child * linear_parent;
  twist.linear.x = linear_child.x();
  twist.linear.y = linear_child.y();
  twist.linear.z = linear_child.z();

  tf2::Quaternion rotation_delta =
    current.getRotation() * previous.getRotation().inverse();
  rotation_delta.normalize();
  const double angle = rotation_delta.getAngleShortestPath();
  if (angle > 1e-12) {
    const tf2::Vector3 angular_parent = rotation_delta.getAxis() * angle / dt;
    const tf2::Vector3 angular_child = parent_to_child * angular_parent;
    twist.angular.x = angular_child.x();
    twist.angular.y = angular_child.y();
    twist.angular.z = angular_child.z();
  }
  return twist;
}

}  // namespace loam_interface

#endif  // LOAM_INTERFACE__EXTRINSIC_MATH_HPP_
