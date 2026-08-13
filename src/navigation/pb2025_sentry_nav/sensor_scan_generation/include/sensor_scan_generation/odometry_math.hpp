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

#ifndef SENSOR_SCAN_GENERATION__ODOMETRY_MATH_HPP_
#define SENSOR_SCAN_GENERATION__ODOMETRY_MATH_HPP_

#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Transform.h"

namespace sensor_scan_generation
{

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

}  // namespace sensor_scan_generation

#endif  // SENSOR_SCAN_GENERATION__ODOMETRY_MATH_HPP_
