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

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>

#include "sensor_scan_generation/odometry_math.hpp"

TEST(OdometryMath, ExpressesWorldMotionInBaseFrame)
{
  tf2::Transform previous;
  previous.setIdentity();

  tf2::Quaternion yaw;
  yaw.setRPY(0.0, 0.0, M_PI_2);
  tf2::Transform current(yaw, tf2::Vector3(0.0, 1.0, 0.0));

  const auto twist = sensor_scan_generation::calculateBodyTwist(current, previous, 0.5);
  EXPECT_NEAR(twist.linear.x, 2.0, 1e-9);
  EXPECT_NEAR(twist.linear.y, 0.0, 1e-9);
  EXPECT_NEAR(twist.angular.z, M_PI, 1e-9);
}

TEST(OdometryMath, RejectsInvalidTimeStep)
{
  tf2::Transform transform;
  transform.setIdentity();
  const auto twist = sensor_scan_generation::calculateBodyTwist(transform, transform, 0.0);
  EXPECT_DOUBLE_EQ(twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.z, 0.0);
}

TEST(OdometryMath, IsInvariantToQuaternionSign)
{
  tf2::Transform previous;
  previous.setIdentity();

  tf2::Quaternion yaw;
  yaw.setRPY(0.0, 0.0, 0.1);
  tf2::Transform current(
    tf2::Quaternion(-yaw.x(), -yaw.y(), -yaw.z(), -yaw.w()), tf2::Vector3(0.0, 0.0, 0.0));

  const auto twist = sensor_scan_generation::calculateBodyTwist(current, previous, 0.1);
  EXPECT_NEAR(twist.angular.z, 1.0, 1e-9);
}
