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

#include <gtest/gtest.h>

#include "pb_diff_pid_pursuit_controller/diff_drive_command.hpp"

using pb_diff_pid_pursuit_controller::makeDiffDriveCommand;

TEST(DiffDriveCommand, DrivesStraightAndTurns)
{
  auto command = makeDiffDriveCommand(0.6, 0.0, 0.0, 2.0, 0.0, 0.7, 0.2, 0.6, 0.2, 1.0);
  EXPECT_DOUBLE_EQ(command.linear_x, 0.6);
  EXPECT_DOUBLE_EQ(command.angular_z, 0.0);

  command = makeDiffDriveCommand(0.6, 0.5, 0.5, 2.0, 0.0, 0.7, 0.2, 0.6, 0.2, 1.0);
  EXPECT_GT(command.linear_x, 0.0);
  EXPECT_LT(command.linear_x, 0.6);
  EXPECT_GT(command.angular_z, 0.0);

  command = makeDiffDriveCommand(0.6, -0.5, -0.5, 2.0, 0.0, 0.7, 0.2, 0.6, 0.2, 1.0);
  EXPECT_LT(command.angular_z, 0.0);
}

TEST(DiffDriveCommand, RotatesInPlaceAndAlignsGoal)
{
  auto command = makeDiffDriveCommand(0.6, 1.5, 0.8, 2.0, 0.0, 0.7, 0.2, 0.6, 0.2, 1.0);
  EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_z, 1.0);

  command = makeDiffDriveCommand(0.6, 0.4, 0.0, 0.19, 0.4, 0.7, 0.2, 0.6, 0.2, 1.0);
  EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_z, 0.4);
}
