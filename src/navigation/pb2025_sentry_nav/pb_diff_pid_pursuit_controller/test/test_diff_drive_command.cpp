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
