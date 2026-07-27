#include <gtest/gtest.h>

#include "pb_diff_teleop_twist_joy/teleop_command.hpp"

TEST(TeleopCommand, AppliesNormalAndTurboLimits)
{
  const std::vector<float> axes{1.0F, -1.0F};
  auto command = pb_diff_teleop_twist_joy::makeTeleopCommand(axes, 1, 0, 0.4, 0.8);
  EXPECT_DOUBLE_EQ(command.linear_x, -0.4);
  EXPECT_DOUBLE_EQ(command.angular_z, 0.8);
  command = pb_diff_teleop_twist_joy::makeTeleopCommand(axes, 1, 0, 0.6, 1.0);
  EXPECT_DOUBLE_EQ(command.linear_x, -0.6);
  EXPECT_DOUBLE_EQ(command.angular_z, 1.0);
}

TEST(TeleopCommand, MissingAxisIsZero)
{
  const auto command = pb_diff_teleop_twist_joy::makeTeleopCommand({}, 1, 0, 0.4, 0.8);
  EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
}

TEST(TeleopCommand, ClampsOutOfRangeAxes)
{
  const std::vector<float> axes{2.0F, -2.0F};
  const auto command = pb_diff_teleop_twist_joy::makeTeleopCommand(axes, 1, 0, 0.4, 0.8);
  EXPECT_DOUBLE_EQ(command.linear_x, -0.4);
  EXPECT_DOUBLE_EQ(command.angular_z, 0.8);
}
