// Copyright 2026 ConQU2026

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "fruit_hardware_interfaces/msg/state_machine_command.hpp"
#include "joy_conventor/ac_pick_buttons.hpp"
#include "joy_conventor/joy_convert_node.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(JoyConvertNode, DefaultAngularScalePreservesRosYawDirection)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<joy_conventor::JoyConvertNode>(rclcpp::NodeOptions());
    EXPECT_GT(node->get_parameter("angular_z_scale").as_double(), 0.0);
  }
  rclcpp::shutdown();
}

TEST(JoyConvertNode, MapsConfiguredButtonPressEdgesToAcPickStatuses)
{
  using fruit_hardware_interfaces::msg::StateMachineCommand;

  const std::vector<int32_t> previous_buttons{0, 0, 0, 0, 0};

  auto statuses = joy_conventor::ac_pick_statuses_for_button_edges(
    {0, 0, 0, 1, 0}, previous_buttons, 3, 1);
  ASSERT_EQ(statuses.size(), 1U);
  EXPECT_EQ(statuses[0], StateMachineCommand::AC_PICK_LEFT);

  statuses = joy_conventor::ac_pick_statuses_for_button_edges(
    {0, 1, 0, 0, 0}, previous_buttons, 3, 1);
  ASSERT_EQ(statuses.size(), 1U);
  EXPECT_EQ(statuses[0], StateMachineCommand::AC_PICK_RIGHT);
}

TEST(JoyConvertNode, MapsYButtonPressEdgeToReturnInitialPose)
{
  using fruit_hardware_interfaces::msg::StateMachineCommand;

  EXPECT_EQ(StateMachineCommand::TASK_RETURN_INITIAL_POSE, 6);
  EXPECT_EQ(StateMachineCommand::RETURN_INITIAL_POSE, 0);
  EXPECT_TRUE(
    joy_conventor::return_initial_pose_requested_for_button_edge(
      {0, 0, 0, 0, 1}, {0, 0, 0, 0, 0}, 4));
}

TEST(JoyConvertNode, IgnoresHeldButtonsAndMissingButtonIndexes)
{
  auto statuses = joy_conventor::ac_pick_statuses_for_button_edges(
    {0, 1, 0, 1, 1}, {0, 1, 0, 1, 1}, 3, 1);
  EXPECT_TRUE(statuses.empty());

  statuses = joy_conventor::ac_pick_statuses_for_button_edges(
    {0, 0}, {0, 0}, 3, 1);
  EXPECT_TRUE(statuses.empty());

  EXPECT_FALSE(
    joy_conventor::return_initial_pose_requested_for_button_edge(
      {0, 0, 0, 0, 1}, {0, 0, 0, 0, 1}, 4));
  EXPECT_FALSE(
    joy_conventor::return_initial_pose_requested_for_button_edge(
      {0, 0}, {0, 0}, 4));
}
