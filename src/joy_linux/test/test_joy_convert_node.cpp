// Copyright 2026 ConQU2026

#include <gtest/gtest.h>

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
