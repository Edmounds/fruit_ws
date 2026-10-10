// Copyright 2026 ConQU2026

#include <gtest/gtest.h>

#include "joy_conventor/joy_convert_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoyConvertNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(JoyConvertNodeTest, DefaultParametersAndDirection)
{
  auto node = std::make_shared<joy_conventor::JoyConvertNode>(rclcpp::NodeOptions());
  EXPECT_GT(node->get_parameter("angular_z_scale").as_double(), 0.0);
  EXPECT_GT(node->get_parameter("linear_x_scale").as_double(), 0.0);
  EXPECT_EQ(node->get_parameter("rt_axis").as_int(), 4);
  EXPECT_EQ(node->get_parameter("lt_axis").as_int(), 5);
  EXPECT_EQ(node->get_parameter("angular_axis").as_int(), 2);
}

TEST_F(JoyConvertNodeTest, UninitializedTriggerYieldsZeroVelocity)
{
  auto node = std::make_shared<joy_conventor::JoyConvertNode>(rclcpp::NodeOptions());
  sensor_msgs::msg::Joy joy_msg;
  // Raw 0.0 before any arming / release event
  joy_msg.axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  auto twist = node->convert_joy_to_twist(joy_msg);
  EXPECT_DOUBLE_EQ(twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.z, 0.0);
}

TEST_F(JoyConvertNodeTest, TriggerReleasedYieldsZeroVelocity)
{
  auto node = std::make_shared<joy_conventor::JoyConvertNode>(rclcpp::NodeOptions());
  sensor_msgs::msg::Joy joy_msg;
  // Released triggers (+1.0 in ROS joy)
  joy_msg.axes = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0};

  auto twist = node->convert_joy_to_twist(joy_msg);
  EXPECT_DOUBLE_EQ(twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.z, 0.0);
}

TEST_F(JoyConvertNodeTest, FullRtYieldsPositiveLinearVelocity)
{
  auto node = std::make_shared<joy_conventor::JoyConvertNode>(rclcpp::NodeOptions());
  sensor_msgs::msg::Joy joy_msg;
  // Arm first with released state
  joy_msg.axes = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0};
  node->convert_joy_to_twist(joy_msg);

  // Full RT pressed (-1.0), LT released (+1.0)
  joy_msg.axes[4] = -1.0;
  joy_msg.axes[5] = 1.0;
  auto twist = node->convert_joy_to_twist(joy_msg);
  EXPECT_NEAR(twist.linear.x, 0.4, 1e-4);
  EXPECT_DOUBLE_EQ(twist.angular.z, 0.0);
}

TEST_F(JoyConvertNodeTest, FullLtYieldsNegativeLinearVelocity)
{
  auto node = std::make_shared<joy_conventor::JoyConvertNode>(rclcpp::NodeOptions());
  sensor_msgs::msg::Joy joy_msg;
  // Arm first
  joy_msg.axes = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0};
  node->convert_joy_to_twist(joy_msg);

  // RT released (+1.0), LT full pressed (-1.0)
  joy_msg.axes[4] = 1.0;
  joy_msg.axes[5] = -1.0;
  auto twist = node->convert_joy_to_twist(joy_msg);
  EXPECT_NEAR(twist.linear.x, -0.4, 1e-4);
  EXPECT_DOUBLE_EQ(twist.angular.z, 0.0);
}

TEST_F(JoyConvertNodeTest, EqualTriggerPressCancelsOut)
{
  auto node = std::make_shared<joy_conventor::JoyConvertNode>(rclcpp::NodeOptions());
  sensor_msgs::msg::Joy joy_msg;
  // Arm first
  joy_msg.axes = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0};
  node->convert_joy_to_twist(joy_msg);

  // Both fully pressed (-1.0)
  joy_msg.axes[4] = -1.0;
  joy_msg.axes[5] = -1.0;
  auto twist = node->convert_joy_to_twist(joy_msg);
  EXPECT_NEAR(twist.linear.x, 0.0, 1e-4);

  // Both half pressed (0.0)
  joy_msg.axes[4] = 0.0;
  joy_msg.axes[5] = 0.0;
  twist = node->convert_joy_to_twist(joy_msg);
  EXPECT_NEAR(twist.linear.x, 0.0, 1e-4);
}

TEST_F(JoyConvertNodeTest, DeadzoneFiltersSmallTriggerDepression)
{
  auto node = std::make_shared<joy_conventor::JoyConvertNode>(rclcpp::NodeOptions());
  sensor_msgs::msg::Joy joy_msg;
  // Arm first
  joy_msg.axes = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0};
  node->convert_joy_to_twist(joy_msg);

  // 2% depression: raw = 0.96 -> u_raw = 0.02 < deadzone 0.05
  joy_msg.axes[4] = 0.96;
  joy_msg.axes[5] = 1.0;
  auto twist = node->convert_joy_to_twist(joy_msg);
  EXPECT_DOUBLE_EQ(twist.linear.x, 0.0);
}
