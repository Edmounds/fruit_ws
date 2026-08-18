// Copyright 2026 ConQU2026

/*
 * joy_linux_node main entry point
 */

#include <memory>

#include "joy_linux/joy_linux_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<joy_linux::JoyLinux>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
