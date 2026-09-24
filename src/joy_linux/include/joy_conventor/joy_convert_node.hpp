// Copyright 2026 ConQU2026

#ifndef JOY_CONVENTOR__JOY_CONVERT_NODE_HPP_
#define JOY_CONVENTOR__JOY_CONVERT_NODE_HPP_

#include <cstdint>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace joy_conventor
{

class JoyConvertNode : public rclcpp::Node
{
public:
  explicit JoyConvertNode(const rclcpp::NodeOptions & options);
  virtual ~JoyConvertNode();

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  int linear_axis_;
  int angular_axis_;
  double linear_x_scale_;
  double angular_z_scale_;
  double dead_zone_;
};

}  // namespace joy_conventor

#endif  // JOY_CONVENTOR__JOY_CONVERT_NODE_HPP_
