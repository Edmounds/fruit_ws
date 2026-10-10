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

  geometry_msgs::msg::Twist convert_joy_to_twist(const sensor_msgs::msg::Joy & msg);

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  double compute_trigger_throttle(double axis_val, bool & armed) const;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  int rt_axis_;
  int lt_axis_;
  int angular_axis_;
  double linear_x_scale_;
  double angular_z_scale_;
  double trigger_dead_zone_;
  double dead_zone_;

  bool rt_armed_{false};
  bool lt_armed_{false};
};

}  // namespace joy_conventor

#endif  // JOY_CONVENTOR__JOY_CONVERT_NODE_HPP_
