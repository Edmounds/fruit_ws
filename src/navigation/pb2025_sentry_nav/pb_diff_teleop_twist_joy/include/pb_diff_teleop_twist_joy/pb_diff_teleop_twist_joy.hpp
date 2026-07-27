#ifndef PB_DIFF_TELEOP_TWIST_JOY__PB_DIFF_TELEOP_TWIST_JOY_HPP_
#define PB_DIFF_TELEOP_TWIST_JOY__PB_DIFF_TELEOP_TWIST_JOY_HPP_

#include <cstddef>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace pb_diff_teleop_twist_joy
{

class TeleopTwistJoyNode : public rclcpp::Node
{
public:
  explicit TeleopTwistJoyNode(const rclcpp::NodeOptions & options);

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void publishZero();

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  int enable_button_;
  int turbo_button_;
  int linear_axis_;
  int angular_axis_;
  double normal_linear_scale_;
  double normal_angular_scale_;
  double turbo_linear_scale_;
  double turbo_angular_scale_;
  bool enabled_last_callback_{false};
  rclcpp::Time last_joy_time_;
};

}  // namespace pb_diff_teleop_twist_joy

#endif  // PB_DIFF_TELEOP_TWIST_JOY__PB_DIFF_TELEOP_TWIST_JOY_HPP_
