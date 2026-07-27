#include "pb_diff_teleop_twist_joy/pb_diff_teleop_twist_joy.hpp"

#include "pb_diff_teleop_twist_joy/teleop_command.hpp"

namespace pb_diff_teleop_twist_joy
{

TeleopTwistJoyNode::TeleopTwistJoyNode(const rclcpp::NodeOptions & options)
: Node("pb_diff_teleop_twist_joy", options)
{
  last_joy_time_ = now();
  enable_button_ = declare_parameter("enable_button", 4);
  turbo_button_ = declare_parameter("turbo_button", 5);
  linear_axis_ = declare_parameter("linear_axis", 1);
  angular_axis_ = declare_parameter("angular_axis", 0);
  normal_linear_scale_ = declare_parameter("normal_linear_scale", 0.4);
  normal_angular_scale_ = declare_parameter("normal_angular_scale", 0.8);
  turbo_linear_scale_ = declare_parameter("turbo_linear_scale", 0.6);
  turbo_angular_scale_ = declare_parameter("turbo_angular_scale", 1.0);

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_teleop", 10);
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&TeleopTwistJoyNode::joyCallback, this, std::placeholders::_1));
}

void TeleopTwistJoyNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  const auto current_time = now();
  const double dt = (current_time - last_joy_time_).seconds();
  last_joy_time_ = current_time;
  const auto pressed = [&msg](int button) {
      return button >= 0 && static_cast<std::size_t>(button) < msg->buttons.size() &&
             msg->buttons[button] != 0;
    };
  const bool enabled = pressed(enable_button_) || pressed(turbo_button_);
  if (enabled_last_callback_ && dt > 0.5) {
    publishZero();
    enabled_last_callback_ = false;
  }
  if (!enabled) {
    if (enabled_last_callback_) {
      publishZero();
    }
    enabled_last_callback_ = false;
    return;
  }

  const bool turbo = pressed(turbo_button_);
  const auto command = makeTeleopCommand(
    msg->axes, linear_axis_, angular_axis_,
    turbo ? turbo_linear_scale_ : normal_linear_scale_,
    turbo ? turbo_angular_scale_ : normal_angular_scale_);
  geometry_msgs::msg::Twist twist;
  twist.linear.x = command.linear_x;
  twist.angular.z = command.angular_z;
  cmd_vel_pub_->publish(twist);
  enabled_last_callback_ = true;
}

void TeleopTwistJoyNode::publishZero()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

}  // namespace pb_diff_teleop_twist_joy

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pb_diff_teleop_twist_joy::TeleopTwistJoyNode)
