// Copyright 2026 ConQU2026

#include "joy_conventor/joy_convert_node.hpp"
#include <cmath>

#include "rclcpp_components/register_node_macro.hpp"

namespace joy_conventor
{

JoyConvertNode::JoyConvertNode(const rclcpp::NodeOptions & options)
: Node("joy_convert_node", options)
{
  this->declare_parameter("rt_axis", 4);
  this->declare_parameter("lt_axis", 5);
  this->declare_parameter("angular_axis", 2);
  this->declare_parameter("ac_pick_left_button", 3);
  this->declare_parameter("ac_pick_right_button", 1);
  this->declare_parameter("return_initial_pose_button", 4);
  this->declare_parameter("linear_x_scale", 0.4);
  this->declare_parameter("angular_z_scale", 0.8);
  this->declare_parameter("trigger_dead_zone", 0.05);
  this->declare_parameter("dead_zone", 0.0);

  rt_axis_ = static_cast<int>(this->get_parameter("rt_axis").as_int());
  lt_axis_ = static_cast<int>(this->get_parameter("lt_axis").as_int());
  angular_axis_ = static_cast<int>(this->get_parameter("angular_axis").as_int());
  linear_x_scale_ = this->get_parameter("linear_x_scale").as_double();
  angular_z_scale_ = this->get_parameter("angular_z_scale").as_double();
  trigger_dead_zone_ = this->get_parameter("trigger_dead_zone").as_double();
  dead_zone_ = this->get_parameter("dead_zone").as_double();

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10,
    std::bind(&JoyConvertNode::joy_callback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  RCLCPP_INFO(this->get_logger(), "JoyConvertNode started component mode (RT/LT trigger teleop)");
}

JoyConvertNode::~JoyConvertNode()
{
}

double JoyConvertNode::compute_trigger_throttle(double axis_val, bool & armed) const
{
  if (!armed) {
    if (axis_val > 0.7) {
      armed = true;
    } else if (axis_val < -0.1) {
      armed = true;
    } else {
      return 0.0;
    }
  }

  const double clamped = std::max(-1.0, std::min(1.0, axis_val));
  const double u_raw = (1.0 - clamped) / 2.0;
  if (u_raw <= trigger_dead_zone_) {
    return 0.0;
  }
  const double effective_range = 1.0 - trigger_dead_zone_;
  if (effective_range <= 1e-6) {
    return 0.0;
  }
  const double u_scaled = (u_raw - trigger_dead_zone_) / effective_range;
  return std::min(1.0, std::max(0.0, u_scaled));
}

geometry_msgs::msg::Twist JoyConvertNode::convert_joy_to_twist(const sensor_msgs::msg::Joy & msg)
{
  geometry_msgs::msg::Twist twist;
  if (msg.axes.empty()) {
    return twist;
  }

  const auto read_axis = [&msg](int axis) {
      if (axis < 0 || static_cast<std::size_t>(axis) >= msg.axes.size()) {
        return 0.0;
      }
      return static_cast<double>(msg.axes[axis]);
    };

  const double rt_throttle = compute_trigger_throttle(read_axis(rt_axis_), rt_armed_);
  const double lt_throttle = compute_trigger_throttle(read_axis(lt_axis_), lt_armed_);
  const double angular_value = read_axis(angular_axis_);

  twist.linear.x = (rt_throttle - lt_throttle) * linear_x_scale_;
  twist.angular.z =
    (std::abs(angular_value) < dead_zone_) ? 0.0 : angular_value * angular_z_scale_;

  return twist;
}

void JoyConvertNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (!msg->axes.empty()) {
    auto twist = std::make_unique<geometry_msgs::msg::Twist>(convert_joy_to_twist(*msg));
    cmd_vel_pub_->publish(std::move(twist));
  }
}

}  // namespace joy_conventor

RCLCPP_COMPONENTS_REGISTER_NODE(joy_conventor::JoyConvertNode)
