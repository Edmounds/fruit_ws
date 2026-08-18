// Copyright 2026 ConQU2026

#include "joy_conventor/joy_convert_node.hpp"
#include <cmath>

#include "joy_conventor/ac_pick_buttons.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace joy_conventor
{

namespace
{

bool is_button_pressed(const std::vector<int32_t> & buttons, int button)
{
  if (button < 0 || static_cast<std::size_t>(button) >= buttons.size()) {
    return false;
  }
  return buttons[button] != 0;
}

}  // namespace

std::vector<uint8_t> ac_pick_statuses_for_button_edges(
  const std::vector<int32_t> & buttons,
  const std::vector<int32_t> & previous_buttons,
  int left_button,
  int right_button)
{
  std::vector<uint8_t> statuses;
  if (is_button_pressed(buttons, left_button) &&
    !is_button_pressed(previous_buttons, left_button))
  {
    statuses.push_back(fruit_hardware_interfaces::msg::StateMachineCommand::AC_PICK_LEFT);
  }
  if (is_button_pressed(buttons, right_button) &&
    !is_button_pressed(previous_buttons, right_button))
  {
    statuses.push_back(fruit_hardware_interfaces::msg::StateMachineCommand::AC_PICK_RIGHT);
  }
  return statuses;
}

bool return_initial_pose_requested_for_button_edge(
  const std::vector<int32_t> & buttons,
  const std::vector<int32_t> & previous_buttons,
  int button)
{
  return is_button_pressed(buttons, button) && !is_button_pressed(previous_buttons, button);
}

JoyConvertNode::JoyConvertNode(const rclcpp::NodeOptions & options)
: Node("joy_convert_node", options)
{
  this->declare_parameter("linear_axis", 1);
  this->declare_parameter("angular_axis", 2);
  this->declare_parameter("ac_pick_left_button", 3);
  this->declare_parameter("ac_pick_right_button", 1);
  this->declare_parameter("return_initial_pose_button", 4);
  this->declare_parameter("linear_x_scale", 2.0);
  this->declare_parameter("angular_z_scale", 4.0);
  this->declare_parameter("dead_zone", 0.1);

  linear_axis_ = static_cast<int>(this->get_parameter("linear_axis").as_int());
  angular_axis_ = static_cast<int>(this->get_parameter("angular_axis").as_int());
  ac_pick_left_button_ = static_cast<int>(this->get_parameter("ac_pick_left_button").as_int());
  ac_pick_right_button_ = static_cast<int>(this->get_parameter("ac_pick_right_button").as_int());
  return_initial_pose_button_ =
    static_cast<int>(this->get_parameter("return_initial_pose_button").as_int());
  linear_x_scale_ = this->get_parameter("linear_x_scale").as_double();
  angular_z_scale_ = this->get_parameter("angular_z_scale").as_double();
  dead_zone_ = this->get_parameter("dead_zone").as_double();

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10,
    std::bind(&JoyConvertNode::joy_callback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  state_machine_command_pub_ =
    this->create_publisher<fruit_hardware_interfaces::msg::StateMachineCommand>(
    "fruit_hardware_bridge/state_machine/command", 10);

  RCLCPP_INFO(this->get_logger(), "JoyConvertNode started component mode");
}

JoyConvertNode::~JoyConvertNode()
{
}

void JoyConvertNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (!msg->axes.empty()) {
    auto twist = std::make_unique<geometry_msgs::msg::Twist>();
    const auto read_axis = [&msg](int axis) {
        if (axis < 0 || static_cast<std::size_t>(axis) >= msg->axes.size()) {
          return 0.0;
        }
        return static_cast<double>(msg->axes[axis]);
      };

    const double linear_value = read_axis(linear_axis_);
    const double angular_value = read_axis(angular_axis_);
    twist->linear.x =
      (std::abs(linear_value) < dead_zone_) ? 0.0 : linear_value * linear_x_scale_;
    twist->angular.z =
      (std::abs(angular_value) < dead_zone_) ? 0.0 : angular_value * angular_z_scale_;

    cmd_vel_pub_->publish(std::move(twist));
  }

  for (const auto task_status : ac_pick_statuses_for_button_edges(
      msg->buttons, previous_buttons_, ac_pick_left_button_, ac_pick_right_button_))
  {
    auto command =
      std::make_unique<fruit_hardware_interfaces::msg::StateMachineCommand>();
    command->task_id = fruit_hardware_interfaces::msg::StateMachineCommand::TASK_AC_PICK;
    command->task_status = task_status;
    state_machine_command_pub_->publish(std::move(command));
  }

  if (return_initial_pose_requested_for_button_edge(
      msg->buttons, previous_buttons_, return_initial_pose_button_))
  {
    auto command =
      std::make_unique<fruit_hardware_interfaces::msg::StateMachineCommand>();
    command->task_id =
      fruit_hardware_interfaces::msg::StateMachineCommand::TASK_RETURN_INITIAL_POSE;
    command->task_status =
      fruit_hardware_interfaces::msg::StateMachineCommand::RETURN_INITIAL_POSE;
    state_machine_command_pub_->publish(std::move(command));
  }

  previous_buttons_ = msg->buttons;
}

}  // namespace joy_conventor

RCLCPP_COMPONENTS_REGISTER_NODE(joy_conventor::JoyConvertNode)
