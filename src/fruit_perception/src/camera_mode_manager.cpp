#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <chrono>
#include <cstddef>
#include <exception>
#include <functional>
#include <memory>
#include <string>

#include "fruit_perception/perception_mode.hpp"

namespace fruit_perception
{
class CameraModeManager : public rclcpp::Node
{
public:
  CameraModeManager()
  : Node("camera_mode_manager")
  {
    const auto mode_topic = declare_parameter(
      "perception_mode_topic", "/fruit_mission/perception_mode");
    const auto active_topic = declare_parameter(
      "active_mode_topic", "/fruit_perception/active_mode");
    const auto left_service = declare_parameter(
      "left_capture_service", "/left_camera/set_capture");
    const auto right_service = declare_parameter(
      "right_capture_service", "/right_camera/set_capture");
    const auto d435i_service = declare_parameter(
      "d435i_parameter_service", "/camera/camera/set_parameters");

    left_client_ = create_client<std_srvs::srv::SetBool>(left_service);
    right_client_ = create_client<std_srvs::srv::SetBool>(right_service);
    d435i_client_ = create_client<rcl_interfaces::srv::SetParameters>(d435i_service);
    active_mode_publisher_ = create_publisher<std_msgs::msg::String>(
      active_topic, rclcpp::QoS(1).reliable().transient_local());
    mode_subscription_ = create_subscription<std_msgs::msg::String>(
      mode_topic, rclcpp::QoS(1).reliable(),
      std::bind(&CameraModeManager::on_mode, this, std::placeholders::_1));
    startup_timer_ = create_wall_timer(
      std::chrono::milliseconds(250), std::bind(&CameraModeManager::initialize_off, this));
  }

private:
  using SetBoolClient = rclcpp::Client<std_srvs::srv::SetBool>;
  using SetParametersClient = rclcpp::Client<rcl_interfaces::srv::SetParameters>;

  void initialize_off()
  {
    if (!left_client_->service_is_ready() || !right_client_->service_is_ready() ||
      !d435i_client_->service_is_ready())
    {
      return;
    }
    startup_timer_->cancel();
    state_.begin_startup_reset();
    execute_current_action();
  }

  void on_mode(const std_msgs::msg::String::ConstSharedPtr message)
  {
    const auto target = parse_perception_mode(message->data);
    if (!target) {
      RCLCPP_ERROR(
        get_logger(), "Invalid perception mode '%s'; expected OFF, SIDES, QR, or TREE",
        message->data.c_str());
      return;
    }
    if (!state_.active_mode()) {
      RCLCPP_ERROR(get_logger(), "Camera manager is not ready; mode remains uninitialized");
      return;
    }
    if (state_.transitioning()) {
      RCLCPP_ERROR(get_logger(), "Camera transition already in progress; mode unchanged");
      return;
    }

    state_.begin(*target);
    if (!state_.transitioning()) {
      publish_active_mode();
      return;
    }
    execute_current_action();
  }

  void execute_current_action()
  {
    const auto action = state_.current_action();
    if (!action) {
      publish_active_mode();
      return;
    }
    switch (*action) {
      case CameraAction::kStopSides:
        set_side_capture(false);
        break;
      case CameraAction::kStopD435i:
        set_d435i_capture(false);
        break;
      case CameraAction::kStartSides:
        set_side_capture(true);
        break;
      case CameraAction::kStartD435i:
        set_d435i_capture(true);
        break;
    }
  }

  void set_side_capture(bool enabled)
  {
    side_results_remaining_ = 2U;
    side_step_succeeded_ = true;
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    const auto callback = [this](SetBoolClient::SharedFuture future) {
        bool success = false;
        try {
          const auto response = future.get();
          success = usb_capture_response_succeeded(response->success, response->message);
          if (!success) {
            RCLCPP_ERROR(
              get_logger(), "USB camera rejected set_capture: %s", response->message.c_str());
          }
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "USB set_capture call failed: %s", error.what());
        }
        side_step_succeeded_ = side_step_succeeded_ && success;
        --side_results_remaining_;
        if (side_results_remaining_ == 0U) {
          finish_action(side_step_succeeded_);
        }
      };
    left_client_->async_send_request(request, callback);
    right_client_->async_send_request(request, callback);
  }

  void set_d435i_capture(bool enabled)
  {
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(bool_parameter("enable_color", enabled));
    request->parameters.push_back(bool_parameter("enable_depth", enabled));
    d435i_client_->async_send_request(
      request, [this](SetParametersClient::SharedFuture future) {
        bool success = false;
        try {
          const auto response = future.get();
          success = response->results.size() == 2U;
          for (const auto & result : response->results) {
            success = success && result.successful;
            if (!result.successful) {
              RCLCPP_ERROR(
                get_logger(), "D435i rejected stream parameter: %s", result.reason.c_str());
            }
          }
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "D435i parameter call failed: %s", error.what());
        }
        finish_action(success);
      });
  }

  static rcl_interfaces::msg::Parameter bool_parameter(
    const std::string & name, bool value)
  {
    rcl_interfaces::msg::Parameter parameter;
    parameter.name = name;
    parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    parameter.value.bool_value = value;
    return parameter;
  }

  void finish_action(bool success)
  {
    if (!success) {
      state_.fail();
      RCLCPP_ERROR(get_logger(), "Camera transition failed; active mode was not changed");
      return;
    }
    state_.complete_action();
    if (state_.transitioning()) {
      execute_current_action();
    } else {
      publish_active_mode();
    }
  }

  void publish_active_mode()
  {
    const auto active = state_.active_mode();
    if (!active) {
      return;
    }
    std_msgs::msg::String message;
    message.data = to_string(*active);
    active_mode_publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "Perception camera mode is now %s", message.data.c_str());
  }

  CameraModeState state_;
  SetBoolClient::SharedPtr left_client_;
  SetBoolClient::SharedPtr right_client_;
  SetParametersClient::SharedPtr d435i_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_mode_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscription_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  std::size_t side_results_remaining_{0U};
  bool side_step_succeeded_{false};
};
}  // namespace fruit_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fruit_perception::CameraModeManager>());
  rclcpp::shutdown();
  return 0;
}
