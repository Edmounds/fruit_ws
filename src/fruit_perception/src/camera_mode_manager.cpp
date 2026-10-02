#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

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
    const auto camera_service = declare_parameter(
      "camera_parameter_service", "/camera/set_parameters");
    const auto camera_get_service = declare_parameter(
      "camera_get_parameters_service", "/camera/get_parameters");

    camera_client_ = create_client<rcl_interfaces::srv::SetParameters>(camera_service);
    camera_get_client_ =
      create_client<rcl_interfaces::srv::GetParameters>(camera_get_service);
    active_mode_publisher_ = create_publisher<std_msgs::msg::String>(
      active_topic, rclcpp::QoS(1).reliable().transient_local());
    mode_subscription_ = create_subscription<std_msgs::msg::String>(
      mode_topic, rclcpp::QoS(1).reliable(),
      std::bind(&CameraModeManager::on_mode, this, std::placeholders::_1));
    startup_timer_ = create_wall_timer(
      std::chrono::milliseconds(250), std::bind(&CameraModeManager::initialize_off, this));
  }

private:
  using GetParametersClient = rclcpp::Client<rcl_interfaces::srv::GetParameters>;
  using SetParametersClient = rclcpp::Client<rcl_interfaces::srv::SetParameters>;

  void initialize_off()
  {
    if (!camera_client_->service_is_ready() || !camera_get_client_->service_is_ready() ||
      camera_readiness_check_pending_)
    {
      return;
    }

    camera_readiness_check_pending_ = true;
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"enable_color", "enable_depth"};
    camera_get_client_->async_send_request(
      request, [this](GetParametersClient::SharedFuture future) {
        camera_readiness_check_pending_ = false;
        try {
          const auto response = future.get();
          if (response->values.size() != 2U ||
            response->values[0].type != rcl_interfaces::msg::ParameterType::PARAMETER_BOOL ||
            response->values[1].type != rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
          {
            return;
          }
        } catch (const std::exception & error) {
          RCLCPP_WARN(get_logger(), "Camera parameter readiness check failed: %s", error.what());
          return;
        }

        startup_timer_->cancel();
        state_.begin_startup_reset();
        execute_current_action();
      });
  }

  void on_mode(const std_msgs::msg::String::ConstSharedPtr message)
  {
    const auto target = parse_perception_mode(message->data);
    if (!target) {
      RCLCPP_ERROR(
        get_logger(), "Invalid perception mode '%s'; expected OFF, QR, or DETECT",
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
      case CameraAction::kStopCamera:
        set_camera_capture(false);
        break;
      case CameraAction::kStartCamera:
        set_camera_capture(true);
        break;
    }
  }

  void set_camera_capture(bool enabled)
  {
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(bool_parameter("enable_color", enabled));
    request->parameters.push_back(bool_parameter("enable_depth", enabled));
    camera_client_->async_send_request(
      request, [this](SetParametersClient::SharedFuture future) {
        bool success = false;
        try {
          const auto response = future.get();
          success = response->results.size() == 2U;
          for (const auto & result : response->results) {
            success = success && result.successful;
            if (!result.successful) {
              RCLCPP_ERROR(
                get_logger(), "Camera rejected stream parameter: %s", result.reason.c_str());
            }
          }
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "Camera parameter call failed: %s", error.what());
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
      const bool startup_failed = !state_.active_mode();
      state_.fail();
      RCLCPP_ERROR(get_logger(), "Camera transition failed; active mode was not changed");
      if (startup_failed) {
        startup_timer_->reset();
      }
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
  SetParametersClient::SharedPtr camera_client_;
  GetParametersClient::SharedPtr camera_get_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_mode_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscription_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  bool camera_readiness_check_pending_{false};
};
}  // namespace fruit_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fruit_perception::CameraModeManager>());
  rclcpp::shutdown();
  return 0;
}
