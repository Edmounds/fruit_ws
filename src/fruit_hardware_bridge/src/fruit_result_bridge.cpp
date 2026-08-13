#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <memory>
#include <string>

#include "fruit_hardware_bridge/fruit_result_mapping.hpp"
#include "fruit_perception/msg/fruit_types.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace fruit_hardware_bridge
{

class FruitResultBridge : public rclcpp::Node
{
public:
  FruitResultBridge()
  : Node("fruit_result_bridge")
  {
    const auto input_topic = declare_parameter(
      "input_topic", "/fruit_perception/d435i/types");
    const auto output_topic = declare_parameter(
      "output_topic", "/fruit_hardware_bridge/d435i/result");

    publisher_ = create_publisher<std_msgs::msg::UInt8MultiArray>(output_topic, 10);
    subscription_ = create_subscription<fruit_perception::msg::FruitTypes>(
      input_topic, 10,
      std::bind(&FruitResultBridge::handle_detection, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "D435i fruit result bridge: %s -> %s",
      input_topic.c_str(), output_topic.c_str());
  }

private:
  void handle_detection(const fruit_perception::msg::FruitTypes::SharedPtr message)
  {
    const auto result = encode_detection(message->types);
    if (!message->types.empty() && result[0] == 0) {
      RCLCPP_WARN(get_logger(), "Unknown fruit label: %s", message->types.front().c_str());
    }

    std_msgs::msg::UInt8MultiArray output;
    output.data.assign(result.begin(), result.end());
    publisher_->publish(output);
  }

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<fruit_perception::msg::FruitTypes>::SharedPtr subscription_;
};

}  // namespace fruit_hardware_bridge

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fruit_hardware_bridge::FruitResultBridge>());
  rclcpp::shutdown();
  return 0;
}
