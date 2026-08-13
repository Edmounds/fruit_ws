#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <openvino/openvino.hpp>
#include <openvino/runtime/properties.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "fruit_perception/detection_parser.hpp"
#include "fruit_perception/image_processing.hpp"
#include "fruit_perception/msg/fruit_types.hpp"

namespace fruit_perception
{
namespace
{

constexpr auto kStatsPeriod = std::chrono::seconds(5);

double percentile(std::vector<double> values, double fraction)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(
    std::ceil(fraction * static_cast<double>(values.size())) - 1.0);
  return values[std::min(index, values.size() - 1)];
}

}  // namespace

class FruitPerceptionNode : public rclcpp::Node
{
public:
  explicit FruitPerceptionNode(const rclcpp::NodeOptions & options)
  : Node("fruit_perception_node", options)
  {
    const auto package_share = ament_index_cpp::get_package_share_directory("fruit_perception");
    const auto model_path = declare_parameter("model_path", package_share + "/model/best.xml");
    confidence_threshold_ = static_cast<float>(declare_parameter("confidence_threshold", 0.50));

    const bool enable_left = declare_parameter("enable_left_camera", false);
    const bool enable_right = declare_parameter("enable_right_camera", false);
    const bool enable_d435i = declare_parameter("enable_d435i", true);
    const auto enabled_streams =
      static_cast<std::size_t>(enable_left) + static_cast<std::size_t>(enable_right) +
      static_cast<std::size_t>(enable_d435i);

    if (enabled_streams == 0U) {
      throw std::invalid_argument("at least one camera must be enabled");
    }
    if (!std::isfinite(confidence_threshold_) ||
      confidence_threshold_ < 0.0F || confidence_threshold_ > 1.0F)
    {
      throw std::invalid_argument("confidence_threshold must be between 0 and 1");
    }

    initialize_model(model_path, enabled_streams);

    if (enable_left) {
      add_stream(
        "left",
        declare_parameter("left_input_topic", "/left_camera/image_raw"),
        declare_parameter("left_output_topic", "/fruit_perception/left/types"),
        declare_parameter(
          "left_annotated_topic", "/fruit_perception/left/annotated_image"));
    }
    if (enable_right) {
      add_stream(
        "right",
        declare_parameter("right_input_topic", "/right_camera/image_raw"),
        declare_parameter("right_output_topic", "/fruit_perception/right/types"),
        declare_parameter(
          "right_annotated_topic", "/fruit_perception/right/annotated_image"));
    }
    if (enable_d435i) {
      add_stream(
        "d435i",
        declare_parameter("d435i_input_topic", "/camera/camera/color/image_raw"),
        declare_parameter("d435i_output_topic", "/fruit_perception/d435i/types"),
        declare_parameter(
          "d435i_annotated_topic", "/fruit_perception/d435i/annotated_image"));
    }

    stats_timer_ = create_wall_timer(kStatsPeriod, [this]() {log_stats();});
    RCLCPP_INFO(
      get_logger(), "Started %zu camera stream(s); confidence threshold %.2f",
      enabled_streams, confidence_threshold_);
  }

  ~FruitPerceptionNode() override
  {
    stopping_.store(true);
    stats_timer_.reset();
    subscriptions_.clear();
    for (const auto & stream : streams_) {
      wait_for_stream(stream);
    }
  }

private:
  struct Stream
  {
    Stream(
      std::string stream_name, ov::InferRequest infer_request,
      rclcpp::Publisher<msg::FruitTypes>::SharedPtr result_publisher,
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher,
      const ov::Shape & input_shape)
    : name(std::move(stream_name)),
      request(std::move(infer_request)),
      result_publisher(std::move(result_publisher)),
      image_publisher(std::move(image_publisher)),
      input(input_shape[0] * input_shape[1] * input_shape[2] * input_shape[3]),
      last_stats_at(std::chrono::steady_clock::now())
    {
      request.set_input_tensor(ov::Tensor(ov::element::f32, input_shape, input.data()));
    }

    std::string name;
    ov::InferRequest request;
    rclcpp::Publisher<msg::FruitTypes>::SharedPtr result_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
    std::vector<float> input;
    cv::Mat frame;
    LetterboxTransform transform{};
    std_msgs::msg::Header header;
    std::chrono::steady_clock::time_point started_at;
    std::chrono::steady_clock::time_point last_completed_at;
    double displayed_fps{0.0};
    std::atomic<bool> busy{false};
    std::mutex stats_mutex;
    std::uint64_t received{0};
    std::uint64_t inferred{0};
    std::uint64_t dropped{0};
    std::vector<double> latencies_ms;
    std::chrono::steady_clock::time_point last_stats_at;
  };

  void initialize_model(const std::string & model_path, std::size_t request_count)
  {
    const auto available_devices = core_.get_available_devices();
    if (std::find(available_devices.begin(), available_devices.end(), "GPU") ==
      available_devices.end())
    {
      throw std::runtime_error("OpenVINO GPU device is not available; CPU fallback is disabled");
    }

    const auto model = core_.read_model(model_path);
    const auto input = model->input();
    if (input.get_element_type() != ov::element::f32 ||
      input.get_shape() != ov::Shape{1, 3, kModelInputSize, kModelInputSize})
    {
      throw std::runtime_error("Expected model input f32 [1,3,640,640]");
    }

    compiled_model_ = core_.compile_model(
      model, "GPU",
      ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT),
      ov::hint::num_requests(static_cast<std::uint32_t>(request_count)));

    const auto execution_devices = compiled_model_.get_property(ov::execution_devices);
    if (execution_devices.size() != 1 || execution_devices.front() != "GPU.0") {
      throw std::runtime_error("Model did not compile exclusively to GPU.0");
    }
    if (compiled_model_.output().get_element_type() != ov::element::f32 ||
      compiled_model_.output().get_shape() != ov::Shape{1, 300, 6})
    {
      throw std::runtime_error("Expected model output f32 [1,300,6]");
    }

    input_shape_ = compiled_model_.input().get_shape();
    output_shape_ = compiled_model_.output().get_shape();
    RCLCPP_INFO(
      get_logger(), "OpenVINO model compiled successfully; execution device: %s",
      execution_devices.front().c_str());
  }

  void add_stream(
    const std::string & name, const std::string & input_topic,
    const std::string & output_topic, const std::string & annotated_topic)
  {
    auto stream = std::make_unique<Stream>(
      name, compiled_model_.create_infer_request(),
      create_publisher<msg::FruitTypes>(output_topic, rclcpp::QoS(10)),
      create_publisher<sensor_msgs::msg::Image>(annotated_topic, rclcpp::SensorDataQoS()),
      input_shape_);
    auto * stream_ptr = stream.get();
    stream->request.set_callback(
      [this, stream_ptr](std::exception_ptr error) {finish_inference(*stream_ptr, error);});
    streams_.push_back(std::move(stream));

    subscriptions_.push_back(
      create_subscription<sensor_msgs::msg::Image>(
        input_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this, stream_ptr](sensor_msgs::msg::Image::ConstSharedPtr image) {
          submit(*stream_ptr, image);
        }));
    RCLCPP_INFO(
      get_logger(), "%s: %s -> %s and %s", name.c_str(), input_topic.c_str(),
      output_topic.c_str(), annotated_topic.c_str());
  }

  void submit(Stream & stream, const sensor_msgs::msg::Image::ConstSharedPtr & image)
  {
    if (stopping_.load()) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(stream.stats_mutex);
      ++stream.received;
    }

    bool expected = false;
    if (!stream.busy.compare_exchange_strong(expected, true)) {
      std::lock_guard<std::mutex> lock(stream.stats_mutex);
      ++stream.dropped;
      return;
    }

    try {
      const auto converted =
        cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::BGR8);
      stream.frame = converted->image;
      stream.transform = letterbox_to_nchw(stream.frame, stream.input);
      stream.header = image->header;
      stream.started_at = std::chrono::steady_clock::now();
      stream.request.start_async();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "%s frame rejected: %s", stream.name.c_str(), error.what());
      {
        std::lock_guard<std::mutex> lock(stream.stats_mutex);
        ++stream.dropped;
      }
      stream.busy.store(false);
    }
  }

  void finish_inference(Stream & stream, std::exception_ptr error)
  {
    try {
      if (error) {
        std::rethrow_exception(error);
      }
      const auto output = stream.request.get_output_tensor();
      const auto detections = parse_detections(
        output.data<const float>(), output_shape_, confidence_threshold_);

      msg::FruitTypes result;
      result.header = stream.header;
      result.types.reserve(detections.size());
      for (const auto & detection : detections) {
        result.types.push_back(detection.type);
      }
      stream.result_publisher->publish(result);

      const auto now = std::chrono::steady_clock::now();
      if (stream.last_completed_at.time_since_epoch().count() != 0) {
        const double interval =
          std::chrono::duration<double>(now - stream.last_completed_at).count();
        const double instantaneous_fps = interval > 0.0 ? 1.0 / interval : 0.0;
        stream.displayed_fps = stream.displayed_fps == 0.0 ? instantaneous_fps :
          0.8 * stream.displayed_fps + 0.2 * instantaneous_fps;
      }
      stream.last_completed_at = now;

      cv::Mat annotated = stream.frame.clone();
      draw_annotations(annotated, detections, stream.transform, stream.displayed_fps);
      const auto annotated_message = cv_bridge::CvImage(
        stream.header, sensor_msgs::image_encodings::BGR8, annotated).toImageMsg();
      stream.image_publisher->publish(*annotated_message);

      const auto elapsed =
        std::chrono::duration<double, std::milli>(now - stream.started_at).count();
      std::lock_guard<std::mutex> lock(stream.stats_mutex);
      ++stream.inferred;
      stream.latencies_ms.push_back(elapsed);
    } catch (const std::exception & callback_error) {
      RCLCPP_ERROR(
        get_logger(), "%s inference failed: %s", stream.name.c_str(), callback_error.what());
      std::lock_guard<std::mutex> lock(stream.stats_mutex);
      ++stream.dropped;
    }
    stream.busy.store(false);
  }

  void log_stats()
  {
    for (const auto & stream : streams_) {
      log_stream_stats(*stream);
    }
  }

  void log_stream_stats(Stream & stream)
  {
    std::uint64_t received;
    std::uint64_t inferred;
    std::uint64_t dropped;
    std::vector<double> latencies;
    const auto now = std::chrono::steady_clock::now();
    double elapsed_seconds;
    {
      std::lock_guard<std::mutex> lock(stream.stats_mutex);
      received = std::exchange(stream.received, 0);
      inferred = std::exchange(stream.inferred, 0);
      dropped = std::exchange(stream.dropped, 0);
      latencies.swap(stream.latencies_ms);
      elapsed_seconds = std::chrono::duration<double>(now - stream.last_stats_at).count();
      stream.last_stats_at = now;
    }

    RCLCPP_INFO(
      get_logger(),
      "%s stats: received=%lu inferred=%lu dropped=%lu inference_fps=%.2f "
      "latency_p50_ms=%.2f latency_p95_ms=%.2f",
      stream.name.c_str(), static_cast<unsigned long>(received),
      static_cast<unsigned long>(inferred), static_cast<unsigned long>(dropped),
      static_cast<double>(inferred) / elapsed_seconds,
      percentile(latencies, 0.50), percentile(latencies, 0.95));
  }

  static void wait_for_stream(const std::unique_ptr<Stream> & stream)
  {
    try {
      stream->request.wait();
    } catch (const std::exception &) {
    }
  }

  ov::Core core_;
  ov::CompiledModel compiled_model_;
  ov::Shape input_shape_;
  ov::Shape output_shape_;
  float confidence_threshold_{0.50F};
  std::atomic<bool> stopping_{false};
  std::vector<std::unique_ptr<Stream>> streams_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace fruit_perception

RCLCPP_COMPONENTS_REGISTER_NODE(fruit_perception::FruitPerceptionNode)
