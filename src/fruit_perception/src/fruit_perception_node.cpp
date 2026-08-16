#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <openvino/openvino.hpp>
#include <openvino/runtime/properties.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "fruit_perception/detection_parser.hpp"
#include "fruit_perception/detection_output.hpp"
#include "fruit_perception/image_processing.hpp"
#include "fruit_perception/msg/fruit_detection.hpp"
#include "fruit_perception/msg/fruit_detections.hpp"
#include "fruit_perception/msg/fruit_types.hpp"
#include "fruit_perception/perception_mode.hpp"

namespace fruit_perception
{
class FruitPerceptionNode : public rclcpp::Node
{
public:
  explicit FruitPerceptionNode(const rclcpp::NodeOptions & options)
  : Node("fruit_perception_node", options)
  {
    const auto package_share = ament_index_cpp::get_package_share_directory("fruit_perception");
    const auto model_path = declare_parameter("model_path", package_share + "/model/best.xml");
    confidence_threshold_ = static_cast<float>(declare_parameter("confidence_threshold", 0.50));
    const auto perception_mode_topic = declare_parameter(
      "perception_mode_topic", "/fruit_mission/perception_mode");
    mode_subscription_ = create_subscription<std_msgs::msg::String>(
      perception_mode_topic, rclcpp::QoS(1).reliable(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        const auto mode = parse_perception_mode(message->data);
        if (!mode) {
          RCLCPP_ERROR(
            get_logger(), "Invalid perception mode '%s'; inference mode unchanged",
            message->data.c_str());
          return;
        }
        perception_mode_.store(*mode);
      });

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
      add_rgbd_stream(
        declare_parameter("d435i_rgbd_topic", "/camera/camera/rgbd"),
        declare_parameter("d435i_output_topic", "/fruit_perception/d435i/types"),
        declare_parameter(
          "d435i_annotated_topic", "/fruit_perception/d435i/annotated_image"),
        declare_parameter(
          "d435i_target_point_topic", "/fruit_perception/d435i/target_point"),
        declare_parameter(
          "d435i_detections_topic", "/fruit_perception/d435i/detections"));
    }

    RCLCPP_INFO(
      get_logger(), "Started %zu camera stream(s); confidence threshold %.2f",
      enabled_streams, confidence_threshold_);
  }

  ~FruitPerceptionNode() override
  {
    stopping_.store(true);
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
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher)
    : name(std::move(stream_name)),
      request(std::move(infer_request)),
      result_publisher(std::move(result_publisher)),
      image_publisher(std::move(image_publisher)),
      input_image(kModelInputSize, kModelInputSize, CV_8UC3, cv::Scalar(114, 114, 114))
    {
      // The u8 NHWC tensor wraps the letterbox buffer directly; conversion to
      // f32 NCHW and the /255 scaling are fused into the compiled model.
      request.set_input_tensor(
        ov::Tensor(
          ov::element::u8,
          ov::Shape{
          1U, static_cast<std::size_t>(kModelInputSize),
          static_cast<std::size_t>(kModelInputSize), 3U},
          input_image.data));
    }

    std::string name;
    ov::InferRequest request;
    rclcpp::Publisher<msg::FruitTypes>::SharedPtr result_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_point_publisher;
    rclcpp::Publisher<msg::FruitDetections>::SharedPtr detections_publisher;
    cv::Mat input_image;
    cv_bridge::CvImageConstPtr frame;
    cv_bridge::CvImageConstPtr aligned_depth;
    sensor_msgs::msg::CameraInfo depth_camera_info;
    LetterboxTransform transform{};
    std_msgs::msg::Header header;
    std::chrono::steady_clock::time_point last_completed_at;
    double displayed_fps{0.0};
    std::atomic<bool> busy{false};
  };

  void initialize_model(const std::string & model_path, std::size_t request_count)
  {
    const auto available_devices = core_.get_available_devices();
    if (std::find(available_devices.begin(), available_devices.end(), "GPU") ==
      available_devices.end())
    {
      throw std::runtime_error("OpenVINO GPU device is not available; CPU fallback is disabled");
    }

    auto model = core_.read_model(model_path);
    const auto input = model->input();
    if (input.get_element_type() != ov::element::f32 ||
      input.get_shape() != ov::Shape{1, 3, kModelInputSize, kModelInputSize})
    {
      throw std::runtime_error("Expected model input f32 [1,3,640,640]");
    }

    // Fold preprocessing into the model so the GPU performs the u8 -> f32
    // conversion, /255 scaling, and NHWC -> NCHW transpose. The host only
    // supplies the letterboxed u8 RGB image.
    ov::preprocess::PrePostProcessor preprocessor(model);
    auto & input_info = preprocessor.input();
    input_info.tensor().set_element_type(ov::element::u8).set_layout("NHWC");
    input_info.preprocess().convert_element_type(ov::element::f32).scale(255.0F);
    input_info.model().set_layout("NCHW");
    model = preprocessor.build();

    compiled_model_ = core_.compile_model(
      model, "GPU",
      ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT),
      ov::hint::num_requests(static_cast<std::uint32_t>(request_count)));

    const auto execution_devices = compiled_model_.get_property(ov::execution_devices);
    if (execution_devices.size() != 1 || execution_devices.front() != "GPU.0") {
      throw std::runtime_error("Model did not compile exclusively to GPU.0");
    }
    if (compiled_model_.input().get_element_type() != ov::element::u8 ||
      compiled_model_.input().get_shape() !=
      ov::Shape{1, kModelInputSize, kModelInputSize, 3})
    {
      throw std::runtime_error("Expected compiled model input u8 [1,640,640,3]");
    }
    if (compiled_model_.output().get_element_type() != ov::element::f32 ||
      compiled_model_.output().get_shape() != ov::Shape{1, 300, 6})
    {
      throw std::runtime_error("Expected model output f32 [1,300,6]");
    }

    output_shape_ = compiled_model_.output().get_shape();
    RCLCPP_INFO(
      get_logger(), "OpenVINO model compiled successfully; execution device: %s",
      execution_devices.front().c_str());
  }

  void add_rgbd_stream(
    const std::string & input_topic, const std::string & output_topic,
    const std::string & annotated_topic, const std::string & target_point_topic,
    const std::string & detections_topic)
  {
    auto stream = std::make_unique<Stream>(
      "d435i", compiled_model_.create_infer_request(),
      create_publisher<msg::FruitTypes>(output_topic, rclcpp::QoS(10)),
      create_publisher<sensor_msgs::msg::Image>(annotated_topic, rclcpp::SensorDataQoS()));
    stream->target_point_publisher =
      create_publisher<geometry_msgs::msg::PointStamped>(target_point_topic, rclcpp::QoS(10));
    stream->detections_publisher =
      create_publisher<msg::FruitDetections>(detections_topic, rclcpp::QoS(10));
    auto * stream_ptr = stream.get();
    stream->request.set_callback(
      [this, stream_ptr](std::exception_ptr error) {finish_inference(*stream_ptr, error);});
    streams_.push_back(std::move(stream));

    subscriptions_.push_back(
      create_subscription<realsense2_camera_msgs::msg::RGBD>(
        input_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this, stream_ptr](realsense2_camera_msgs::msg::RGBD::ConstSharedPtr rgbd) {
          submit_rgbd(*stream_ptr, rgbd);
        }));
    RCLCPP_INFO(
      get_logger(), "d435i: %s -> %s, %s, %s and %s", input_topic.c_str(),
      output_topic.c_str(), annotated_topic.c_str(), target_point_topic.c_str(),
      detections_topic.c_str());
  }

  void add_stream(
    const std::string & name, const std::string & input_topic,
    const std::string & output_topic, const std::string & annotated_topic)
  {
    auto stream = std::make_unique<Stream>(
      name, compiled_model_.create_infer_request(),
      create_publisher<msg::FruitTypes>(output_topic, rclcpp::QoS(10)),
      create_publisher<sensor_msgs::msg::Image>(annotated_topic, rclcpp::SensorDataQoS()));
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
    if (stopping_.load() || !runs_side_inference(perception_mode_.load())) {
      return;
    }

    bool expected = false;
    if (!stream.busy.compare_exchange_strong(expected, true)) {
      return;
    }

    try {
      // toCvShare avoids a full-frame copy when the source already publishes
      // rgb8 (usb_cam mjpeg2rgb does); the returned pointer
      // keeps the message alive for the duration of the inference.
      stream.frame = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::RGB8);
      stream.transform = letterbox_image(stream.frame->image, stream.input_image);
      stream.header = image->header;
      stream.request.start_async();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "%s frame rejected: %s", stream.name.c_str(), error.what());
      stream.busy.store(false);
    }
  }

  void submit_rgbd(
    Stream & stream, const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr & rgbd)
  {
    if (stopping_.load() || !runs_d435i_inference(perception_mode_.load())) {
      return;
    }

    bool expected = false;
    if (!stream.busy.compare_exchange_strong(expected, true)) {
      return;
    }

    try {
      stream.frame = cv_bridge::toCvShare(
        rgbd->rgb, rgbd, sensor_msgs::image_encodings::RGB8);
      stream.aligned_depth = cv_bridge::toCvShare(
        rgbd->depth, rgbd, sensor_msgs::image_encodings::TYPE_16UC1);
      stream.depth_camera_info = rgbd->depth_camera_info;
      stream.transform = letterbox_image(stream.frame->image, stream.input_image);
      stream.header = rgbd->rgb.header;
      stream.request.start_async();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "d435i RGBD frame rejected: %s", error.what());
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

      const bool mode_allows_output = stream.target_point_publisher ?
        runs_d435i_inference(perception_mode_.load()) :
        runs_side_inference(perception_mode_.load());
      if (!mode_allows_output) {
        stream.busy.store(false);
        return;
      }

      msg::FruitTypes result;
      result.header = stream.header;
      result.types.reserve(detections.size());
      for (const auto & detection : detections) {
        result.types.push_back(detection.type);
      }
      stream.result_publisher->publish(result);

      if (stream.detections_publisher) {
        msg::FruitDetections all_detections;
        all_detections.header = stream.aligned_depth->header;
        const auto outputs = make_detection_outputs(
          detections, stream.transform, stream.aligned_depth->image,
          stream.depth_camera_info);
        all_detections.detections.reserve(outputs.size());
        for (const auto & output : outputs) {
          msg::FruitDetection item;
          item.type = output.type;
          item.confidence = output.confidence;
          item.roi.x_offset = static_cast<std::uint32_t>(output.roi.x);
          item.roi.y_offset = static_cast<std::uint32_t>(output.roi.y);
          item.roi.width = static_cast<std::uint32_t>(output.roi.width);
          item.roi.height = static_cast<std::uint32_t>(output.roi.height);
          item.has_point = output.has_point;
          if (output.has_point) {
            item.point.x = output.point.x;
            item.point.y = output.point.y;
            item.point.z = output.point.z;
          }
          all_detections.detections.push_back(std::move(item));
        }
        stream.detections_publisher->publish(all_detections);

        if (!all_detections.detections.empty() &&
          all_detections.detections.front().has_point)
        {
          geometry_msgs::msg::PointStamped target;
          target.header = stream.aligned_depth->header;
          target.point = all_detections.detections.front().point;
          stream.target_point_publisher->publish(target);
        }
      }

      const auto now = std::chrono::steady_clock::now();
      if (stream.last_completed_at.time_since_epoch().count() != 0) {
        const double interval =
          std::chrono::duration<double>(now - stream.last_completed_at).count();
        const double instantaneous_fps = interval > 0.0 ? 1.0 / interval : 0.0;
        stream.displayed_fps = stream.displayed_fps == 0.0 ? instantaneous_fps :
          0.8 * stream.displayed_fps + 0.2 * instantaneous_fps;
      }
      stream.last_completed_at = now;

      if (stream.image_publisher->get_subscription_count() > 0U) {
        cv::Mat annotated = stream.frame->image.clone();
        draw_annotations(annotated, detections, stream.transform, stream.displayed_fps);
        const auto annotated_message = cv_bridge::CvImage(
          stream.header, sensor_msgs::image_encodings::RGB8, annotated).toImageMsg();
        stream.image_publisher->publish(*annotated_message);
      }
    } catch (const std::exception & callback_error) {
      RCLCPP_ERROR(
        get_logger(), "%s inference failed: %s", stream.name.c_str(), callback_error.what());
    }
    stream.busy.store(false);
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
  ov::Shape output_shape_;
  float confidence_threshold_{0.50F};
  std::atomic<bool> stopping_{false};
  std::atomic<PerceptionMode> perception_mode_{PerceptionMode::kOff};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscription_;
  std::vector<std::unique_ptr<Stream>> streams_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
};

}  // namespace fruit_perception

RCLCPP_COMPONENTS_REGISTER_NODE(fruit_perception::FruitPerceptionNode)
