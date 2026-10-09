#include <cv_bridge/cv_bridge.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fruit_robviz_bridge
{
namespace
{
using Clock = std::chrono::steady_clock;
using Image = sensor_msgs::msg::Image;

struct StreamConfig
{
  double bandwidth_mbps{1.0};
  bool auto_quality{true};
  int quality{70};
  int width{640};
  int height{360};
  double max_fps{10.0};
};

struct StreamRuntime
{
  double tokens{0.0};
  double token_capacity{0.0};
  Clock::time_point last_refill{Clock::now()};
  Clock::time_point last_attempt{};
  int current_quality{70};
};

double finite_parameter(const rclcpp::Parameter & parameter)
{
  if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
    parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
  {
    throw std::invalid_argument(parameter.get_name() + " must be numeric");
  }
  const double value = parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ?
    parameter.as_double() : static_cast<double>(parameter.as_int());
  if (!std::isfinite(value)) {
    throw std::invalid_argument(parameter.get_name() + " must be finite");
  }
  return value;
}
}  // namespace

class ImageBridgeComponent : public rclcpp::Node
{
public:
  explicit ImageBridgeComponent(const rclcpp::NodeOptions & options)
  : Node("fruit_robviz_image_bridge", options)
  {
    camera_frame_ = declare_parameter("camera_frame_id", "camera_color_optical_frame");
    arm_frame_ = declare_parameter("arm_frame_id", "arm_base_link");
    annotated_topic_ = declare_parameter(
      "annotated_topic", "/fruit_perception/camera/annotated_image");
    depth_topic_ = declare_parameter("depth_topic", "/camera/depth/image_rect_raw");
    detections_topic_ = declare_parameter(
      "detections_topic", "/fruit_perception/camera/preview_detections");
    base_topic_ = declare_parameter(
      "base_detections_topic", "/fruit_perception/camera/base_preview_detections");
    declare_parameter("settings_version", 0);
    declare_parameter("settings_session", "");
    declare_stream_parameters("rgb", 1.5, 640, 360);
    declare_stream_parameters("depth", 0.8, 424, 240);
    rgb_config_ = read_stream_parameters("rgb");
    depth_config_ = read_stream_parameters("depth");
    initialize_runtime(rgb_runtime_, rgb_config_);
    initialize_runtime(depth_runtime_, depth_config_);
    parameter_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return on_parameters(parameters);
      });
    rgb_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      declare_parameter("rgb_output_topic", "/fruit_perception/camera/annotated_image/compressed"),
      rclcpp::SensorDataQoS().keep_last(1));
    depth_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      declare_parameter("depth_output_topic", "/fruit_perception/camera/native_depth/compressed"),
      rclcpp::SensorDataQoS().keep_last(1));
    demand_timer_ = create_wall_timer(
      std::chrono::milliseconds(20), [this]() {
        update_demand();
        process_pending_rgb();
      });
  }

private:
  void declare_stream_parameters(
    const std::string & prefix, double bandwidth, int width, int height)
  {
    declare_parameter(prefix + ".bandwidth_mbps", bandwidth);
    declare_parameter(prefix + ".auto_quality", true);
    declare_parameter(prefix + ".quality", 70);
    declare_parameter(prefix + ".width", width);
    declare_parameter(prefix + ".height", height);
    declare_parameter(prefix + ".max_fps", 10.0);
  }

  StreamConfig read_stream_parameters(const std::string & prefix) const
  {
    StreamConfig config;
    config.bandwidth_mbps = get_parameter(prefix + ".bandwidth_mbps").as_double();
    config.auto_quality = get_parameter(prefix + ".auto_quality").as_bool();
    config.quality = static_cast<int>(get_parameter(prefix + ".quality").as_int());
    config.width = static_cast<int>(get_parameter(prefix + ".width").as_int());
    config.height = static_cast<int>(get_parameter(prefix + ".height").as_int());
    config.max_fps = get_parameter(prefix + ".max_fps").as_double();
    validate_stream_config(prefix, config);
    return config;
  }

  static void validate_stream_config(const std::string & prefix, const StreamConfig & config)
  {
    if (!std::isfinite(config.bandwidth_mbps) || config.bandwidth_mbps <= 0.0 ||
      config.bandwidth_mbps > 1000.0)
    {
      throw std::invalid_argument(prefix + ".bandwidth_mbps must be positive");
    }
    if (config.quality < 30 || config.quality > 95) {
      throw std::invalid_argument(prefix + ".quality must be in [30, 95]");
    }
    if (config.width < 64 || config.width > 3840 ||
      config.height < 48 || config.height > 2160)
    {
      throw std::invalid_argument(prefix + " dimensions are outside the supported range");
    }
    if (!std::isfinite(config.max_fps) || config.max_fps <= 0.0 || config.max_fps > 60.0) {
      throw std::invalid_argument(prefix + ".max_fps must be in (0, 60]");
    }
  }

  static void initialize_runtime(StreamRuntime & runtime, const StreamConfig & config)
  {
    runtime.token_capacity = config.bandwidth_mbps * 1000000.0 / 8.0;
    runtime.tokens = runtime.token_capacity;
    runtime.last_refill = Clock::now();
    runtime.current_quality = 70;
  }

  static void apply_parameters(
    const std::vector<rclcpp::Parameter> & parameters, const std::string & prefix,
    StreamConfig & config)
  {
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == prefix + ".bandwidth_mbps") {
        config.bandwidth_mbps = finite_parameter(parameter);
      } else if (parameter.get_name() == prefix + ".auto_quality") {
        config.auto_quality = parameter.as_bool();
      } else if (parameter.get_name() == prefix + ".quality") {
        const auto value = parameter.as_int();
        if (value < 30 || value > 95) {
          throw std::invalid_argument("JPEG quality must be in [30, 95]");
        }
        config.quality = static_cast<int>(value);
      } else if (parameter.get_name() == prefix + ".width") {
        const auto value = parameter.as_int();
        if (value < 64 || value > 3840) {
          throw std::invalid_argument("width must be in [64, 3840]");
        }
        config.width = static_cast<int>(value);
      } else if (parameter.get_name() == prefix + ".height") {
        const auto value = parameter.as_int();
        if (value < 48 || value > 2160) {
          throw std::invalid_argument("height must be in [48, 2160]");
        }
        config.height = static_cast<int>(value);
      } else if (parameter.get_name() == prefix + ".max_fps") {
        config.max_fps = finite_parameter(parameter);
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    try {
      std::lock_guard<std::mutex> lock(state_mutex_);
      auto rgb = rgb_config_;
      auto depth = depth_config_;
      std::int64_t version = 0;
      std::string session;
      bool image_settings = false;
      for (const auto & parameter : parameters) {
        if (parameter.get_name().rfind("rgb.", 0) == 0 ||
          parameter.get_name().rfind("depth.", 0) == 0 ||
          parameter.get_name().rfind("settings_", 0) == 0)
        {
          image_settings = true;
        }
        if (parameter.get_name() == "settings_version") {
          version = parameter.as_int();
        } else if (parameter.get_name() == "settings_session") {
          session = parameter.as_string();
        }
      }
      if (!image_settings) {
        result.successful = true;
        return result;
      }
      if (version <= 0 || session.size() != 32U) {
        throw std::invalid_argument("a complete settings version/session is required");
      }
      if (session == applied_session_ && version < applied_version_) {
        throw std::invalid_argument("stale settings version");
      }
      apply_parameters(parameters, "rgb", rgb);
      apply_parameters(parameters, "depth", depth);
      validate_stream_config("rgb", rgb);
      validate_stream_config("depth", depth);
      if (rgb.bandwidth_mbps != rgb_config_.bandwidth_mbps) {
        initialize_runtime(rgb_runtime_, rgb);
      }
      if (depth.bandwidth_mbps != depth_config_.bandwidth_mbps) {
        initialize_runtime(depth_runtime_, depth);
      }
      rgb_config_ = rgb;
      depth_config_ = depth;
      applied_version_ = version;
      applied_session_ = session;
      settings_applied_ = true;
      result.successful = true;
    } catch (const std::exception & error) {
      result.successful = false;
      result.reason = error.what();
    }
    return result;
  }

  template<typename Publisher>
  static bool has_demand(const Publisher & publisher)
  {
    return publisher->get_subscription_count() +
           publisher->get_intra_process_subscription_count() > 0U;
  }

  void update_demand()
  {
    bool applied;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      applied = settings_applied_;
    }
    if (applied && has_demand(rgb_publisher_)) {
      if (!annotated_subscription_) {
        annotated_subscription_ = create_subscription<Image>(
          annotated_topic_, rclcpp::SensorDataQoS().keep_last(1),
          [this](Image::ConstSharedPtr message) {
            if (!admit_frame(true)) {
              return;
            }
            pending_rgb_ = message;
            process_pending_rgb();
          });
      }
    } else {
      annotated_subscription_.reset();
      pending_rgb_.reset();
    }
    if (applied && has_demand(depth_publisher_)) {
      if (!depth_subscription_) {
        depth_subscription_ = create_subscription<Image>(
          depth_topic_, rclcpp::SensorDataQoS().keep_last(1),
          [this](Image::ConstSharedPtr message) {
            if (admit_frame(false)) {
              on_depth_image(message);
            }
          });
      }
    } else {
      depth_subscription_.reset();
    }
  }

  bool admit_frame(bool rgb)
  {
    if (!has_demand(rgb ? rgb_publisher_ : depth_publisher_)) {
      return false;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!settings_applied_) {
      return false;
    }
    auto & runtime = rgb ? rgb_runtime_ : depth_runtime_;
    const auto & config = rgb ? rgb_config_ : depth_config_;
    const auto now = Clock::now();
    if (runtime.last_attempt != Clock::time_point{} &&
      std::chrono::duration<double>(now - runtime.last_attempt).count() < 1.0 / config.max_fps)
    {
      return false;
    }
    runtime.last_attempt = now;
    return true;
  }

  void process_pending_rgb()
  {
    if (!pending_rgb_ || !has_demand(rgb_publisher_)) {
      return;
    }
    const auto message = std::move(pending_rgb_);
    try {
      const auto image = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::RGB8);
      StreamConfig config;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        config = rgb_config_;
      }
      cv::Mat annotated;
      const cv::Size output_size(config.width, config.height);
      cv::resize(image->image, annotated, output_size, 0, 0, cv::INTER_AREA);
      cv::Mat bgr;
      cv::cvtColor(annotated, bgr, cv::COLOR_RGB2BGR);
      encode_and_publish(true, bgr, message->header);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "RGB preview rejected: %s", error.what());
    }
  }

  void on_depth_image(const Image::ConstSharedPtr & message)
  {
    if (message->encoding != sensor_msgs::image_encodings::TYPE_16UC1 ||
      message->width == 0 || message->height == 0 ||
      message->step < static_cast<std::size_t>(message->width) * sizeof(std::uint16_t) ||
      message->data.size() < static_cast<std::size_t>(message->step) * message->height)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Native depth frame has invalid 16UC1 encoding, step or data length");
      return;
    }
    try {
      const auto image = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::TYPE_16UC1);
      StreamConfig config;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        config = depth_config_;
      }
      cv::Mat sampled;
      const cv::Size output_size(config.width, config.height);
      cv::resize(image->image, sampled, output_size, 0, 0, cv::INTER_NEAREST);
      cv::Mat valid_mask = sampled > 0;
      cv::Mat colored = cv::Mat::zeros(sampled.size(), CV_8UC3);
      double minimum = 0;
      double maximum = 0;
      const auto valid_count = cv::countNonZero(valid_mask);
      if (valid_count > 0) {
        cv::minMaxLoc(sampled, &minimum, &maximum, nullptr, nullptr, valid_mask);
        cv::Mat normalized = cv::Mat::zeros(sampled.size(), CV_8UC1);
        if (minimum == maximum) {
          normalized.setTo(cv::Scalar(128), valid_mask);
        } else {
          sampled.convertTo(
            normalized, CV_8UC1, 254.0 / (maximum - minimum),
            1.0 - 254.0 * minimum / (maximum - minimum));
          normalized.setTo(cv::Scalar(0), ~valid_mask);
        }
        cv::Mat mapped;
        cv::applyColorMap(normalized, mapped, cv::COLORMAP_JET);
        mapped.copyTo(colored, valid_mask);
      }
      std::ostringstream banner;
      banner << "Depth (" << config.width << "x" << config.height << ") | ";
      if (valid_count == 0) {
        banner << "No valid depth";
      } else {
        banner << std::fixed << std::setprecision(2) << minimum * 0.001 << "~"
               << maximum * 0.001 << "m | " << std::setprecision(0)
               << 100.0 * valid_count / sampled.total() << "%";
      }
      cv::rectangle(colored, cv::Rect(0, 0, colored.cols, 22), cv::Scalar(20, 20, 20), cv::FILLED);
      cv::putText(
        colored, banner.str(), cv::Point(5, 16), cv::FONT_HERSHEY_SIMPLEX, 0.38,
        cv::Scalar(0, 255, 120), 1, cv::LINE_AA);
      encode_and_publish(false, colored, message->header);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Depth preview rejected: %s", error.what());
    }
  }

  void encode_and_publish(bool rgb, const cv::Mat & image, const std_msgs::msg::Header & header)
  {
    const auto publisher = rgb ? rgb_publisher_ : depth_publisher_;
    if (!has_demand(publisher)) {
      return;
    }
    // Parameters can change concurrently. Charge and publish against one config snapshot.
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto & config = rgb ? rgb_config_ : depth_config_;
    auto & runtime = rgb ? rgb_runtime_ : depth_runtime_;
    const auto now = Clock::now();
    const double elapsed = std::chrono::duration<double>(now - runtime.last_refill).count();
    runtime.tokens = std::min(
      runtime.token_capacity,
      runtime.tokens + elapsed * config.bandwidth_mbps * 1000000.0 / 8.0);
    runtime.last_refill = now;
    std::vector<std::uint8_t> encoded;
    const int first_quality = config.auto_quality ?
      std::clamp(runtime.current_quality, 30, 70) : config.quality;
    for (int quality = first_quality; quality >= 30; quality -= 5) {
      if (!cv::imencode(".jpg", image, encoded, {cv::IMWRITE_JPEG_QUALITY, quality})) {
        throw std::runtime_error("JPEG encoding failed");
      }
      if (static_cast<double>(encoded.size()) <= runtime.tokens) {
        runtime.tokens -= static_cast<double>(encoded.size());
        runtime.current_quality = config.auto_quality ? std::min(70, quality + 5) : config.quality;
        sensor_msgs::msg::CompressedImage output;
        output.header = header;
        output.format = "jpeg";
        output.data = std::move(encoded);
        publisher->publish(output);
        return;
      }
      if (!config.auto_quality) {
        break;
      }
    }
    if (config.auto_quality) {
      runtime.current_quality = 30;
    }
  }

  std::mutex state_mutex_;
  bool settings_applied_{false};
  std::int64_t applied_version_{0};
  std::string applied_session_;
  StreamConfig rgb_config_, depth_config_;
  StreamRuntime rgb_runtime_, depth_runtime_;
  std::string camera_frame_, arm_frame_, annotated_topic_, depth_topic_, detections_topic_,
    base_topic_;
  Image::ConstSharedPtr pending_rgb_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_publisher_, depth_publisher_;
  rclcpp::Subscription<Image>::SharedPtr annotated_subscription_, depth_subscription_;
  rclcpp::TimerBase::SharedPtr demand_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};
}  // namespace fruit_robviz_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(fruit_robviz_bridge::ImageBridgeComponent)
