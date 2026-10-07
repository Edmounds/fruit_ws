#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "fruit_arm_interfaces/msg/arm_telemetry.hpp"
#include "fruit_hardware_interfaces/msg/arm_status.hpp"
#include "fruit_perception/msg/fruit_detections.hpp"

namespace fruit_robviz_bridge
{
namespace
{
using Clock = std::chrono::steady_clock;
constexpr double kPi = 3.14159265358979323846;

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
  Clock::time_point last_refill{};
  Clock::time_point last_attempt{};
  int current_quality{70};
};

struct Extrinsic
{
  std::array<std::array<double, 3>, 3> rotation{};
  std::array<double, 3> translation_mm{};
};

using JointLimits = std::array<std::pair<double, double>, 4>;

bool finite(double value)
{
  return std::isfinite(value);
}

double finite_parameter(const rclcpp::Parameter & parameter)
{
  if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
    parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
  {
    throw std::invalid_argument(parameter.get_name() + " must be numeric");
  }
  const double value = parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ?
    parameter.as_double() : static_cast<double>(parameter.as_int());
  if (!finite(value)) {
    throw std::invalid_argument(parameter.get_name() + " must be finite");
  }
  return value;
}

std::vector<double> read_sequence(const cv::FileNode & node, const std::string & name)
{
  if (node.empty() || !node.isSeq() || node.size() != 3U) {
    throw std::runtime_error(name + " must contain three numbers");
  }
  std::vector<double> values;
  values.reserve(3U);
  for (const auto & item : node) {
    const double value = static_cast<double>(item);
    if (!finite(value)) {
      throw std::runtime_error(name + " contains a non-finite value");
    }
    values.push_back(value);
  }
  return values;
}

Extrinsic load_extrinsic(const std::string & path)
{
  if (path.empty() || !std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("camera_extrinsic_config is not a file: " + path);
  }
  cv::FileStorage storage(path, cv::FileStorage::READ);
  if (!storage.isOpened()) {
    throw std::runtime_error("Cannot open camera extrinsic config: " + path);
  }
  const auto end_to_screw = read_sequence(
    storage["translation_end_to_screw_mm"], "translation_end_to_screw_mm");
  const auto screw_to_rgb = read_sequence(
    storage["translation_screw_to_rgb_mm"], "translation_screw_to_rgb_mm");
  const auto rotation_node = storage["rotation_end_from_rgb"];
  if (rotation_node.empty() || !rotation_node.isSeq() || rotation_node.size() != 3U) {
    throw std::runtime_error("rotation_end_from_rgb must be a 3x3 matrix");
  }

  Extrinsic result;
  for (std::size_t row = 0; row < 3U; ++row) {
    const auto row_values = read_sequence(rotation_node[static_cast<int>(row)], "rotation row");
    for (std::size_t column = 0; column < 3U; ++column) {
      result.rotation[row][column] = row_values[column];
    }
  }
  for (std::size_t index = 0; index < 3U; ++index) {
    result.translation_mm[index] = end_to_screw[index] + screw_to_rgb[index];
  }

  for (std::size_t row = 0; row < 3U; ++row) {
    for (std::size_t other = 0; other < 3U; ++other) {
      double dot = 0.0;
      for (std::size_t column = 0; column < 3U; ++column) {
        dot += result.rotation[row][column] * result.rotation[other][column];
      }
      const double expected = row == other ? 1.0 : 0.0;
      if (std::abs(dot - expected) > 1e-3) {
        throw std::runtime_error("camera rotation is not orthonormal");
      }
    }
  }
  const auto & a = result.rotation[0];
  const auto & b = result.rotation[1];
  const auto & c = result.rotation[2];
  const double determinant =
    a[0] * (b[1] * c[2] - b[2] * c[1]) -
    a[1] * (b[0] * c[2] - b[2] * c[0]) +
    a[2] * (b[0] * c[1] - b[1] * c[0]);
  if (std::abs(determinant - 1.0) > 1e-3) {
    throw std::runtime_error("camera rotation must have determinant +1");
  }
  return result;
}

JointLimits load_joint_limits(const std::string & path)
{
  if (path.empty() || !std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("arm_presets_config is not a file: " + path);
  }
  cv::FileStorage storage(path, cv::FileStorage::READ);
  if (!storage.isOpened()) {
    throw std::runtime_error("Cannot open arm presets config: " + path);
  }
  const auto mapping = storage["arm_planner_node"]["ros__parameters"]["joint_mapping"];
  if (mapping.empty()) {
    throw std::runtime_error("joint_mapping is missing from arm presets config: " + path);
  }
  const std::array<std::string, 4> names{"q1", "q2", "q3", "q4"};
  JointLimits limits{};
  for (std::size_t index = 0; index < names.size(); ++index) {
    const auto values = mapping[names[index]]["limits_deg"];
    if (values.empty() || !values.isSeq() || values.size() != 2U) {
      throw std::runtime_error(names[index] + ".limits_deg must contain two numbers");
    }
    const double minimum = static_cast<double>(values[0]);
    const double maximum = static_cast<double>(values[1]);
    if (!finite(minimum) || !finite(maximum) || minimum >= maximum) {
      throw std::runtime_error(names[index] + ".limits_deg is invalid");
    }
    limits[index] = {minimum * kPi / 180.0, maximum * kPi / 180.0};
  }
  return limits;
}

bool same_stamp(
  const builtin_interfaces::msg::Time & left,
  const builtin_interfaces::msg::Time & right)
{
  return left.sec == right.sec && left.nanosec == right.nanosec;
}

std::int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

bool fresh_for_image(
  const builtin_interfaces::msg::Time & sample_stamp,
  const builtin_interfaces::msg::Time & image_stamp,
  double max_age_sec)
{
  const auto sample_ns = stamp_nanoseconds(sample_stamp);
  const auto image_ns = stamp_nanoseconds(image_stamp);
  if (sample_ns <= 0 || image_ns <= 0 || sample_ns > image_ns) {
    return false;
  }
  return static_cast<double>(image_ns - sample_ns) <= max_age_sec * 1e9;
}

std::string point_text(const geometry_msgs::msg::Point & point)
{
  std::ostringstream stream;
  stream << std::fixed;
  stream.precision(2);
  stream << "(" << point.x << "," << point.y << "," << point.z << ")";
  return stream.str();
}

}  // namespace

class ImageBridgeComponent : public rclcpp::Node
{
public:
  explicit ImageBridgeComponent(const rclcpp::NodeOptions & options)
  : Node("fruit_robviz_image_bridge", options),
    rgb_runtime_(default_runtime()), depth_runtime_(default_runtime())
  {
    const auto extrinsic_path = declare_parameter(
      "camera_extrinsic_config",
      "/home/rc4/fruit_ws/src/fruit_mission/config/camera_extrinsic.yaml");
    const auto arm_presets_path = declare_parameter(
      "arm_presets_config",
      "/home/rc4/fruit_ws/src/fruit_arm_planner/config/arm_presets.yaml");
    extrinsic_ = load_extrinsic(extrinsic_path);
    joint_limits_ = load_joint_limits(arm_presets_path);
    camera_frame_id_ = declare_parameter("camera_frame_id", "camera_color_optical_frame");
    arm_frame_id_ = declare_parameter("arm_frame_id", "arm_base_link");
    base_linear_speed_threshold_mps_ = declare_parameter(
      "base_linear_speed_threshold_mps", 0.03);
    base_angular_speed_threshold_rps_ = declare_parameter(
      "base_angular_speed_threshold_rps", 0.08);
    arm_joint_speed_threshold_rps_ = declare_parameter(
      "arm_joint_speed_threshold_rps", 0.05);
    status_max_age_sec_ = declare_parameter("status_max_age_sec", 0.25);
    odom_frame_ = declare_parameter("odom_frame", "odom");
    if (camera_frame_id_.empty() || arm_frame_id_.empty() || odom_frame_.empty()) {
      throw std::invalid_argument("camera, arm and odom frame IDs must be nonempty");
    }
    if (!finite(base_linear_speed_threshold_mps_) || base_linear_speed_threshold_mps_ < 0.0 ||
      !finite(base_angular_speed_threshold_rps_) || base_angular_speed_threshold_rps_ < 0.0 ||
      !finite(arm_joint_speed_threshold_rps_) || arm_joint_speed_threshold_rps_ < 0.0 ||
      !finite(status_max_age_sec_) || status_max_age_sec_ <= 0.0)
    {
      throw std::invalid_argument("standstill thresholds and status age must be valid");
    }

    declare_stream_parameters("rgb", 1.5, 640, 360);
    declare_stream_parameters("depth", 0.8, 424, 240);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      rgb_config_ = read_stream_parameters("rgb");
      depth_config_ = read_stream_parameters("depth");
      initialize_runtime(rgb_runtime_, rgb_config_);
      initialize_runtime(depth_runtime_, depth_config_);
    }
    parameter_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return on_parameters(parameters);
      });

    annotated_topic_ = declare_parameter(
      "annotated_topic", "/fruit_perception/camera/annotated_image");
    depth_topic_ = declare_parameter(
      "depth_topic", "/camera/depth/image_rect_raw");
    detections_topic_ = declare_parameter(
      "detections_topic", "/fruit_perception/camera/preview_detections");
    rgb_output_topic_ = declare_parameter(
      "rgb_output_topic", "/fruit_perception/camera/annotated_image/compressed");
    depth_output_topic_ = declare_parameter(
      "depth_output_topic", "/fruit_perception/camera/native_depth/compressed");
    arm_telemetry_topic_ = declare_parameter(
      "arm_telemetry_topic", "/arm_planner_node/joint_states");
    arm_status_topic_ = declare_parameter(
      "arm_status_topic", "/fruit_hardware_bridge/arm/status");
    odometry_topic_ = declare_parameter("odometry_topic", "/odometry");

    rgb_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      rgb_output_topic_, rclcpp::SensorDataQoS().keep_last(1));
    depth_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      depth_output_topic_, rclcpp::SensorDataQoS().keep_last(1));

    annotated_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      annotated_topic_, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        on_annotated_image(message);
      });
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        on_depth_image(message);
      });
    detections_subscription_ = create_subscription<fruit_perception::msg::FruitDetections>(
      detections_topic_, rclcpp::SensorDataQoS().keep_last(1),
      [this](fruit_perception::msg::FruitDetections::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_detections_ = message;
      });
    arm_telemetry_subscription_ = create_subscription<fruit_arm_interfaces::msg::ArmTelemetry>(
      arm_telemetry_topic_, rclcpp::QoS(1).reliable(),
      [this](fruit_arm_interfaces::msg::ArmTelemetry::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_arm_telemetry_ = message;
        last_arm_telemetry_received_ = Clock::now();
      });
    arm_status_subscription_ = create_subscription<fruit_hardware_interfaces::msg::ArmStatus>(
      arm_status_topic_, rclcpp::QoS(1).reliable(),
      [this](fruit_hardware_interfaces::msg::ArmStatus::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_arm_status_ = message;
        last_arm_status_received_ = Clock::now();
      });
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::SensorDataQoS().keep_last(1),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_odometry_ = message;
      });
  }

private:
  static StreamRuntime default_runtime()
  {
    StreamRuntime runtime;
    runtime.last_refill = Clock::now();
    runtime.last_attempt = Clock::time_point::min();
    return runtime;
  }

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
    if (!finite(config.bandwidth_mbps) || config.bandwidth_mbps <= 0.0 ||
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
    if (!finite(config.max_fps) || config.max_fps <= 0.0 || config.max_fps > 60.0) {
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

  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    try {
      std::lock_guard<std::mutex> lock(state_mutex_);
      StreamConfig rgb = rgb_config_;
      StreamConfig depth = depth_config_;
      apply_parameters(parameters, "rgb", rgb);
      apply_parameters(parameters, "depth", depth);
      validate_stream_config("rgb", rgb);
      validate_stream_config("depth", depth);
      const bool rgb_changed = rgb.bandwidth_mbps != rgb_config_.bandwidth_mbps;
      const bool depth_changed = depth.bandwidth_mbps != depth_config_.bandwidth_mbps;
      rgb_config_ = rgb;
      depth_config_ = depth;
      if (rgb_changed) {
        initialize_runtime(rgb_runtime_, rgb_config_);
      }
      if (depth_changed) {
        initialize_runtime(depth_runtime_, depth_config_);
      }
    } catch (const std::exception & error) {
      result.successful = false;
      result.reason = error.what();
    }
    return result;
  }

  static void apply_parameters(
    const std::vector<rclcpp::Parameter> & parameters,
    const std::string & prefix,
    StreamConfig & config)
  {
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == prefix + ".bandwidth_mbps") {
        config.bandwidth_mbps = finite_parameter(parameter);
      } else if (parameter.get_name() == prefix + ".auto_quality") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          throw std::invalid_argument(parameter.get_name() + " must be boolean");
        }
        config.auto_quality = parameter.as_bool();
      } else if (parameter.get_name() == prefix + ".quality") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
          throw std::invalid_argument(parameter.get_name() + " must be integer");
        }
        config.quality = static_cast<int>(parameter.as_int());
      } else if (parameter.get_name() == prefix + ".width") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
          throw std::invalid_argument(parameter.get_name() + " must be integer");
        }
        config.width = static_cast<int>(parameter.as_int());
      } else if (parameter.get_name() == prefix + ".height") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
          throw std::invalid_argument(parameter.get_name() + " must be integer");
        }
        config.height = static_cast<int>(parameter.as_int());
      } else if (parameter.get_name() == prefix + ".max_fps") {
        config.max_fps = finite_parameter(parameter);
      }
    }
  }

  void on_annotated_image(const sensor_msgs::msg::Image::ConstSharedPtr & message)
  {
    if (rgb_publisher_->get_subscription_count() == 0U) {
      return;
    }
    fruit_perception::msg::FruitDetections::ConstSharedPtr detections;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (latest_detections_ &&
        same_stamp(latest_detections_->header.stamp, message->header.stamp) &&
        latest_detections_->header.frame_id == message->header.frame_id &&
        message->header.frame_id == camera_frame_id_)
      {
        detections = latest_detections_;
      }
    }
    try {
      const auto image = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::RGB8);
      if (image->image.empty()) {
        return;
      }
      StreamConfig config;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        config = rgb_config_;
      }
      cv::Mat annotated;
      cv::resize(
        image->image, annotated, cv::Size(config.width, config.height), 0.0, 0.0,
        cv::INTER_AREA);
      if (detections) {
        draw_detections(annotated, *detections, image->image.size());
      }
      cv::Mat bgr;
      cv::cvtColor(annotated, bgr, cv::COLOR_RGB2BGR);
      encode_and_publish(true, bgr, message->header);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "RGB preview rejected: %s", error.what());
    }
  }

  void draw_detections(
    cv::Mat & image,
    const fruit_perception::msg::FruitDetections & detections,
    const cv::Size & source_size)
  {
    const double scale_x = static_cast<double>(image.cols) / source_size.width;
    const double scale_y = static_cast<double>(image.rows) / source_size.height;
    for (const auto & detection : detections.detections) {
      const int x = std::clamp(
        static_cast<int>(detection.roi.x_offset * scale_x), 0, image.cols - 1);
      const int y = std::clamp(
        static_cast<int>(detection.roi.y_offset * scale_y), 0, image.rows - 1);
      const int right = std::clamp(
        static_cast<int>((detection.roi.x_offset + detection.roi.width) * scale_x),
        x + 1, image.cols - 1);
      const int bottom = std::clamp(
        static_cast<int>((detection.roi.y_offset + detection.roi.height) * scale_y),
        y + 1, image.rows - 1);
      cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Point(right, bottom)), cv::Scalar(0, 255, 0), 2);

      std::ostringstream label;
      label << detection.type << " " << std::fixed << std::setprecision(0)
            << detection.confidence * 100.0F << "%";
      if (detection.has_point && finite(detection.point.x) && finite(detection.point.y) &&
        finite(detection.point.z) && detection.point.z > 0.0)
      {
        label << " D=" << std::setprecision(2) << detection.point.z
              << " Cam=" << point_text(detection.point);
        const auto base = base_point_for(
          detection.point, detections.header.stamp, detections.header.frame_id);
        if (base) {
          label << " Base=" << point_text(*base);
        } else {
          label << " Base=N/A";
        }
      } else {
        label << " D=N/A Cam=N/A Base=N/A";
      }
      const int baseline_y = std::max(14, y - 5);
      cv::putText(
        image, label.str(), cv::Point(x, baseline_y), cv::FONT_HERSHEY_SIMPLEX,
        0.35, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
  }

  void on_depth_image(const sensor_msgs::msg::Image::ConstSharedPtr & message)
  {
    if (depth_publisher_->get_subscription_count() == 0U) {
      return;
    }
    if (message->encoding != sensor_msgs::image_encodings::TYPE_16UC1 ||
      message->step < message->width * sizeof(std::uint16_t) ||
      message->data.size() < message->step * message->height)
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
      cv::resize(
        image->image, sampled, cv::Size(config.width, config.height), 0.0, 0.0,
        cv::INTER_NEAREST);
      cv::Mat valid_mask = sampled > 0;
      cv::Mat colored = cv::Mat::zeros(sampled.size(), CV_8UC3);
      double min_depth = 0.0;
      double max_depth = 0.0;
      const auto valid_count = cv::countNonZero(valid_mask);
      if (valid_count > 0) {
        double min_value = 0.0;
        double max_value = 0.0;
        cv::minMaxLoc(sampled, &min_value, &max_value, nullptr, nullptr, valid_mask);
        min_depth = min_value * 0.001;
        max_depth = max_value * 0.001;
        cv::Mat normalized = cv::Mat::zeros(sampled.size(), CV_8UC1);
        if (max_value == min_value) {
          normalized.setTo(cv::Scalar(128), valid_mask);
        } else {
          sampled.convertTo(
            normalized, CV_8UC1, 254.0 / (max_value - min_value),
            1.0 - 254.0 * min_value / (max_value - min_value));
          normalized.setTo(cv::Scalar(0), ~valid_mask);
        }
        cv::Mat mapped;
        cv::applyColorMap(normalized, mapped, cv::COLORMAP_JET);
        mapped.copyTo(colored, valid_mask);
      }
      const double coverage = 100.0 * static_cast<double>(valid_count) /
        static_cast<double>(sampled.total());
      std::ostringstream banner;
      banner << "Depth (" << config.width << "x" << config.height << ") | ";
      if (valid_count == 0) {
        banner << "No valid depth";
      } else {
        banner << std::fixed << std::setprecision(2) << min_depth << "~" << max_depth
               << "m | " << std::setprecision(0) << coverage << "%";
      }
      cv::rectangle(colored, cv::Rect(0, 0, colored.cols, 22), cv::Scalar(20, 20, 20), cv::FILLED);
      cv::putText(
        colored, banner.str(), cv::Point(5, 16), cv::FONT_HERSHEY_SIMPLEX, 0.38,
        cv::Scalar(0, 255, 120), 1, cv::LINE_AA);
      encode_and_publish(false, colored, message->header);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Depth preview rejected: %s", error.what());
    }
  }

  std::optional<geometry_msgs::msg::Point> base_point_for(
    const geometry_msgs::msg::Point & camera_point,
    const builtin_interfaces::msg::Time & image_stamp,
    const std::string & image_frame_id)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!latest_arm_telemetry_ || !latest_arm_status_ || !latest_odometry_) {
      return std::nullopt;
    }
    if (image_frame_id != camera_frame_id_ ||
      latest_arm_telemetry_->header.frame_id != arm_frame_id_ ||
      std::chrono::duration<double>(Clock::now() - last_arm_telemetry_received_).count() >
      status_max_age_sec_ ||
      std::chrono::duration<double>(Clock::now() - last_arm_status_received_).count() >
      status_max_age_sec_ ||
      !fresh_for_image(latest_arm_telemetry_->header.stamp, image_stamp, status_max_age_sec_) ||
      !fresh_for_image(latest_odometry_->header.stamp, image_stamp, status_max_age_sec_) ||
      latest_odometry_->header.frame_id != odom_frame_)
    {
      return std::nullopt;
    }
    if ((latest_arm_status_->task_status !=
      fruit_hardware_interfaces::msg::ArmStatus::STATUS_IDLE &&
      latest_arm_status_->task_status !=
      fruit_hardware_interfaces::msg::ArmStatus::STATUS_COMPLETED) ||
      latest_arm_status_->error_code !=
      fruit_hardware_interfaces::msg::ArmStatus::ERROR_NONE ||
      (latest_arm_telemetry_->task_status !=
      fruit_hardware_interfaces::msg::ArmStatus::STATUS_IDLE &&
      latest_arm_telemetry_->task_status !=
      fruit_hardware_interfaces::msg::ArmStatus::STATUS_COMPLETED) ||
      latest_arm_telemetry_->error_code !=
      fruit_hardware_interfaces::msg::ArmStatus::ERROR_NONE)
    {
      return std::nullopt;
    }
    const auto & hardware = latest_arm_telemetry_->hardware_joint_state;
    const auto & model = latest_arm_telemetry_->model_joint_state;
    const std::array<std::string, 4> joint_names{"joint_1", "joint_2", "joint_3", "joint_4"};
    if (hardware.name.size() != 4U || hardware.name != std::vector<std::string>(joint_names.begin(), joint_names.end()) ||
      model.name.size() != 4U || model.name != std::vector<std::string>(joint_names.begin(), joint_names.end()) ||
      hardware.position.size() != 4U || hardware.velocity.size() != 4U ||
      model.position.size() != 4U || model.velocity.size() != 4U ||
      !finite(latest_arm_telemetry_->tcp_position.x) ||
      !finite(latest_arm_telemetry_->tcp_position.y) ||
      !finite(latest_arm_telemetry_->tcp_position.z) ||
      !finite(latest_arm_telemetry_->tcp_pitch_deg))
    {
      return std::nullopt;
    }
    for (std::size_t index = 0; index < 4U; ++index) {
      if (!finite(hardware.position[index]) ||
        hardware.position[index] < joint_limits_[index].first ||
        hardware.position[index] > joint_limits_[index].second ||
        !finite(hardware.velocity[index]) ||
        std::abs(hardware.velocity[index]) > arm_joint_speed_threshold_rps_ ||
        !finite(model.position[index]) || !finite(model.velocity[index]) ||
        std::abs(model.velocity[index]) > arm_joint_speed_threshold_rps_)
      {
        return std::nullopt;
      }
    }
    if (!finite(latest_arm_status_->joint_1_velocity) ||
      !finite(latest_arm_status_->joint_2_velocity) ||
      !finite(latest_arm_status_->joint_3_velocity) ||
      !finite(latest_arm_status_->joint_4_velocity) ||
      std::abs(latest_arm_status_->joint_1_velocity) > arm_joint_speed_threshold_rps_ ||
      std::abs(latest_arm_status_->joint_2_velocity) > arm_joint_speed_threshold_rps_ ||
      std::abs(latest_arm_status_->joint_3_velocity) > arm_joint_speed_threshold_rps_ ||
      std::abs(latest_arm_status_->joint_4_velocity) > arm_joint_speed_threshold_rps_)
    {
      return std::nullopt;
    }
    const std::array<float, 4> status_positions{
      latest_arm_status_->joint_1, latest_arm_status_->joint_2,
      latest_arm_status_->joint_3, latest_arm_status_->joint_4};
    for (std::size_t index = 0; index < status_positions.size(); ++index) {
      if (!finite(status_positions[index]) ||
        status_positions[index] < joint_limits_[index].first ||
        status_positions[index] > joint_limits_[index].second)
      {
        return std::nullopt;
      }
    }
    const auto & twist = latest_odometry_->twist.twist;
    const double linear_speed = std::sqrt(
      twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y +
      twist.linear.z * twist.linear.z);
    const double angular_speed = std::sqrt(
      twist.angular.x * twist.angular.x + twist.angular.y * twist.angular.y +
      twist.angular.z * twist.angular.z);
    if (!finite(linear_speed) || !finite(angular_speed) ||
      linear_speed > base_linear_speed_threshold_mps_ ||
      angular_speed > base_angular_speed_threshold_rps_)
    {
      return std::nullopt;
    }

    const std::array<double, 3> camera_mm{
      camera_point.x * 1000.0, camera_point.y * 1000.0, camera_point.z * 1000.0};
    std::array<double, 3> end_mm{};
    for (std::size_t row = 0; row < 3U; ++row) {
      end_mm[row] = extrinsic_.translation_mm[row];
      for (std::size_t column = 0; column < 3U; ++column) {
        end_mm[row] += extrinsic_.rotation[row][column] * camera_mm[column];
      }
    }
    const double yaw = model.position[0];
    const double pitch = latest_arm_telemetry_->tcp_pitch_deg * kPi / 180.0;
    const double cosine_yaw = std::cos(yaw);
    const double sine_yaw = std::sin(yaw);
    const double cosine_pitch = std::cos(pitch);
    const double sine_pitch = std::sin(pitch);
    const double radial = cosine_pitch * end_mm[0] - sine_pitch * end_mm[2];
    geometry_msgs::msg::Point base;
    base.x = latest_arm_telemetry_->tcp_position.x * 1000.0 +
      cosine_yaw * radial - sine_yaw * end_mm[1];
    base.y = latest_arm_telemetry_->tcp_position.y * 1000.0 +
      sine_yaw * radial + cosine_yaw * end_mm[1];
    base.z = latest_arm_telemetry_->tcp_position.z * 1000.0 +
      sine_pitch * end_mm[0] + cosine_pitch * end_mm[2];
    base.x *= 0.001;
    base.y *= 0.001;
    base.z *= 0.001;
    if (!finite(base.x) || !finite(base.y) || !finite(base.z)) {
      return std::nullopt;
    }
    return base;
  }

  void encode_and_publish(bool rgb, const cv::Mat & image, const std_msgs::msg::Header & header)
  {
    auto publisher = rgb ? rgb_publisher_ : depth_publisher_;
    if (publisher->get_subscription_count() == 0U) {
      return;
    }
    StreamConfig config;
    StreamRuntime * runtime = nullptr;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      config = rgb ? rgb_config_ : depth_config_;
      runtime = rgb ? &rgb_runtime_ : &depth_runtime_;
      const auto now = Clock::now();
      if (runtime->last_attempt != Clock::time_point::min() &&
        std::chrono::duration<double>(now - runtime->last_attempt).count() < 1.0 / config.max_fps)
      {
        return;
      }
      runtime->last_attempt = now;
      const double elapsed = std::chrono::duration<double>(now - runtime->last_refill).count();
      runtime->tokens = std::min(
        runtime->token_capacity,
        runtime->tokens + elapsed * config.bandwidth_mbps * 1000000.0 / 8.0);
      runtime->last_refill = now;
    }

    std::vector<std::uint8_t> encoded;
    int selected_quality = config.auto_quality ? 70 : config.quality;
    int first_quality = config.quality;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      first_quality = config.auto_quality ?
        std::clamp(runtime->current_quality, 30, 70) : config.quality;
    }
    bool encoded_with_budget = false;
    for (int quality = first_quality; quality >= 30; quality -= 5) {
      std::vector<int> parameters{cv::IMWRITE_JPEG_QUALITY, quality};
      if (!cv::imencode(".jpg", image, encoded, parameters)) {
        throw std::runtime_error("JPEG encoding failed");
      }
      selected_quality = quality;
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto now = Clock::now();
      const double elapsed = std::chrono::duration<double>(now - runtime->last_refill).count();
      runtime->tokens = std::min(
        runtime->token_capacity,
        runtime->tokens + elapsed * config.bandwidth_mbps * 1000000.0 / 8.0);
      runtime->last_refill = now;
      if (static_cast<double>(encoded.size()) <= runtime->tokens) {
        encoded_with_budget = true;
        runtime->tokens -= static_cast<double>(encoded.size());
        runtime->current_quality = config.auto_quality ?
          std::min(70, quality + 5) : config.quality;
        break;
      }
      if (!config.auto_quality) {
        break;
      }
    }
    if (!encoded_with_budget) {
      return;
    }

    sensor_msgs::msg::CompressedImage output;
    output.header = header;
    output.format = "jpeg";
    output.data = std::move(encoded);
    publisher->publish(output);
    (void)selected_quality;
  }

  Extrinsic extrinsic_;
  JointLimits joint_limits_{};
  std::string camera_frame_id_;
  std::string arm_frame_id_;
  double base_linear_speed_threshold_mps_{0.03};
  double base_angular_speed_threshold_rps_{0.08};
  double arm_joint_speed_threshold_rps_{0.05};
  double status_max_age_sec_{0.25};
  std::string odom_frame_;
  std::string annotated_topic_;
  std::string depth_topic_;
  std::string detections_topic_;
  std::string rgb_output_topic_;
  std::string depth_output_topic_;
  std::string arm_telemetry_topic_;
  std::string arm_status_topic_;
  std::string odometry_topic_;

  mutable std::mutex state_mutex_;
  StreamConfig rgb_config_;
  StreamConfig depth_config_;
  StreamRuntime rgb_runtime_;
  StreamRuntime depth_runtime_;
  fruit_perception::msg::FruitDetections::ConstSharedPtr latest_detections_;
  fruit_arm_interfaces::msg::ArmTelemetry::ConstSharedPtr latest_arm_telemetry_;
  fruit_hardware_interfaces::msg::ArmStatus::ConstSharedPtr latest_arm_status_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odometry_;
  Clock::time_point last_arm_telemetry_received_{};
  Clock::time_point last_arm_status_received_{};

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr depth_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr annotated_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<fruit_perception::msg::FruitDetections>::SharedPtr detections_subscription_;
  rclcpp::Subscription<fruit_arm_interfaces::msg::ArmTelemetry>::SharedPtr arm_telemetry_subscription_;
  rclcpp::Subscription<fruit_hardware_interfaces::msg::ArmStatus>::SharedPtr arm_status_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace fruit_robviz_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(fruit_robviz_bridge::ImageBridgeComponent)
