#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <openvino/openvino.hpp>
#include <openvino/runtime/intel_gpu/properties.hpp>
#include <openvino/runtime/properties.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "fruit_perception/b_target_selector.hpp"
#include "fruit_perception/camera_target_selector.hpp"
#include "fruit_perception/detection_output.hpp"
#include "fruit_perception/detection_parser.hpp"
#include "fruit_perception/image_processing.hpp"
#include "fruit_perception/msg/fruit_decision.hpp"
#include "fruit_perception/msg/fruit_detections.hpp"
#include "fruit_perception/msg/fruit_detection.hpp"
#include "fruit_perception/msg/fruit_target.hpp"
#include "fruit_perception/ripeness_classifier.hpp"

namespace fruit_perception
{
namespace
{
struct ObservationWindowCommand
{
  std::string window_id;
  int valid{0};
  std::int64_t start_sec{0};
  std::int64_t start_nanosec{0};
  char area{'A'};
  std::string purpose;
  std::string expected_type;
};

std::optional<ObservationWindowCommand> parse_observation_window_command(
  const std::string & json_str, const rclcpp::Logger & logger)
{
  if (json_str.empty()) {
    return std::nullopt;
  }
  try {
    cv::FileStorage fs(
      json_str, cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
    if (!fs.isOpened()) {
      RCLCPP_WARN(logger, "Failed to parse observation window JSON");
      return std::nullopt;
    }
    const auto wid_node = fs["window_id"];
    const auto valid_node = fs["valid"];
    const auto sec_node = fs["start_sec"];
    const auto nsec_node = fs["start_nanosec"];
    const auto area_node = fs["area"];
    const auto purpose_node = fs["purpose"];
    const auto expected_node = fs["expected_type"];

    if (!wid_node.isString() || !valid_node.isInt() || !sec_node.isInt() ||
      !nsec_node.isInt() || !area_node.isString() || !purpose_node.isString())
    {
      RCLCPP_WARN(logger, "Observation window JSON field types invalid or missing");
      return std::nullopt;
    }
    if (!expected_node.empty() && !expected_node.isString()) {
      RCLCPP_WARN(logger, "Observation window expected_type is not a string");
      return std::nullopt;
    }

    ObservationWindowCommand cmd;
    cmd.window_id = static_cast<std::string>(wid_node);
    if (cmd.window_id.empty()) {
      RCLCPP_WARN(logger, "Observation window window_id is empty");
      return std::nullopt;
    }

    cmd.valid = static_cast<int>(valid_node);
    if (cmd.valid != 0 && cmd.valid != 1) {
      RCLCPP_WARN(logger, "Observation window valid flag must be 0 or 1, got %d", cmd.valid);
      return std::nullopt;
    }

    cmd.start_sec = static_cast<std::int64_t>(static_cast<int>(sec_node));
    if (cmd.start_sec < 0) {
      RCLCPP_WARN(logger, "Observation window start_sec must be non-negative");
      return std::nullopt;
    }

    cmd.start_nanosec = static_cast<std::int64_t>(static_cast<int>(nsec_node));
    if (cmd.start_nanosec < 0 || cmd.start_nanosec >= 1000000000LL) {
      RCLCPP_WARN(logger, "Observation window start_nanosec must be in [0, 999999999]");
      return std::nullopt;
    }

    const std::string area_str = static_cast<std::string>(area_node);
    if (area_str != "A" && area_str != "B" && area_str != "C") {
      RCLCPP_WARN(logger, "Observation window area '%s' invalid", area_str.c_str());
      return std::nullopt;
    }
    cmd.area = area_str.front();

    cmd.purpose = static_cast<std::string>(purpose_node);
    if (cmd.purpose != "harvest" && cmd.purpose != "qr") {
      RCLCPP_WARN(logger, "Observation window purpose '%s' invalid", cmd.purpose.c_str());
      return std::nullopt;
    }

    if (!expected_node.empty()) {
      cmd.expected_type = static_cast<std::string>(expected_node);
    }
    return cmd;
  } catch (const cv::Exception & e) {
    RCLCPP_WARN(logger, "Observation window JSON parse exception: %s", e.what());
    return std::nullopt;
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger, "Observation window exception: %s", e.what());
    return std::nullopt;
  }
}

bool is_stamp_at_or_after(
  const builtin_interfaces::msg::Time & stamp, std::int64_t start_sec,
  std::int64_t start_nanosec)
{
  const auto stamp_sec = static_cast<std::int64_t>(stamp.sec);
  const auto stamp_nanosec = static_cast<std::int64_t>(stamp.nanosec);
  if (stamp_sec > start_sec) {
    return true;
  }
  if (stamp_sec == start_sec && stamp_nanosec >= start_nanosec) {
    return true;
  }
  return false;
}
}  // namespace
class FruitPerceptionNode : public rclcpp::Node
{
public:
  explicit FruitPerceptionNode(const rclcpp::NodeOptions & options)
  : Node("fruit_perception_node", options)
  {
    const auto model_root = declare_parameter(
      "model_root", "/home/rc4/fruit_ws/src/fruit_perception/model");
    if (!std::filesystem::path(model_root).is_absolute()) {
      throw std::invalid_argument("model_root must be an absolute source-directory path");
    }
    ripeness_model_ = declare_parameter("ripeness_model", "edge");
    performance_hint_ = declare_parameter("performance_hint", "latency");
    if (performance_hint_ != "latency" && performance_hint_ != "throughput") {
      throw std::invalid_argument("performance_hint must be latency or throughput");
    }

    confidence_threshold_ = static_cast<float>(declare_parameter("confidence_threshold", 0.50));
    nms_iou_threshold_ = static_cast<float>(declare_parameter("nms_iou_threshold", 0.45));
    target_confidence_weight_ =
      static_cast<float>(declare_parameter("camera_target_confidence_weight", 0.70));
    target_score_close_threshold_ =
      static_cast<float>(declare_parameter("camera_target_score_close_threshold", 0.05));
    const auto target_acquisition_frames =
      declare_parameter<std::int64_t>("camera_target_acquisition_frames", 8);
    const auto target_association_radius =
      declare_parameter("camera_target_association_radius_m", 0.03);
    const auto target_outlier_sigma =
      declare_parameter("camera_target_outlier_sigma", 2.0);
    const auto target_lock_miss_frames =
      declare_parameter<std::int64_t>("camera_target_lock_miss_frames", 8);
    if (target_acquisition_frames <= 0) {
      throw std::invalid_argument("camera_target_acquisition_frames must be positive");
    }
    if (target_lock_miss_frames <= 0) {
      throw std::invalid_argument("camera_target_lock_miss_frames must be positive");
    }
    target_selector_config_ = CameraTargetSelectorConfig{
      static_cast<std::size_t>(target_acquisition_frames),
      target_association_radius,
      target_outlier_sigma,
      static_cast<std::size_t>(target_lock_miss_frames),
      target_confidence_weight_,
      target_score_close_threshold_};
    const auto min_observations =
      declare_parameter<std::int64_t>("camera_target_min_observations", 6);
    const auto max_missed_frames =
      declare_parameter<std::int64_t>("camera_target_max_missed_frames", 2);
    if (min_observations <= 0 || max_missed_frames < 0) {
      throw std::invalid_argument("Invalid target confirmation observation/miss counts");
    }
    target_selector_config_.min_observations = static_cast<std::size_t>(min_observations);
    target_selector_config_.type_support_ratio =
      declare_parameter("camera_target_type_support_ratio", 0.75);
    target_selector_config_.max_missed_frames = static_cast<std::size_t>(max_missed_frames);
    target_selector_config_.maturity_confidence_threshold =
      declare_parameter("camera_target_maturity_confidence_threshold", 0.70);

    const auto b_target_acquisition_frames =
      declare_parameter<std::int64_t>("camera_b_target_acquisition_frames", 8);
    if (b_target_acquisition_frames <= 0) {
      throw std::invalid_argument("camera_b_target_acquisition_frames must be positive");
    }
    b_target_selector_config_ = BTargetSelectorConfig{
      static_cast<std::size_t>(b_target_acquisition_frames),
      declare_parameter("camera_b_target_association_radius_m", 0.03),
      declare_parameter("camera_b_target_outlier_sigma", 2.0)};
    b_target_selector_config_.min_observations = target_selector_config_.min_observations;
    b_target_selector_config_.type_support_ratio = target_selector_config_.type_support_ratio;
    b_target_selector_config_.max_missed_frames = target_selector_config_.max_missed_frames;
    b_target_selector_config_.maturity_confidence_threshold =
      target_selector_config_.maturity_confidence_threshold;

    const auto observation_window_topic = declare_parameter(
      "observation_window_topic", "/fruit_mission/observation_window");
    observation_window_subscription_ = create_subscription<std_msgs::msg::String>(
      observation_window_topic, rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        on_observation_window(message);
      });

    const auto zone = declare_parameter("zone", "A");
    if (zone != "A" && zone != "B" && zone != "C") {
      throw std::invalid_argument("zone must be A, B or C");
    }
    zone_.store(zone.front());
    zone_parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        char next_zone = zone_.load();
        for (const auto & parameter : parameters) {
          if (parameter.get_name() != "zone") {
            continue;
          }
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
          (parameter.as_string() != "A" && parameter.as_string() != "B" &&
          parameter.as_string() != "C"))
          {
            result.successful = false;
            result.reason = "zone must be A, B or C";
            return result;
          }
          next_zone = parameter.as_string().front();
        }
        std::lock_guard<std::mutex> lock(window_mutex_);
        if (zone_.load() != next_zone) {
          zone_.store(next_zone);
          current_window_.valid = false;
          target_selection_generation_.fetch_add(1U);
          RCLCPP_INFO(get_logger(), "Perception zone changed to %c; observation closed", next_zone);
        }
        return result;
      });

    const auto decision_topic = declare_parameter(
      "decision_topic", "/fruit_perception/decision");
    decision_publisher_ = create_publisher<msg::FruitDecision>(decision_topic, rclcpp::QoS(10));
    const auto preview_detections_topic = declare_parameter(
      "preview_detections_topic", "/fruit_perception/camera/preview_detections");
    preview_detections_publisher_ = create_publisher<msg::FruitDetections>(
      preview_detections_topic, rclcpp::SensorDataQoS().keep_last(1));

    const bool enable_camera = declare_parameter("enable_camera", true);
    if (!enable_camera) {
      throw std::invalid_argument("enable_camera must be true");
    }

    if (!std::isfinite(confidence_threshold_) ||
      confidence_threshold_ < 0.0F || confidence_threshold_ > 1.0F)
    {
      throw std::invalid_argument("confidence_threshold must be between 0 and 1");
    }
    if (!std::isfinite(nms_iou_threshold_) ||
      nms_iou_threshold_ < 0.0F || nms_iou_threshold_ > 1.0F)
    {
      throw std::invalid_argument("nms_iou_threshold must be between 0 and 1");
    }
    if (!std::isfinite(target_confidence_weight_) ||
      target_confidence_weight_ < 0.0F || target_confidence_weight_ > 1.0F)
    {
      throw std::invalid_argument("camera_target_confidence_weight must be between 0 and 1");
    }
    if (!std::isfinite(target_score_close_threshold_) ||
      target_score_close_threshold_ < 0.0F || target_score_close_threshold_ > 1.0F)
    {
      throw std::invalid_argument("camera_target_score_close_threshold must be between 0 and 1");
    }

    core_.set_property(ov::cache_dir("/home/rc4/fruit_ws/tmp/openvino_cache"));
    // Avoid the Intel OpenCL LLVM/SPIR-V crash during parallel kernel compilation.
    core_.set_property("GPU.0", ov::compilation_num_threads(1));
    core_.set_property(
      "GPU.0", ov::intel_gpu::hint::queue_throttle(ov::intel_gpu::hint::ThrottleLevel::LOW));

    initialize_model(
      vegetable_model_, model_root + "/vegetable_type/best.onnx", 1U,
      "A/C vegetable", DetectionLabelSet::kVegetableSpecies);
    initialize_model(
      fruit_model_, model_root + "/fruit_type/best.onnx", 1U,
      "B fruit", DetectionLabelSet::kFruitSpecies);
    ripeness_classifier_.initialize(core_, model_root, ripeness_model_);

    rgbd_heartbeat_publisher_ = create_publisher<std_msgs::msg::Header>(
      declare_parameter(
        "camera_rgbd_heartbeat_topic", "/fruit_perception/camera/rgbd_heartbeat"),
      rclcpp::SensorDataQoS().keep_last(1));

    add_rgbd_stream(
      vegetable_model_,
      fruit_model_,
      declare_parameter("camera_rgbd_topic", "/camera/rgbd"),
      declare_parameter(
        "camera_annotated_topic", "/fruit_perception/camera/annotated_image"));

    RCLCPP_INFO(
      get_logger(),
      "Started camera perception stack: root=%s, ripeness=%s, hint=%s, conf=%.2f, nms=%.2f",
      model_root.c_str(), ripeness_model_.c_str(), performance_hint_.c_str(),
      confidence_threshold_, nms_iou_threshold_);
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
      std::string stream_name,
      rclcpp::Publisher<msg::FruitDecision>::SharedPtr decision_publisher,
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher)
    : name(std::move(stream_name)),
      decision_publisher(std::move(decision_publisher)),
      image_publisher(std::move(image_publisher)),
      input_image(kModelInputSize, kModelInputSize, CV_8UC3, cv::Scalar(114, 114, 114))
    {
    }

    struct RequestSlot
    {
      DetectionLabelSet label_set;
      ov::InferRequest request;
      ov::Shape output_shape;
    };

    std::string name;
    rclcpp::Publisher<msg::FruitDecision>::SharedPtr decision_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
    std::vector<std::unique_ptr<RequestSlot>> request_slots;
    RequestSlot * vegetable_request{nullptr};
    RequestSlot * fruit_request{nullptr};
    cv::Mat input_image;
    cv_bridge::CvImageConstPtr frame;
    cv_bridge::CvImageConstPtr aligned_depth;
    sensor_msgs::msg::CameraInfo depth_camera_info;
    LetterboxTransform transform{};
    std_msgs::msg::Header header;
    std::chrono::steady_clock::time_point last_completed_at;
    double displayed_fps{0.0};
    std::chrono::steady_clock::time_point submitted_at;
    std::chrono::steady_clock::time_point statistics_started;
    std::size_t completed_frames{0U};
    std::size_t classified_rois{0U};
    double total_detection_ms{0.0};
    double total_classification_ms{0.0};
    double total_processing_ms{0.0};
    std::unique_ptr<CameraTargetSelector> target_selector;
    std::unique_ptr<BTargetSelector> b_target_selector;
    std::uint64_t target_selection_generation{0U};
    std::uint64_t submitted_target_selection_generation{0U};
    std::atomic<bool> busy{false};

    struct SubmissionContext
    {
      std::string window_id;
      bool is_harvest_window{false};
      std::int64_t start_sec{0};
      std::int64_t start_nanosec{0};
      char area{'A'};
      std::string expected_type;
      builtin_interfaces::msg::Time rgb_stamp;
      builtin_interfaces::msg::Time depth_stamp;
    };
    SubmissionContext submission_context;
  };

  struct ModelRuntime
  {
    ov::CompiledModel compiled_model;
    ov::Shape output_shape;
    DetectionLabelSet label_set;
  };

  void initialize_model(
    ModelRuntime & runtime,
    const std::string & model_path,
    std::size_t request_count,
    const std::string & model_name,
    DetectionLabelSet label_set)
  {
    const auto available_devices = core_.get_available_devices();
    if (std::find(available_devices.begin(), available_devices.end(), "GPU") ==
      available_devices.end())
    {
      throw std::runtime_error("OpenVINO GPU device is not available; CPU fallback is disabled");
    }

    if (!std::filesystem::is_regular_file(model_path)) {
      throw std::runtime_error("Missing detector: " + model_path);
    }
    auto model = core_.read_model(model_path);
    const std::string expected_names = label_set == DetectionLabelSet::kFruitSpecies ?
      "{0: 'apple', 1: 'pear', 2: 'bad'}" :
      "{0: 'bad', 1: 'onion', 2: 'pepper', 3: 'pumpkin', 4: 'tomato'}";
    if (model->inputs().size() != 1 || model->outputs().size() != 1 ||
      !model->has_rt_info({"framework", "names"}) ||
      model->get_rt_info<std::string>({"framework", "names"}) != expected_names ||
      !model->has_rt_info({"framework", "end2end"}) ||
      model->get_rt_info<std::string>({"framework", "end2end"}) != "False")
    {
      throw std::runtime_error("Detector species labels or raw-output contract mismatch");
    }

    const auto input = model->input();
    if (input.get_element_type() != ov::element::f32 ||
      input.get_shape() != ov::Shape{1, 3, kModelInputSize, kModelInputSize})
    {
      throw std::runtime_error("Expected model input f32 [1,3,640,640]");
    }

    ov::preprocess::PrePostProcessor preprocessor(model);
    auto & input_info = preprocessor.input();
    input_info.tensor().set_element_type(ov::element::u8).set_layout("NHWC");
    input_info.preprocess().convert_element_type(ov::element::f32).scale(255.0F);
    input_info.model().set_layout("NCHW");
    model = preprocessor.build();

    runtime.compiled_model = core_.compile_model(
      model, "GPU.0",
      ov::hint::inference_precision(ov::element::f16),
      ov::hint::performance_mode(
        performance_hint_ == "throughput" ?
        ov::hint::PerformanceMode::THROUGHPUT : ov::hint::PerformanceMode::LATENCY),
      ov::hint::num_requests(static_cast<std::uint32_t>(request_count)));

    const auto execution_devices = runtime.compiled_model.get_property(ov::execution_devices);
    if (execution_devices.size() != 1 || execution_devices.front() != "GPU.0") {
      throw std::runtime_error("Model did not compile exclusively to GPU.0");
    }
    if (runtime.compiled_model.input().get_element_type() != ov::element::u8 ||
      runtime.compiled_model.input().get_shape() !=
      ov::Shape{1, kModelInputSize, kModelInputSize, 3})
    {
      throw std::runtime_error("Expected compiled model input u8 [1,640,640,3]");
    }
    if (runtime.compiled_model.output().get_element_type() != ov::element::f32 ||
      !is_supported_detection_output_shape(
        runtime.compiled_model.output().get_shape(), label_set))
    {
      throw std::runtime_error(
              "Model " + model_name + " output does not match its label set");
    }

    runtime.output_shape = runtime.compiled_model.output().get_shape();
    runtime.label_set = label_set;
    RCLCPP_INFO(
      get_logger(),
      "OpenVINO model %s compiled successfully; labels=%s (%zu classes), device=%s",
      model_name.c_str(), detection_label_set_name(label_set),
      detection_label_count(label_set), execution_devices.front().c_str());
  }

  void add_rgbd_stream(
    ModelRuntime & vegetable_model,
    ModelRuntime & fruit_model,
    const std::string & input_topic,
    const std::string & annotated_topic)
  {
    auto stream = std::make_unique<Stream>(
      "camera",
      decision_publisher_,
      create_publisher<sensor_msgs::msg::Image>(
        annotated_topic, rclcpp::QoS(1).reliable()));
    stream->target_selector = std::make_unique<CameraTargetSelector>(target_selector_config_);
    stream->b_target_selector = std::make_unique<BTargetSelector>(b_target_selector_config_);
    stream->target_selection_generation = target_selection_generation_.load();
    auto * stream_ptr = stream.get();
    stream->vegetable_request = add_request_slot(*stream, vegetable_model);
    stream->fruit_request = add_request_slot(*stream, fruit_model);
    for (auto & slot : stream->request_slots) {
      auto * slot_ptr = slot.get();
      slot->request.set_callback(
        [this, stream_ptr, slot_ptr](std::exception_ptr error) {
          finish_inference(*stream_ptr, *slot_ptr, error);
        });
    }
    streams_.push_back(std::move(stream));

    subscriptions_.push_back(
      create_subscription<realsense2_camera_msgs::msg::RGBD>(
        input_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this, stream_ptr](realsense2_camera_msgs::msg::RGBD::ConstSharedPtr rgbd) {
          rgbd_heartbeat_publisher_->publish(rgbd->header);
          submit_rgbd(*stream_ptr, rgbd);
        }));
    RCLCPP_INFO(
      get_logger(), "camera: %s -> %s and %s", input_topic.c_str(),
      decision_publisher_->get_topic_name(), annotated_topic.c_str());
  }

  void submit_rgbd(
    Stream & stream, const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr & rgbd)
  {
    if (stopping_.load()) {
      return;
    }

    bool expected = false;
    if (!stream.busy.compare_exchange_strong(expected, true)) {
      return;
    }

    try {
      stream.submitted_at = std::chrono::steady_clock::now();
      const auto target_generation = target_selection_generation_.load();
      if (stream.target_selection_generation != target_generation) {
        stream.target_selector->reset();
        stream.b_target_selector->reset();
        stream.target_selection_generation = target_generation;
      }
      stream.submitted_target_selection_generation = target_generation;

      Stream::SubmissionContext context;
      {
        std::lock_guard<std::mutex> lock(window_mutex_);
        if (current_window_.valid && current_window_.purpose == "harvest") {
          context.window_id = current_window_.window_id;
          context.is_harvest_window = true;
          context.start_sec = current_window_.start_sec;
          context.start_nanosec = current_window_.start_nanosec;
          context.expected_type = current_window_.expected_type;
        } else {
          context.window_id = "";
          context.is_harvest_window = false;
        }
        context.area = zone_.load();
      }
      context.rgb_stamp = rgbd->rgb.header.stamp;
      context.depth_stamp = rgbd->depth.header.stamp;
      stream.submission_context = std::move(context);

      stream.frame = cv_bridge::toCvShare(
        rgbd->rgb, rgbd, sensor_msgs::image_encodings::RGB8);
      stream.aligned_depth = cv_bridge::toCvShare(
        rgbd->depth, rgbd, sensor_msgs::image_encodings::TYPE_16UC1);
      stream.depth_camera_info = rgbd->depth_camera_info;
      stream.transform = letterbox_image(stream.frame->image, stream.input_image);
      stream.header = rgbd->rgb.header;
      auto * request = select_camera_request(stream, stream.submission_context.area);
      if (request == nullptr) {
        stream.busy.store(false);
        return;
      }
      request->request.start_async();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "camera RGBD frame rejected: %s", error.what());
      stream.busy.store(false);
    }
  }

  void finish_inference(Stream & stream, Stream::RequestSlot & request, std::exception_ptr error)
  {
    try {
      if (error) {
        std::rethrow_exception(error);
      }
      const auto detected_at = std::chrono::steady_clock::now();
      const auto output = request.request.get_output_tensor();
      auto detections = parse_detections(
        output.data<const float>(),
        request.output_shape,
        request.label_set,
        confidence_threshold_,
        nms_iou_threshold_);

      const auto context = stream.submission_context;
      const char area = context.area;
      std::vector<Detection> classified;
      classified.reserve(detections.size());
      std::size_t roi_count = 0U;
      for (auto detection : detections) {
        const auto box = to_original_box(detection, stream.transform);
        if (box.empty()) {
          continue;
        }
        if (detection.type == "bad") {
          classified.push_back(std::move(detection));
          continue;
        }
        const int x1 = std::max(0, static_cast<int>(std::floor(box.x - box.width * 0.1)));
        const int y1 = std::max(0, static_cast<int>(std::floor(box.y - box.height * 0.1)));
        const int x2 = std::min(
          stream.frame->image.cols,
          static_cast<int>(std::ceil(box.x + box.width * 1.1)));
        const int y2 = std::min(
          stream.frame->image.rows,
          static_cast<int>(std::ceil(box.y + box.height * 1.1)));
        if (x2 - x1 < 8 || y2 - y1 < 8) {
          continue;
        }
        try {
          ++roi_count;
          const auto maturity = ripeness_classifier_.classify(
            stream.frame->image(cv::Rect(x1, y1, x2 - x1, y2 - y1)), detection.type);
          RCLCPP_DEBUG(
            get_logger(), "%s: %s probability=%.4f", detection.type.c_str(),
            maturity.maturity.c_str(), maturity.confidence);
          detection.species = detection.type;
          detection.maturity = maturity.maturity;
          detection.maturity_confidence = maturity.confidence;
          // Retain species internally even in A, so species flips count as conflicts.
          if (detection.type == "pepper") {
            detection.type = "chili";
          }
          detection.type += "_" + maturity.maturity;
          classified.push_back(std::move(detection));
        } catch (const std::exception & classification_error) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000, "ROI classification rejected: %s",
            classification_error.what());
        }
      }
      detections = std::move(classified);
      const auto classified_at = std::chrono::steady_clock::now();

      msg::FruitDecision result;
      const auto business_type = [](const std::string & type) {
          return type;
        };
      result.header = stream.aligned_depth->header;
      result.types.reserve(detections.size());
      for (const auto & detection : detections) {
        result.types.push_back(business_type(detection.type));
      }
      const auto outputs = make_detection_outputs(
        detections, stream.transform, stream.aligned_depth->image,
        stream.depth_camera_info);
      for (std::size_t index = 0U; index < outputs.size(); ++index) {
        if (outputs[index].has_point) {
          detections[index].depth_m = outputs[index].point.z;
        }
      }
      result.detections.reserve(outputs.size());
      for (const auto & output : outputs) {
        msg::FruitDetection item;
        item.type = business_type(output.type);
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
        result.detections.push_back(std::move(item));
      }

      msg::FruitDetections preview_detections;
      preview_detections.header = stream.header;
      preview_detections.detections = result.detections;
      preview_detections_publisher_->publish(preview_detections);

      const bool in_window = context.is_harvest_window &&
        !context.window_id.empty() &&
        (stream.submitted_target_selection_generation == target_selection_generation_.load()) &&
        is_stamp_at_or_after(context.rgb_stamp, context.start_sec, context.start_nanosec) &&
        is_stamp_at_or_after(context.depth_stamp, context.start_sec, context.start_nanosec);

      if (in_window) {
        result.observation_window_id = context.window_id;
        if (area == 'B') {
          std::vector<BTargetCandidate> candidates;
          candidates.reserve(outputs.size());
          for (std::size_t index = 0U; index < outputs.size(); ++index) {
            const auto & output = outputs[index];
            if (output.type == "bad" || !output.has_point ||
              !std::isfinite(output.point.x) ||
              !std::isfinite(output.point.y) ||
              !std::isfinite(output.point.z))
            {
              continue;
            }
            candidates.push_back(
              BTargetCandidate{
                output.type,
                output.point,
                cv::Point2d{
                  static_cast<double>(output.roi.x) +
                  static_cast<double>(output.roi.width) * 0.5,
                  static_cast<double>(output.roi.y) +
                  static_cast<double>(output.roi.height) * 0.5},
                static_cast<std::size_t>(stream.transform.original_width),
                static_cast<double>(output.confidence), detections[index].maturity_confidence});
          }

          const auto target_update = stream.b_target_selector->update(candidates);
          result.targets_ready = target_update.ready;
          result.targets.reserve(target_update.targets.size());
          for (const auto & target : target_update.targets) {
            msg::FruitTarget item;
            item.type = target.type;
            item.confidence = static_cast<float>(target.confidence);
            item.has_point = true;
            item.point.x = target.point.x;
            item.point.y = target.point.y;
            item.point.z = target.point.z;
            item.pixel_x = static_cast<float>(target.pixel_center.x);
            item.pixel_y = static_cast<float>(target.pixel_center.y);
            item.image_width = static_cast<std::uint32_t>(target.image_width);
            item.observation_count = static_cast<std::uint32_t>(target.observation_count);
            item.inlier_count = static_cast<std::uint32_t>(target.inlier_count);
            result.targets.push_back(std::move(item));
          }
          if (!result.targets.empty()) {
            result.has_target = true;
            result.target_point = result.targets.front().point;
          }
        } else {
          std::vector<CameraTargetCandidate> candidates;
          candidates.reserve(outputs.size());
          const auto expected_type = context.expected_type;
          const double half_width = static_cast<double>(stream.frame->image.cols) * 0.5;
          for (std::size_t index = 0U; index < outputs.size(); ++index) {
            const auto & output = outputs[index];
            if (output.type == "bad" || !output.has_point ||
              !std::isfinite(output.point.x) ||
              !std::isfinite(output.point.y) ||
              !std::isfinite(output.point.z))
            {
              continue;
            }
            const double roi_center_x =
              static_cast<double>(output.roi.x) + static_cast<double>(output.roi.width) * 0.5;
            const double horizontal_offset = std::clamp(
              std::abs(roi_center_x - half_width) / half_width, 0.0, 1.0);
            candidates.push_back(
              CameraTargetCandidate{
                output.point, static_cast<double>(output.confidence), 1.0 - horizontal_offset,
                output.type, detections[index].maturity_confidence});
          }

          const auto target_update = stream.target_selector->update(candidates, expected_type);
          if (target_update.lock_lost) {
            RCLCPP_WARN(
              get_logger(),
              "Camera target lock lost after %zu missed frames; reacquiring",
              target_selector_config_.lock_miss_frames);
          }
          if (target_update.lock_acquired) {
            RCLCPP_INFO(
              get_logger(),
              "Camera target locked with %zu/%zu inliers in a %zu-frame window: score=%.3f, "
              "confidence=%.3f, horizontal_centrality=%.3f, "
              "mean=(%.3f, %.3f, %.3f), stddev=(%.4f, %.4f, %.4f)",
              target_update.inlier_count, target_update.observation_count,
              target_selector_config_.acquisition_frames,
              target_update.score, target_update.confidence,
              target_update.horizontal_centrality,
              target_update.reference_point.x, target_update.reference_point.y,
              target_update.reference_point.z, target_update.stddev.x,
              target_update.stddev.y, target_update.stddev.z);
          }
          if (target_update.point) {
            result.has_target = true;
            result.target_point.x = target_update.point->x;
            result.target_point.y = target_update.point->y;
            result.target_point.z = target_update.point->z;
          }
        }
      } else {
        result.observation_window_id = "";
        result.has_target = false;
        result.targets_ready = false;
        result.targets.clear();
        result.target_point = geometry_msgs::msg::Point();
      }

      if (stopping_.load() || stream.submitted_target_selection_generation !=
        target_selection_generation_.load())
      {
        stream.busy.store(false);
        return;
      }
      stream.decision_publisher->publish(result);

      const auto now = std::chrono::steady_clock::now();
      if (stream.last_completed_at.time_since_epoch().count() != 0) {
        const double interval =
          std::chrono::duration<double>(now - stream.last_completed_at).count();
        const double instantaneous_fps = interval > 0.0 ? 1.0 / interval : 0.0;
        stream.displayed_fps = stream.displayed_fps == 0.0 ? instantaneous_fps :
          0.8 * stream.displayed_fps + 0.2 * instantaneous_fps;
      }
      stream.last_completed_at = now;

      if (stream.statistics_started.time_since_epoch().count() == 0) {
        stream.statistics_started = stream.submitted_at;
      }
      ++stream.completed_frames;
      stream.classified_rois += roi_count;
      stream.total_detection_ms +=
        std::chrono::duration<double, std::milli>(detected_at - stream.submitted_at).count();
      stream.total_classification_ms +=
        std::chrono::duration<double, std::milli>(classified_at - detected_at).count();
      stream.total_processing_ms +=
        std::chrono::duration<double, std::milli>(now - stream.submitted_at).count();
      const double statistics_seconds =
        std::chrono::duration<double>(now - stream.statistics_started).count();
      if (statistics_seconds >= 5.0) {
        const double frames = static_cast<double>(stream.completed_frames);
        RCLCPP_INFO(
          get_logger(), "camera performance: engine=%s area=%c fps=%.2f "
          "processing_ms=%.2f detector_ms=%.2f classification_ms=%.2f rois_per_frame=%.2f",
          ripeness_model_.c_str(), area, frames / statistics_seconds,
          stream.total_processing_ms / frames, stream.total_detection_ms / frames,
          stream.total_classification_ms / frames, stream.classified_rois / frames);
        stream.statistics_started = now;
        stream.completed_frames = stream.classified_rois = 0U;
        stream.total_detection_ms = stream.total_classification_ms = stream.total_processing_ms =
          0.0;
      }

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
      if (!stopping_.load() && stream.submitted_target_selection_generation ==
        target_selection_generation_.load())
      {
        msg::FruitDecision empty;
        empty.header = stream.header;
        empty.observation_window_id = "";
        empty.has_target = false;
        empty.targets_ready = false;
        stream.decision_publisher->publish(empty);
      }
    }
    stream.busy.store(false);
  }

  static void wait_for_stream(const std::unique_ptr<Stream> & stream)
  {
    for (const auto & slot : stream->request_slots) {
      try {
        slot->request.wait();
      } catch (const std::exception &) {
      }
    }
  }

  Stream::RequestSlot * add_request_slot(Stream & stream, ModelRuntime & model)
  {
    auto slot = std::make_unique<Stream::RequestSlot>();
    slot->label_set = model.label_set;
    slot->request = model.compiled_model.create_infer_request();
    slot->output_shape = model.output_shape;
    slot->request.set_input_tensor(
      ov::Tensor(
        ov::element::u8,
        ov::Shape{
        1U, static_cast<std::size_t>(kModelInputSize),
        static_cast<std::size_t>(kModelInputSize), 3U},
        stream.input_image.data));
    stream.request_slots.push_back(std::move(slot));
    return stream.request_slots.back().get();
  }

  Stream::RequestSlot * select_camera_request(Stream & stream, char area) const
  {
    if (area == 'A' || area == 'C') {
      return stream.vegetable_request;
    }
    if (area == 'B') {
      return stream.fruit_request;
    }
    return nullptr;
  }

  void on_observation_window(const std_msgs::msg::String::ConstSharedPtr & message)
  {
    const auto cmd = parse_observation_window_command(message->data, get_logger());
    if (!cmd) {
      return;
    }
    std::lock_guard<std::mutex> lock(window_mutex_);
    if (cmd->valid == 1) {
      if (cmd->purpose == "harvest" && cmd->area != zone_.load()) {
        current_window_.valid = false;
        target_selection_generation_.fetch_add(1U);
        RCLCPP_WARN(
          get_logger(), "Ignoring harvest window for area %c; zone parameter is %c",
          cmd->area, zone_.load());
        return;
      }
      current_window_.window_id = cmd->window_id;
      current_window_.valid = true;
      current_window_.start_sec = cmd->start_sec;
      current_window_.start_nanosec = cmd->start_nanosec;
      current_window_.area = cmd->area;
      current_window_.purpose = cmd->purpose;
      current_window_.expected_type = cmd->expected_type;
      target_selection_generation_.fetch_add(1U);
      RCLCPP_INFO(
        get_logger(),
        "Opened observation window '%s': area=%c, purpose=%s, start=%ld.%09ld, expected='%s'",
        cmd->window_id.c_str(), cmd->area, cmd->purpose.c_str(),
        static_cast<long>(cmd->start_sec), static_cast<long>(cmd->start_nanosec),
        cmd->expected_type.c_str());
    } else {
      if (current_window_.valid && current_window_.window_id == cmd->window_id) {
        current_window_.valid = false;
        target_selection_generation_.fetch_add(1U);
        RCLCPP_INFO(
          get_logger(), "Closed observation window '%s'", cmd->window_id.c_str());
      } else if (current_window_.valid) {
        RCLCPP_WARN(
          get_logger(),
          "Ignoring close for window '%s'; active window is '%s'",
          cmd->window_id.c_str(), current_window_.window_id.c_str());
      }
    }
  }

  ov::Core core_;
  ModelRuntime vegetable_model_;
  ModelRuntime fruit_model_;
  RipenessClassifier ripeness_classifier_;
  std::string ripeness_model_;
  std::string performance_hint_;
  float confidence_threshold_{0.50F};
  float nms_iou_threshold_{0.45F};
  float target_confidence_weight_{0.70F};
  float target_score_close_threshold_{0.05F};
  CameraTargetSelectorConfig target_selector_config_{8U, 0.03, 2.0, 8U, 0.70, 0.05};
  BTargetSelectorConfig b_target_selector_config_{8U, 0.03, 2.0};
  std::atomic<std::uint64_t> target_selection_generation_{0U};
  std::atomic<bool> stopping_{false};
  std::atomic<char> zone_{'A'};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr zone_parameter_callback_;

  struct ActiveWindow
  {
    std::string window_id;
    bool valid{false};
    std::int64_t start_sec{0};
    std::int64_t start_nanosec{0};
    char area{'A'};
    std::string purpose;
    std::string expected_type;
  };
  ActiveWindow current_window_;
  std::mutex window_mutex_;

  rclcpp::Publisher<msg::FruitDecision>::SharedPtr decision_publisher_;
  rclcpp::Publisher<msg::FruitDetections>::SharedPtr preview_detections_publisher_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr rgbd_heartbeat_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr observation_window_subscription_;
  std::vector<std::unique_ptr<Stream>> streams_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
};

}  // namespace fruit_perception

RCLCPP_COMPONENTS_REGISTER_NODE(fruit_perception::FruitPerceptionNode)
