#include "fruit_perception/ripeness_classifier.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <stdexcept>

#include <opencv2/imgproc.hpp>
#include <openvino/runtime/properties.hpp>

namespace fruit_perception
{
namespace
{
constexpr int kSize = 224;
const std::array<std::string, 6> kCrops = {"apple", "pear", "onion", "pepper", "pumpkin", "tomato"};
}

void RipenessClassifier::initialize(
  ov::Core & core, const std::string & model_root, const std::string & engine)
{
  if (engine != "edge" && engine != "yolo") {
    throw std::invalid_argument("ripeness_model must be edge or yolo");
  }
  engine_ = engine;
  const std::vector<std::string> keys = engine == "edge" ?
    std::vector<std::string>(kCrops.begin(), kCrops.end()) : std::vector<std::string>{"unified"};
  for (const auto & key : keys) {
    const auto path = std::filesystem::path(model_root) /
      (engine == "edge" ? "ripeness_edge/" + key : "ripeness/unified") / "best.onnx";
    if (!std::filesystem::is_regular_file(path)) {
      throw std::runtime_error("Missing ripeness model: " + path.string());
    }
    auto model = core.read_model(path.string());
    const ov::PartialShape expected{1, 3, kSize, kSize};
    if (model->inputs().size() != 1 || model->outputs().size() != 1 ||
      model->input().get_element_type() != ov::element::f32 ||
      !model->input().get_partial_shape().compatible(expected) ||
      model->output().get_element_type() != ov::element::f32 ||
      !model->output().get_partial_shape().compatible(ov::PartialShape{1, 2}))
    {
      throw std::runtime_error("Invalid classifier f32 input/output contract: " + path.string());
    }
    if (engine == "yolo" &&
      (!model->has_rt_info({"framework", "names"}) ||
      model->get_rt_info<std::string>({"framework", "names"}) !=
      "{0: 'ripe', 1: 'unripe'}"))
    {
      throw std::runtime_error("Unified classifier labels must be 0=ripe, 1=unripe");
    }
    model->reshape(ov::Shape{1, 3, kSize, kSize});
    auto compiled = core.compile_model(
      model, "GPU.0", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
      ov::hint::inference_precision(ov::element::f16));
    if (compiled.get_property(ov::execution_devices) != std::vector<std::string>{"GPU.0"} ||
      compiled.output().get_shape() != ov::Shape{1, 2})
    {
      throw std::runtime_error("Classifier must execute exclusively on GPU.0 with output [1,2]");
    }
    requests_.emplace(key, compiled.create_infer_request());
  }
}

RipenessResult RipenessClassifier::classify(const cv::Mat & rgb_roi, const std::string & crop)
{
  if (rgb_roi.type() != CV_8UC3 || rgb_roi.rows < 8 || rgb_roi.cols < 8 ||
    std::find(kCrops.begin(), kCrops.end(), crop) == kCrops.end())
  {
    throw std::invalid_argument("Invalid RGB ROI or crop for ripeness classification");
  }
  cv::Mat input_image;
  if (engine_ == "edge") {
    cv::resize(rgb_roi, resized_, cv::Size(kSize, kSize), 0, 0, cv::INTER_LINEAR);
    input_image = resized_;
  } else {
    // torchvision Resize(int) truncates the long side; CenterCrop rounds offsets.
    const int short_side = std::min(rgb_roi.cols, rgb_roi.rows);
    const cv::Size size(
      rgb_roi.cols * kSize / short_side, rgb_roi.rows * kSize / short_side);
    cv::resize(rgb_roi, resized_, size, 0, 0, cv::INTER_LINEAR);
    const int x = static_cast<int>(std::nearbyint((size.width - kSize) / 2.0));
    const int y = static_cast<int>(std::nearbyint((size.height - kSize) / 2.0));
    input_image = resized_(cv::Rect(x, y, kSize, kSize));
  }
  auto & request = requests_.at(engine_ == "edge" ? crop : "unified");
  auto tensor = request.get_input_tensor();
  auto * data = tensor.data<float>();
  constexpr std::array<float, 3> mean{0.485F, 0.456F, 0.406F};
  constexpr std::array<float, 3> stddev{0.229F, 0.224F, 0.225F};
  for (int y = 0; y < kSize; ++y) {
    const auto * row = input_image.ptr<cv::Vec3b>(y);
    for (int x = 0; x < kSize; ++x) {
      for (int c = 0; c < 3; ++c) {
        const float value = row[x][c] / 255.0F;
        data[c * kSize * kSize + y * kSize + x] =
          engine_ == "edge" ? (value - mean[c]) / stddev[c] : value;
      }
    }
  }
  request.infer();
  const auto output = request.get_output_tensor();
  const auto * values = output.data<const float>();
  if (!std::isfinite(values[0]) || !std::isfinite(values[1])) {
    throw std::runtime_error("Non-finite ripeness classifier output");
  }
  float p0 = values[0];
  float p1 = values[1];
  if (engine_ == "edge") {
    const float maximum = std::max(p0, p1);
    p0 = std::exp(p0 - maximum);
    p1 = std::exp(p1 - maximum);
    const float sum = p0 + p1;
    p0 /= sum;
    p1 /= sum;
  } else if (p0 < 0.0F || p1 < 0.0F || p0 > 1.0F || p1 > 1.0F ||
    std::abs(p0 + p1 - 1.0F) > 0.02F)
  {
    throw std::runtime_error("Invalid unified classifier probabilities");
  }
  return p0 >= p1 ? RipenessResult{"ripe", p0} : RipenessResult{"unripe", p1};
}

}  // namespace fruit_perception
