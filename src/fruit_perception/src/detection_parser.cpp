#include "fruit_perception/detection_parser.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace fruit_perception
{
namespace
{

constexpr std::size_t kDetectionRows = 300;
constexpr std::size_t kDetectionColumns = 6;

constexpr std::array<const char *, 12> kFruitTypes = {
  "apple_ripe", "apple_unripe", "chili_ripe", "chili_unripe",
  "pumpkin_ripe", "pumpkin_unripe", "onion_ripe", "onion_unripe",
  "pear_ripe", "pear_unripe", "tomato_ripe", "tomato_unripe"};

}  // namespace

std::vector<Detection> parse_detections(
  const float * output,
  const std::vector<std::size_t> & shape,
  float confidence_threshold)
{
  if (shape != std::vector<std::size_t>{1, kDetectionRows, kDetectionColumns}) {
    throw std::invalid_argument("Expected model output shape [1,300,6]");
  }
  if (!std::isfinite(confidence_threshold)) {
    throw std::invalid_argument("confidence_threshold must be finite");
  }

  std::vector<Detection> detections;
  for (std::size_t row = 0; row < kDetectionRows; ++row) {
    const auto offset = row * kDetectionColumns;
    const float score = output[offset + 4];
    if (!std::isfinite(score) || score < confidence_threshold) {
      continue;
    }

    const float class_value = output[offset + 5];
    if (!std::isfinite(class_value)) {
      continue;
    }
    const auto class_id = static_cast<long>(std::lround(class_value));
    if (class_id < 0 || static_cast<std::size_t>(class_id) >= kFruitTypes.size()) {
      continue;
    }
    if (!std::isfinite(output[offset]) || !std::isfinite(output[offset + 1]) ||
      !std::isfinite(output[offset + 2]) || !std::isfinite(output[offset + 3]))
    {
      continue;
    }
    detections.push_back(
      Detection{
        output[offset], output[offset + 1], output[offset + 2], output[offset + 3], score,
        kFruitTypes[static_cast<std::size_t>(class_id)]});
  }

  std::stable_sort(
    detections.begin(), detections.end(),
    [](const auto & lhs, const auto & rhs) {return lhs.score > rhs.score;});

  return detections;
}

std::vector<std::string> parse_fruit_types(
  const float * output,
  const std::vector<std::size_t> & shape,
  float confidence_threshold)
{
  auto detections = parse_detections(output, shape, confidence_threshold);

  std::vector<std::string> types;
  types.reserve(detections.size());
  for (auto & detection : detections) {
    types.push_back(std::move(detection.type));
  }
  return types;
}

}  // namespace fruit_perception
