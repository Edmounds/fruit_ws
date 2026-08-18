#include "fruit_perception/detection_parser.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

#include <opencv2/dnn.hpp>

namespace fruit_perception
{
namespace
{

constexpr std::size_t kDetectionRows = 300;
constexpr std::size_t kDetectionColumns = 6;
constexpr float kDefaultNmsIouThreshold = 0.45F;

constexpr std::array<const char *, 12> kFruitTypes = {
  "apple_ripe", "apple_unripe", "chili_ripe", "chili_unripe",
  "pumpkin_ripe", "pumpkin_unripe", "onion_ripe", "onion_unripe",
  "pear_ripe", "pear_unripe", "tomato_ripe", "tomato_unripe"};

constexpr std::array<const char *, 2> kMaturityTypes = {"ripe", "unripe"};

constexpr std::array<const char *, 4> kTreeFruitTypes = {
  "apple_ripe", "apple_unripe", "pear_ripe", "pear_unripe"};

constexpr std::array<const char *, 8> kSideFruitTypes = {
  "chili_ripe", "chili_unripe", "pumpkin_ripe", "pumpkin_unripe",
  "onion_ripe", "onion_unripe", "tomato_ripe", "tomato_unripe"};

std::vector<std::string> labels_for_class_count(std::size_t class_count)
{
  if (class_count == kMaturityTypes.size()) {
    return {kMaturityTypes.begin(), kMaturityTypes.end()};
  }
  if (class_count == kTreeFruitTypes.size()) {
    return {kTreeFruitTypes.begin(), kTreeFruitTypes.end()};
  }
  if (class_count == kSideFruitTypes.size()) {
    return {kSideFruitTypes.begin(), kSideFruitTypes.end()};
  }
  if (class_count == kFruitTypes.size()) {
    return {kFruitTypes.begin(), kFruitTypes.end()};
  }
  return {};
}

void validate_thresholds(float confidence_threshold, float nms_iou_threshold)
{
  if (!std::isfinite(confidence_threshold)) {
    throw std::invalid_argument("confidence_threshold must be finite");
  }
  if (!std::isfinite(nms_iou_threshold)) {
    throw std::invalid_argument("nms_iou_threshold must be finite");
  }
}

std::vector<Detection> parse_nms_detections(
  const float * output,
  const std::vector<std::size_t> & shape,
  float confidence_threshold)
{
  if (shape != std::vector<std::size_t>{1, kDetectionRows, kDetectionColumns}) {
    throw std::invalid_argument("Expected model output shape [1,300,6]");
  }

  const auto labels = labels_for_class_count(kFruitTypes.size());
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
    if (class_id < 0 || static_cast<std::size_t>(class_id) >= labels.size()) {
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
        labels[static_cast<std::size_t>(class_id)]});
  }

  return detections;
}

std::vector<Detection> parse_raw_yolo_detections(
  const float * output,
  const std::vector<std::size_t> & shape,
  float confidence_threshold,
  float nms_iou_threshold)
{
  if (shape.size() != 3 || shape[0] != 1 || shape[1] < 5 || shape[2] == 0) {
    throw std::invalid_argument("Expected raw YOLO output shape [1,4+classes,N]");
  }

  const std::size_t class_count = shape[1] - 4;
  const std::size_t candidate_count = shape[2];
  const auto labels = labels_for_class_count(class_count);
  if (labels.empty()) {
    throw std::invalid_argument("Unsupported raw YOLO class count");
  }

  std::vector<cv::Rect> boxes;
  std::vector<float> scores;
  std::vector<std::size_t> class_ids;
  boxes.reserve(candidate_count);
  scores.reserve(candidate_count);
  class_ids.reserve(candidate_count);

  for (std::size_t column = 0; column < candidate_count; ++column) {
    const float x = output[column];
    const float y = output[candidate_count + column];
    const float width = output[2 * candidate_count + column];
    const float height = output[3 * candidate_count + column];
    if (!std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(width) || !std::isfinite(height) ||
      width <= 0.0F || height <= 0.0F)
    {
      continue;
    }

    float best_score = -1.0F;
    std::size_t best_class = 0;
    for (std::size_t class_id = 0; class_id < class_count; ++class_id) {
      const float score = output[(4 + class_id) * candidate_count + column];
      if (std::isfinite(score) && score > best_score) {
        best_score = score;
        best_class = class_id;
      }
    }
    if (best_score < confidence_threshold) {
      continue;
    }

    const float x1 = x - width / 2.0F;
    const float y1 = y - height / 2.0F;
    boxes.emplace_back(
      cv::Rect(
        static_cast<int>(std::round(x1)),
        static_cast<int>(std::round(y1)),
        std::max(1, static_cast<int>(std::round(width))),
        std::max(1, static_cast<int>(std::round(height)))));
    scores.push_back(best_score);
    class_ids.push_back(best_class);
  }

  std::vector<int> kept_indices;
  cv::dnn::NMSBoxes(boxes, scores, confidence_threshold, nms_iou_threshold, kept_indices);

  std::vector<Detection> detections;
  detections.reserve(kept_indices.size());
  for (const int kept_index : kept_indices) {
    if (kept_index < 0 || static_cast<std::size_t>(kept_index) >= boxes.size()) {
      continue;
    }
    const auto index = static_cast<std::size_t>(kept_index);
    const auto & box = boxes[index];
    detections.push_back(
      Detection{
        static_cast<float>(box.x),
        static_cast<float>(box.y),
        static_cast<float>(box.x + box.width),
        static_cast<float>(box.y + box.height),
        scores[index],
        labels[class_ids[index]]});
  }

  return detections;
}

}  // namespace

std::vector<Detection> parse_detections(
  const float * output,
  const std::vector<std::size_t> & shape,
  float confidence_threshold,
  float nms_iou_threshold)
{
  validate_thresholds(confidence_threshold, nms_iou_threshold);

  std::vector<Detection> detections;
  if (shape == std::vector<std::size_t>{1, kDetectionRows, kDetectionColumns}) {
    detections = parse_nms_detections(output, shape, confidence_threshold);
  } else {
    detections = parse_raw_yolo_detections(
      output, shape, confidence_threshold, nms_iou_threshold);
  }

  std::stable_sort(
    detections.begin(), detections.end(),
    [](const auto & lhs, const auto & rhs) {return lhs.score > rhs.score;});

  return detections;
}

std::vector<Detection> parse_detections(
  const float * output,
  const std::vector<std::size_t> & shape,
  float confidence_threshold)
{
  return parse_detections(output, shape, confidence_threshold, kDefaultNmsIouThreshold);
}

std::vector<std::string> parse_fruit_types(
  const float * output,
  const std::vector<std::size_t> & shape,
  float confidence_threshold,
  float nms_iou_threshold)
{
  auto detections = parse_detections(
    output, shape, confidence_threshold, nms_iou_threshold);

  std::vector<std::string> types;
  types.reserve(detections.size());
  for (auto & detection : detections) {
    types.push_back(std::move(detection.type));
  }
  return types;
}

std::vector<std::string> parse_fruit_types(
  const float * output,
  const std::vector<std::size_t> & shape,
  float confidence_threshold)
{
  return parse_fruit_types(output, shape, confidence_threshold, kDefaultNmsIouThreshold);
}

bool is_supported_detection_output_shape(const std::vector<std::size_t> & shape)
{
  if (shape == std::vector<std::size_t>{1, kDetectionRows, kDetectionColumns}) {
    return true;
  }
  if (shape.size() != 3 || shape[0] != 1 || shape[1] < 5 || shape[2] == 0) {
    return false;
  }
  return !labels_for_class_count(shape[1] - 4).empty();
}

}  // namespace fruit_perception
