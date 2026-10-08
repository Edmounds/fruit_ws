#include "fruit_perception/detection_parser.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <opencv2/dnn.hpp>

namespace fruit_perception
{
namespace
{

constexpr std::size_t kDetectionRows = 300;
constexpr std::size_t kDetectionColumns = 6;

const std::vector<std::string> kMaturityTypes = {"ripe", "unripe"};
const std::vector<std::string> kTreeFruitTypes = {
  "apple_ripe", "apple_unripe", "pear_ripe", "pear_unripe"};
const std::vector<std::string> kSideFruitTypes = {
  "chili_ripe", "chili_unripe", "pumpkin_ripe", "pumpkin_unripe",
  "onion_ripe", "onion_unripe", "tomato_ripe", "tomato_unripe"};
const std::vector<std::string> kFruitSpecies = {"apple", "pear", "bad"};
const std::vector<std::string> kVegetableSpecies = {"bad", "onion", "pepper", "pumpkin", "tomato"};

const std::vector<std::string> & labels_for_set(DetectionLabelSet label_set)
{
  switch (label_set) {
    case DetectionLabelSet::kMaturity:
      return kMaturityTypes;
    case DetectionLabelSet::kTreeFruit:
      return kTreeFruitTypes;
    case DetectionLabelSet::kSideFruit:
      return kSideFruitTypes;
    case DetectionLabelSet::kFruitSpecies:
      return kFruitSpecies;
    case DetectionLabelSet::kVegetableSpecies:
      return kVegetableSpecies;
  }
  throw std::invalid_argument("Unknown detection label set");
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
  float confidence_threshold,
  const std::vector<std::string> & labels)
{
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
  float nms_iou_threshold,
  const std::vector<std::string> & labels)
{
  if (shape.size() != 3 || shape[0] != 1 || shape[1] < 5 || shape[2] == 0) {
    throw std::invalid_argument("Expected raw YOLO output shape [1,4+classes,N]");
  }

  const std::size_t class_count = shape[1] - 4;
  const std::size_t candidate_count = shape[2];
  if (class_count != labels.size()) {
    throw std::invalid_argument("Raw YOLO class count does not match model labels");
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
    // Bound before converting to int; malformed tensors must not cause overflow.
    const double limit = std::numeric_limits<int>::max() / 4.0;
    if (!std::isfinite(x1) || !std::isfinite(y1) || std::abs(x1) > limit ||
      std::abs(y1) > limit || width > limit || height > limit || best_score > 1.0F)
    {
      continue;
    }

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
  for (std::size_t class_id = 0; class_id < labels.size(); ++class_id) {
    std::vector<cv::Rect> class_boxes;
    std::vector<float> class_scores;
    std::vector<int> indices;
    for (std::size_t i = 0; i < boxes.size(); ++i) {
      if (class_ids[i] == class_id) {
        class_boxes.push_back(boxes[i]);
        class_scores.push_back(scores[i]);
        indices.push_back(static_cast<int>(i));
      }
    }
    std::vector<int> kept;
    cv::dnn::NMSBoxes(class_boxes, class_scores, confidence_threshold, nms_iou_threshold, kept);
    for (const int index : kept) {
      kept_indices.push_back(indices.at(static_cast<std::size_t>(index)));
    }
  }

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

const char * detection_label_set_name(DetectionLabelSet label_set)
{
  switch (label_set) {
    case DetectionLabelSet::kMaturity:
      return "maturity";
    case DetectionLabelSet::kTreeFruit:
      return "tree-fruit";
    case DetectionLabelSet::kSideFruit:
      return "side-fruit";
    case DetectionLabelSet::kFruitSpecies:
      return "fruit-species";
    case DetectionLabelSet::kVegetableSpecies:
      return "vegetable-species";
  }
  return "unknown";
}

std::size_t detection_label_count(DetectionLabelSet label_set)
{
  return labels_for_set(label_set).size();
}

std::vector<Detection> parse_detections(
  const float * output,
  const std::vector<std::size_t> & shape,
  DetectionLabelSet label_set,
  float confidence_threshold,
  float nms_iou_threshold)
{
  validate_thresholds(confidence_threshold, nms_iou_threshold);
  if (output == nullptr) {
    throw std::invalid_argument("Detection output must not be null");
  }
  const auto & labels = labels_for_set(label_set);

  std::vector<Detection> detections;
  if (shape == std::vector<std::size_t>{1, kDetectionRows, kDetectionColumns}) {
    detections = parse_nms_detections(output, confidence_threshold, labels);
  } else {
    detections = parse_raw_yolo_detections(
      output, shape, confidence_threshold, nms_iou_threshold, labels);
  }

  std::stable_sort(
    detections.begin(), detections.end(),
    [](const auto & lhs, const auto & rhs) {return lhs.score > rhs.score;});
  return detections;
}

bool is_supported_detection_output_shape(
  const std::vector<std::size_t> & shape,
  DetectionLabelSet label_set)
{
  if (shape == std::vector<std::size_t>{1, kDetectionRows, kDetectionColumns}) {
    return true;
  }
  return shape.size() == 3 && shape[0] == 1 && shape[2] > 0 &&
         shape[1] == 4 + detection_label_count(label_set);
}

}  // namespace fruit_perception
