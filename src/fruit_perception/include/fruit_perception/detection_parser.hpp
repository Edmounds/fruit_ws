#ifndef FRUIT_PERCEPTION__DETECTION_PARSER_HPP_
#define FRUIT_PERCEPTION__DETECTION_PARSER_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace fruit_perception
{

enum class DetectionLabelSet
{
  kMaturity,
  kTreeFruit,
  kSideFruit,
  kFruitSpecies,
  kVegetableSpecies,
};

struct Detection
{
  float x1;
  float y1;
  float x2;
  float y2;
  float score;
  std::string type;
};

const char * detection_label_set_name(DetectionLabelSet label_set);
std::size_t detection_label_count(DetectionLabelSet label_set);

std::vector<Detection> parse_detections(
  const float * output,
  const std::vector<std::size_t> & shape,
  DetectionLabelSet label_set,
  float confidence_threshold,
  float nms_iou_threshold);

bool is_supported_detection_output_shape(
  const std::vector<std::size_t> & shape,
  DetectionLabelSet label_set);

}  // namespace fruit_perception

#endif  // FRUIT_PERCEPTION__DETECTION_PARSER_HPP_
