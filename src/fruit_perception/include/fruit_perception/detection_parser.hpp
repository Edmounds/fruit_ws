#ifndef FRUIT_PERCEPTION__DETECTION_PARSER_HPP_
#define FRUIT_PERCEPTION__DETECTION_PARSER_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace fruit_perception
{

struct Detection
{
  float x1;
  float y1;
  float x2;
  float y2;
  float score;
  std::string type;
};

std::vector<Detection> parse_detections(
  const float * output,
  const std::vector<std::size_t> & shape,
  float confidence_threshold,
  float nms_iou_threshold);

std::vector<Detection> parse_detections(
  const float * output,
  const std::vector<std::size_t> & shape,
  float confidence_threshold);

std::vector<std::string> parse_fruit_types(
  const float * output,
  const std::vector<std::size_t> & shape,
  float confidence_threshold,
  float nms_iou_threshold);

std::vector<std::string> parse_fruit_types(
  const float * output,
  const std::vector<std::size_t> & shape,
  float confidence_threshold);

bool is_supported_detection_output_shape(const std::vector<std::size_t> & shape);

}  // namespace fruit_perception

#endif  // FRUIT_PERCEPTION__DETECTION_PARSER_HPP_
