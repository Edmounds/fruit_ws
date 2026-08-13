#ifndef FRUIT_PERCEPTION__IMAGE_PROCESSING_HPP_
#define FRUIT_PERCEPTION__IMAGE_PROCESSING_HPP_

#include <opencv2/core.hpp>

#include <vector>

#include "fruit_perception/detection_parser.hpp"

namespace fruit_perception
{

constexpr int kModelInputSize = 640;

struct LetterboxTransform
{
  double scale;
  int pad_x;
  int pad_y;
  int original_width;
  int original_height;
};

LetterboxTransform letterbox_to_nchw(const cv::Mat & bgr, std::vector<float> & input);

cv::Rect to_original_box(
  const Detection & detection,
  const LetterboxTransform & transform);

void draw_annotations(
  cv::Mat & bgr,
  const std::vector<Detection> & detections,
  const LetterboxTransform & transform,
  double frames_per_second);

}  // namespace fruit_perception

#endif  // FRUIT_PERCEPTION__IMAGE_PROCESSING_HPP_
