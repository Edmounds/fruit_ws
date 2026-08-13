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

// Letterboxes `source` into the pre-allocated continuous 640x640 8UC3 `letterboxed`
// buffer, filling the padding with 114. The buffer can be reused across frames;
// only the regions outside the resized image are rewritten with padding.
LetterboxTransform letterbox_image(const cv::Mat & source, cv::Mat & letterboxed);

cv::Rect to_original_box(
  const Detection & detection,
  const LetterboxTransform & transform);

void draw_annotations(
  cv::Mat & image,
  const std::vector<Detection> & detections,
  const LetterboxTransform & transform,
  double frames_per_second);

}  // namespace fruit_perception

#endif  // FRUIT_PERCEPTION__IMAGE_PROCESSING_HPP_
