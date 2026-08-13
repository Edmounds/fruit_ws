#include "fruit_perception/image_processing.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace fruit_perception
{

LetterboxTransform letterbox_to_nchw(const cv::Mat & bgr, std::vector<float> & input)
{
  constexpr std::size_t kInputElements =
    3U * static_cast<std::size_t>(kModelInputSize) * kModelInputSize;
  if (bgr.empty() || bgr.type() != CV_8UC3) {
    throw std::invalid_argument("letterbox input must be a non-empty BGR8 image");
  }
  if (input.size() != kInputElements) {
    throw std::invalid_argument("model input buffer must contain 3*640*640 floats");
  }

  const double scale = std::min(
    static_cast<double>(kModelInputSize) / bgr.cols,
    static_cast<double>(kModelInputSize) / bgr.rows);
  const int resized_width = static_cast<int>(std::round(bgr.cols * scale));
  const int resized_height = static_cast<int>(std::round(bgr.rows * scale));
  const int pad_x = (kModelInputSize - resized_width) / 2;
  const int pad_y = (kModelInputSize - resized_height) / 2;

  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(resized_width, resized_height));
  cv::Mat letterboxed(
    kModelInputSize, kModelInputSize, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(letterboxed(cv::Rect(pad_x, pad_y, resized_width, resized_height)));

  const std::size_t plane_size =
    static_cast<std::size_t>(kModelInputSize) * kModelInputSize;
  for (int row = 0; row < kModelInputSize; ++row) {
    const auto * pixels = letterboxed.ptr<cv::Vec3b>(row);
    for (int column = 0; column < kModelInputSize; ++column) {
      const auto index = static_cast<std::size_t>(row * kModelInputSize + column);
      input[index] = static_cast<float>(pixels[column][2]) / 255.0F;
      input[plane_size + index] = static_cast<float>(pixels[column][1]) / 255.0F;
      input[2 * plane_size + index] = static_cast<float>(pixels[column][0]) / 255.0F;
    }
  }

  return LetterboxTransform{scale, pad_x, pad_y, bgr.cols, bgr.rows};
}

cv::Rect to_original_box(
  const Detection & detection,
  const LetterboxTransform & transform)
{
  const auto map_x = [&transform](float x) {
      return (static_cast<double>(x) - transform.pad_x) / transform.scale;
    };
  const auto map_y = [&transform](float y) {
      return (static_cast<double>(y) - transform.pad_y) / transform.scale;
    };

  const int x1 = std::clamp(
    static_cast<int>(std::floor(map_x(detection.x1))), 0, transform.original_width);
  const int y1 = std::clamp(
    static_cast<int>(std::floor(map_y(detection.y1))), 0, transform.original_height);
  const int x2 = std::clamp(
    static_cast<int>(std::ceil(map_x(detection.x2))), 0, transform.original_width);
  const int y2 = std::clamp(
    static_cast<int>(std::ceil(map_y(detection.y2))), 0, transform.original_height);
  return cv::Rect(x1, y1, std::max(0, x2 - x1), std::max(0, y2 - y1));
}

void draw_annotations(
  cv::Mat & bgr,
  const std::vector<Detection> & detections,
  const LetterboxTransform & transform,
  double frames_per_second)
{
  const cv::Scalar color(0, 255, 0);
  for (const auto & detection : detections) {
    const auto box = to_original_box(detection, transform);
    if (box.empty()) {
      continue;
    }
    cv::rectangle(bgr, box, color, 2);

    std::ostringstream label;
    label << detection.type << ' ' << std::fixed << std::setprecision(2) << detection.score;
    int baseline = 0;
    const auto text_size = cv::getTextSize(
      label.str(), cv::FONT_HERSHEY_SIMPLEX, 0.55, 1, &baseline);
    const int text_x = std::clamp(box.x, 0, std::max(0, bgr.cols - text_size.width - 6));
    const int text_y = std::max(box.y - 6, text_size.height + 4);
    cv::rectangle(
      bgr,
      cv::Rect(
        text_x, text_y - text_size.height - 4,
        text_size.width + 6, text_size.height + baseline + 6),
      color, cv::FILLED);
    cv::putText(
      bgr, label.str(), cv::Point(text_x + 3, text_y),
      cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
  }

  std::ostringstream fps_label;
  fps_label << "FPS: " << std::fixed << std::setprecision(1) << frames_per_second;
  cv::putText(
    bgr, fps_label.str(), cv::Point(10, 28),
    cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
  cv::putText(
    bgr, fps_label.str(), cv::Point(10, 28),
    cv::FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv::LINE_AA);
}

}  // namespace fruit_perception
