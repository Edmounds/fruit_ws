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

LetterboxTransform letterbox_image(const cv::Mat & source, cv::Mat & letterboxed)
{
  if (source.empty() || source.type() != CV_8UC3) {
    throw std::invalid_argument("letterbox input must be a non-empty 8UC3 image");
  }
  if (letterboxed.rows != kModelInputSize || letterboxed.cols != kModelInputSize ||
    letterboxed.type() != CV_8UC3 || !letterboxed.isContinuous())
  {
    throw std::invalid_argument(
      "letterbox target must be a continuous 640x640 8UC3 buffer");
  }

  const double scale = std::min(
    static_cast<double>(kModelInputSize) / source.cols,
    static_cast<double>(kModelInputSize) / source.rows);
  const int resized_width = static_cast<int>(std::round(source.cols * scale));
  const int resized_height = static_cast<int>(std::round(source.rows * scale));
  const int pad_x = (kModelInputSize - resized_width) / 2;
  const int pad_y = (kModelInputSize - resized_height) / 2;

  const cv::Rect roi(pad_x, pad_y, resized_width, resized_height);
  cv::Mat destination = letterboxed(roi);
  cv::resize(source, destination, roi.size());

  const cv::Scalar pad_color(114, 114, 114);
  const int roi_bottom = pad_y + resized_height;
  const int roi_right = pad_x + resized_width;
  if (pad_y > 0) {
    letterboxed(cv::Rect(0, 0, kModelInputSize, pad_y)) = pad_color;
  }
  if (roi_bottom < kModelInputSize) {
    letterboxed(
      cv::Rect(0, roi_bottom, kModelInputSize, kModelInputSize - roi_bottom)) = pad_color;
  }
  if (pad_x > 0) {
    letterboxed(cv::Rect(0, pad_y, pad_x, resized_height)) = pad_color;
  }
  if (roi_right < kModelInputSize) {
    letterboxed(
      cv::Rect(roi_right, pad_y, kModelInputSize - roi_right, resized_height)) = pad_color;
  }

  return LetterboxTransform{scale, pad_x, pad_y, source.cols, source.rows};
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
  cv::Mat & image,
  const std::vector<Detection> & detections,
  const LetterboxTransform & transform,
  double frames_per_second)
{
  // The annotated image is RGB8, not OpenCV's usual BGR.
  const cv::Scalar color(0, 255, 0);
  for (const auto & detection : detections) {
    const auto box = to_original_box(detection, transform);
    if (box.empty()) {
      continue;
    }
    const cv::Scalar box_color = detection.maturity == "unripe" ?
      cv::Scalar(255, 165, 0) : color;
    cv::rectangle(image, box, box_color, 2);
    const cv::Point center(box.x + box.width / 2, box.y + box.height / 2);
    const cv::Scalar center_color(255, 220, 0);
    cv::circle(image, center, 4, center_color, -1);
    cv::line(image, center + cv::Point(-8, 0), center + cv::Point(8, 0), center_color, 1);
    cv::line(image, center + cv::Point(0, -8), center + cv::Point(0, 8), center_color, 1);

    std::ostringstream label;
    label << (detection.species.empty() ? detection.type : detection.species);
    if (!detection.maturity.empty()) {
      label << " [" << (detection.maturity == "ripe" ? "RIPE" : "UNRIPE") << ' '
            << std::fixed << std::setprecision(0) << detection.maturity_confidence * 100.0
            << "%]";
    }
    label << ' ' << std::fixed << std::setprecision(2) << detection.score;
    if (std::isfinite(detection.depth_m) && detection.depth_m > 0.0) {
      label << " | D=" << std::setprecision(3) << detection.depth_m << 'm';
    } else {
      label << " | D: NoDepth";
    }
    int baseline = 0;
    const auto text_size = cv::getTextSize(
      label.str(), cv::FONT_HERSHEY_SIMPLEX, 0.55, 1, &baseline);
    const int text_x = std::clamp(box.x, 0, std::max(0, image.cols - text_size.width - 6));
    const int text_y = std::max(box.y - 6, text_size.height + 4);
    cv::rectangle(
      image,
      cv::Rect(
        text_x, text_y - text_size.height - 4,
        text_size.width + 6, text_size.height + baseline + 6),
      box_color, cv::FILLED);
    cv::putText(
      image, label.str(), cv::Point(text_x + 3, text_y),
      cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
  }

  std::ostringstream fps_label;
  fps_label << "FPS: " << std::fixed << std::setprecision(1) << frames_per_second;
  cv::putText(
    image, fps_label.str(), cv::Point(10, 28),
    cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
  cv::putText(
    image, fps_label.str(), cv::Point(10, 28),
    cv::FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv::LINE_AA);
}

}  // namespace fruit_perception
