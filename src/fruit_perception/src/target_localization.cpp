#include "fruit_perception/target_localization.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace fruit_perception
{

std::optional<cv::Point3d> localize_detection_center(
  const Detection & detection,
  const LetterboxTransform & transform,
  const cv::Mat & aligned_depth,
  const sensor_msgs::msg::CameraInfo & camera_info)
{
  if (aligned_depth.type() != CV_16UC1 ||
    aligned_depth.cols != transform.original_width ||
    aligned_depth.rows != transform.original_height ||
    camera_info.width != static_cast<std::uint32_t>(aligned_depth.cols) ||
    camera_info.height != static_cast<std::uint32_t>(aligned_depth.rows))
  {
    throw std::invalid_argument("depth and camera info must align with the color image");
  }

  const double fx = camera_info.k[0];
  const double fy = camera_info.k[4];
  const double cx = camera_info.k[2];
  const double cy = camera_info.k[5];
  if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0.0 || fy <= 0.0) {
    throw std::invalid_argument("camera focal lengths must be positive and finite");
  }

  const cv::Rect box = to_original_box(detection, transform);
  if (box.empty()) {
    return std::nullopt;
  }

  const int u = box.x + box.width / 2;
  const int v = box.y + box.height / 2;
  const std::uint16_t depth_mm = aligned_depth.at<std::uint16_t>(v, u);
  if (depth_mm == 0U) {
    return std::nullopt;
  }

  const double z = static_cast<double>(depth_mm) * 0.001;
  return cv::Point3d((u - cx) * z / fx, (v - cy) * z / fy, z);
}

}  // namespace fruit_perception
