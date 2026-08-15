#ifndef FRUIT_PERCEPTION__TARGET_LOCALIZATION_HPP_
#define FRUIT_PERCEPTION__TARGET_LOCALIZATION_HPP_

#include <opencv2/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <optional>

#include "fruit_perception/detection_parser.hpp"
#include "fruit_perception/image_processing.hpp"

namespace fruit_perception
{

std::optional<cv::Point3d> localize_detection_center(
  const Detection & detection,
  const LetterboxTransform & transform,
  const cv::Mat & aligned_depth,
  const sensor_msgs::msg::CameraInfo & camera_info);

}  // namespace fruit_perception

#endif  // FRUIT_PERCEPTION__TARGET_LOCALIZATION_HPP_
