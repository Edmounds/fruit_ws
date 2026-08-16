#ifndef FRUIT_PERCEPTION__DETECTION_OUTPUT_HPP_
#define FRUIT_PERCEPTION__DETECTION_OUTPUT_HPP_

#include <opencv2/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <string>
#include <vector>

#include "fruit_perception/detection_parser.hpp"
#include "fruit_perception/image_processing.hpp"

namespace fruit_perception
{

struct DetectionOutput
{
  std::string type;
  float confidence;
  cv::Rect roi;
  bool has_point;
  cv::Point3d point;
};

DetectionOutput make_detection_output(
  const Detection & detection,
  const LetterboxTransform & transform,
  const cv::Mat & aligned_depth,
  const sensor_msgs::msg::CameraInfo & camera_info);

std::vector<DetectionOutput> make_detection_outputs(
  const std::vector<Detection> & detections,
  const LetterboxTransform & transform,
  const cv::Mat & aligned_depth,
  const sensor_msgs::msg::CameraInfo & camera_info);

}  // namespace fruit_perception

#endif  // FRUIT_PERCEPTION__DETECTION_OUTPUT_HPP_
