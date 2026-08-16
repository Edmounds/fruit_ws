#include "fruit_perception/detection_output.hpp"

#include "fruit_perception/target_localization.hpp"

namespace fruit_perception
{

DetectionOutput make_detection_output(
  const Detection & detection,
  const LetterboxTransform & transform,
  const cv::Mat & aligned_depth,
  const sensor_msgs::msg::CameraInfo & camera_info)
{
  DetectionOutput output{
    detection.type, detection.score, to_original_box(detection, transform), false, {}};
  const auto point = localize_detection_center(
    detection, transform, aligned_depth, camera_info);
  if (point) {
    output.has_point = true;
    output.point = *point;
  }
  return output;
}

std::vector<DetectionOutput> make_detection_outputs(
  const std::vector<Detection> & detections,
  const LetterboxTransform & transform,
  const cv::Mat & aligned_depth,
  const sensor_msgs::msg::CameraInfo & camera_info)
{
  std::vector<DetectionOutput> outputs;
  outputs.reserve(detections.size());
  for (const auto & detection : detections) {
    outputs.push_back(make_detection_output(detection, transform, aligned_depth, camera_info));
  }
  return outputs;
}

}  // namespace fruit_perception
