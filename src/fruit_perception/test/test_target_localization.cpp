#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <cstdint>
#include <stdexcept>

#include "fruit_perception/detection_parser.hpp"
#include "fruit_perception/image_processing.hpp"
#include "fruit_perception/target_localization.hpp"

namespace
{

sensor_msgs::msg::CameraInfo make_camera_info()
{
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.width = 640;
  camera_info.height = 480;
  camera_info.k = {600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0};
  return camera_info;
}

TEST(TargetLocalization, ProjectsDetectionCenterDepthIntoOpticalFrameMeters)
{
  cv::Mat depth(480, 640, CV_16UC1, cv::Scalar(0));
  depth.at<std::uint16_t>(270, 380) = 2000U;
  const fruit_perception::Detection detection{
    360.0F, 330.0F, 400.0F, 370.0F, 0.9F, "apple_ripe"};
  const fruit_perception::LetterboxTransform transform{1.0, 0, 80, 640, 480};

  const auto point = fruit_perception::localize_detection_center(
    detection, transform, depth, make_camera_info());

  ASSERT_TRUE(point.has_value());
  EXPECT_NEAR(point->x, 0.2, 1e-9);
  EXPECT_NEAR(point->y, 0.1, 1e-9);
  EXPECT_NEAR(point->z, 2.0, 1e-9);
}

TEST(TargetLocalization, ReturnsNoPointWhenCenterDepthIsInvalid)
{
  const cv::Mat depth(480, 640, CV_16UC1, cv::Scalar(0));
  const fruit_perception::Detection detection{
    300.0F, 300.0F, 340.0F, 340.0F, 0.9F, "apple_ripe"};
  const fruit_perception::LetterboxTransform transform{1.0, 0, 80, 640, 480};

  const auto point = fruit_perception::localize_detection_center(
    detection, transform, depth, make_camera_info());

  EXPECT_FALSE(point.has_value());
}

TEST(TargetLocalization, RejectsDepthThatIsNotAlignedToTheColorImage)
{
  const cv::Mat depth(240, 320, CV_16UC1, cv::Scalar(1000));
  const fruit_perception::Detection detection{
    300.0F, 300.0F, 340.0F, 340.0F, 0.9F, "apple_ripe"};
  const fruit_perception::LetterboxTransform transform{1.0, 0, 80, 640, 480};

  EXPECT_THROW(
    fruit_perception::localize_detection_center(
      detection, transform, depth, make_camera_info()),
    std::invalid_argument);
}

}  // namespace
