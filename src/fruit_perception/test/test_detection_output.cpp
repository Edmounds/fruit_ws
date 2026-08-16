#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "fruit_perception/detection_output.hpp"

namespace
{
sensor_msgs::msg::CameraInfo make_camera_info()
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 1920;
  info.height = 1080;
  info.k = {1000.0, 0.0, 960.0, 0.0, 1000.0, 540.0, 0.0, 0.0, 1.0};
  return info;
}

TEST(DetectionOutput, MapsRoiToSourceAndCreatesOpticalPoint)
{
  const fruit_perception::LetterboxTransform transform{1.0 / 3.0, 0, 140, 1920, 1080};
  const fruit_perception::Detection detection{300.0F, 290.0F, 340.0F, 330.0F, 0.91F, "apple"};
  cv::Mat depth(1080, 1920, CV_16UC1, cv::Scalar(0));
  depth.at<std::uint16_t>(510, 960) = 1500;

  const auto output = fruit_perception::make_detection_output(
    detection, transform, depth, make_camera_info());

  EXPECT_EQ(output.type, "apple");
  EXPECT_FLOAT_EQ(output.confidence, 0.91F);
  EXPECT_EQ(output.roi.x, 900);
  EXPECT_EQ(output.roi.y, 450);
  EXPECT_EQ(output.roi.width, 120);
  EXPECT_EQ(output.roi.height, 120);
  EXPECT_TRUE(output.has_point);
  EXPECT_DOUBLE_EQ(output.point.x, 0.0);
  EXPECT_DOUBLE_EQ(output.point.y, -0.045);
  EXPECT_DOUBLE_EQ(output.point.z, 1.5);
}

TEST(DetectionOutput, KeepsTwoDimensionalDetectionWhenDepthIsInvalid)
{
  const fruit_perception::LetterboxTransform transform{1.0 / 3.0, 0, 140, 1920, 1080};
  const fruit_perception::Detection detection{300.0F, 290.0F, 340.0F, 330.0F, 0.82F, "pear"};
  const cv::Mat depth(1080, 1920, CV_16UC1, cv::Scalar(0));

  const auto output = fruit_perception::make_detection_output(
    detection, transform, depth, make_camera_info());

  EXPECT_EQ(output.type, "pear");
  EXPECT_FALSE(output.has_point);
  EXPECT_EQ(output.roi.width, 120);
}

TEST(DetectionOutput, PreservesEveryDetectionAndItsConfidenceOrder)
{
  const fruit_perception::LetterboxTransform transform{1.0 / 3.0, 0, 140, 1920, 1080};
  const std::vector<fruit_perception::Detection> detections{
    {300.0F, 290.0F, 340.0F, 330.0F, 0.91F, "apple"},
    {100.0F, 200.0F, 140.0F, 240.0F, 0.73F, "pear"}};
  const cv::Mat depth(1080, 1920, CV_16UC1, cv::Scalar(0));

  const auto outputs = fruit_perception::make_detection_outputs(
    detections, transform, depth, make_camera_info());

  ASSERT_EQ(outputs.size(), 2U);
  EXPECT_EQ(outputs[0].type, "apple");
  EXPECT_FLOAT_EQ(outputs[0].confidence, 0.91F);
  EXPECT_EQ(outputs[1].type, "pear");
  EXPECT_FLOAT_EQ(outputs[1].confidence, 0.73F);
}
}  // namespace
