#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include <vector>

#include "fruit_perception/detection_parser.hpp"
#include "fruit_perception/image_processing.hpp"

TEST(ImageProcessing, LetterboxPreservesEntireFourByThreeImage)
{
  const cv::Mat image(480, 640, CV_8UC3, cv::Scalar(10, 20, 30));
  std::vector<float> input(3U * 640U * 640U);

  const auto transform = fruit_perception::letterbox_to_nchw(image, input);

  EXPECT_DOUBLE_EQ(transform.scale, 1.0);
  EXPECT_EQ(transform.pad_x, 0);
  EXPECT_EQ(transform.pad_y, 80);
  EXPECT_EQ(transform.original_width, 640);
  EXPECT_EQ(transform.original_height, 480);
  EXPECT_NEAR(input[0], 114.0F / 255.0F, 1e-6F);
  EXPECT_NEAR(input[80U * 640U], 30.0F / 255.0F, 1e-6F);
  EXPECT_NEAR(input[640U * 640U + 80U * 640U], 20.0F / 255.0F, 1e-6F);
  EXPECT_NEAR(input[2U * 640U * 640U + 80U * 640U], 10.0F / 255.0F, 1e-6F);
}

TEST(ImageProcessing, MapsLetterboxBoxBackToOriginalImage)
{
  const fruit_perception::LetterboxTransform transform{1.0, 0, 80, 640, 480};
  const fruit_perception::Detection detection{
    160.0F, 160.0F, 480.0F, 480.0F, 0.9F, "apple_ripe"};

  const auto box = fruit_perception::to_original_box(detection, transform);

  EXPECT_EQ(box, cv::Rect(160, 80, 320, 320));
}

TEST(ImageProcessing, DrawsDetectionLabelAndFrameRateOnOriginalImage)
{
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  const fruit_perception::LetterboxTransform transform{1.0, 0, 80, 640, 480};
  const std::vector<fruit_perception::Detection> detections{
    {160.0F, 160.0F, 480.0F, 480.0F, 0.9F, "apple_ripe"}};

  fruit_perception::draw_annotations(image, detections, transform, 18.5);

  EXPECT_GT(cv::countNonZero(image.reshape(1)), 0);
}
