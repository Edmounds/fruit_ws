#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include <stdexcept>
#include <vector>

#include "fruit_perception/detection_parser.hpp"
#include "fruit_perception/image_processing.hpp"

TEST(ImageProcessing, LetterboxPreservesEntireFourByThreeImage)
{
  const cv::Mat image(480, 640, CV_8UC3, cv::Scalar(10, 20, 30));
  cv::Mat letterboxed(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));

  const auto transform = fruit_perception::letterbox_image(image, letterboxed);

  EXPECT_DOUBLE_EQ(transform.scale, 1.0);
  EXPECT_EQ(transform.pad_x, 0);
  EXPECT_EQ(transform.pad_y, 80);
  EXPECT_EQ(transform.original_width, 640);
  EXPECT_EQ(transform.original_height, 480);
  EXPECT_EQ(letterboxed.at<cv::Vec3b>(0, 0), cv::Vec3b(114, 114, 114));
  EXPECT_EQ(letterboxed.at<cv::Vec3b>(79, 639), cv::Vec3b(114, 114, 114));
  EXPECT_EQ(letterboxed.at<cv::Vec3b>(80, 0), cv::Vec3b(10, 20, 30));
  EXPECT_EQ(letterboxed.at<cv::Vec3b>(559, 639), cv::Vec3b(10, 20, 30));
  EXPECT_EQ(letterboxed.at<cv::Vec3b>(560, 0), cv::Vec3b(114, 114, 114));
}

TEST(ImageProcessing, LetterboxRewritesPaddingWhenBufferIsReused)
{
  cv::Mat letterboxed(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  const cv::Mat first(480, 640, CV_8UC3, cv::Scalar(50, 60, 70));
  fruit_perception::letterbox_image(first, letterboxed);

  const cv::Mat second(400, 200, CV_8UC3, cv::Scalar(1, 2, 3));
  const auto transform = fruit_perception::letterbox_image(second, letterboxed);

  EXPECT_DOUBLE_EQ(transform.scale, 1.6);
  EXPECT_EQ(transform.pad_x, 160);
  EXPECT_EQ(transform.pad_y, 0);
  EXPECT_EQ(letterboxed.at<cv::Vec3b>(100, 0), cv::Vec3b(114, 114, 114));
  EXPECT_EQ(letterboxed.at<cv::Vec3b>(100, 320), cv::Vec3b(1, 2, 3));
  EXPECT_EQ(letterboxed.at<cv::Vec3b>(100, 500), cv::Vec3b(114, 114, 114));
}

TEST(ImageProcessing, LetterboxRejectsWrongTargetBuffer)
{
  const cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat wrong_size(320, 320, CV_8UC3, cv::Scalar(0, 0, 0));

  EXPECT_THROW(
    fruit_perception::letterbox_image(image, wrong_size), std::invalid_argument);
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
