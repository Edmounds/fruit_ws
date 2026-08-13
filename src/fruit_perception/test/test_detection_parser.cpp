#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "fruit_perception/detection_parser.hpp"

namespace
{

constexpr std::size_t kRows = 300;
constexpr std::size_t kColumns = 6;
using Output = std::array<float, kRows * kColumns>;

void set_detection(Output & output, std::size_t row, float score, float class_id)
{
  output[row * kColumns] = 10.0F;
  output[row * kColumns + 1] = 20.0F;
  output[row * kColumns + 2] = 110.0F;
  output[row * kColumns + 3] = 220.0F;
  output[row * kColumns + 4] = score;
  output[row * kColumns + 5] = class_id;
}

std::vector<std::string> parse(const Output & output, float threshold = 0.5F)
{
  return fruit_perception::parse_fruit_types(
    output.data(), {1, kRows, kColumns}, threshold);
}

TEST(DetectionParser, ReturnsEmptyTypesForPaddedOutput)
{
  const Output output{};
  EXPECT_TRUE(parse(output).empty());
}

TEST(DetectionParser, FiltersScoresBelowRuntimeThreshold)
{
  Output output{};
  set_detection(output, 0, 0.49F, 0.0F);
  set_detection(output, 1, 0.50F, 1.0F);
  EXPECT_EQ(parse(output), (std::vector<std::string>{"apple_unripe"}));
}

TEST(DetectionParser, MapsAllTwelveClassIndices)
{
  Output output{};
  for (std::size_t class_id = 0; class_id < 12; ++class_id) {
    set_detection(output, class_id, 0.9F, static_cast<float>(class_id));
  }
  EXPECT_EQ(
    parse(output),
    (std::vector<std::string>{
      "apple_ripe", "apple_unripe", "chili_ripe", "chili_unripe",
      "pumpkin_ripe", "pumpkin_unripe", "onion_ripe", "onion_unripe",
      "pear_ripe", "pear_unripe", "tomato_ripe", "tomato_unripe"}));
}

TEST(DetectionParser, PreservesDuplicateLabels)
{
  Output output{};
  set_detection(output, 0, 0.9F, 2.0F);
  set_detection(output, 1, 0.8F, 2.0F);
  EXPECT_EQ(
    parse(output),
    (std::vector<std::string>{"chili_ripe", "chili_ripe"}));
}

TEST(DetectionParser, SortsLabelsByDescendingConfidence)
{
  Output output{};
  set_detection(output, 0, 0.6F, 8.0F);
  set_detection(output, 1, 0.95F, 3.0F);
  set_detection(output, 2, 0.75F, 0.0F);
  EXPECT_EQ(
    parse(output),
    (std::vector<std::string>{"chili_unripe", "apple_ripe", "pear_ripe"}));
}

TEST(DetectionParser, ReturnsBoxScoreAndTypeForAnnotation)
{
  Output output{};
  set_detection(output, 0, 0.75F, 8.0F);

  const auto detections = fruit_perception::parse_detections(
    output.data(), {1, kRows, kColumns}, 0.5F);

  ASSERT_EQ(detections.size(), 1U);
  EXPECT_FLOAT_EQ(detections[0].x1, 10.0F);
  EXPECT_FLOAT_EQ(detections[0].y1, 20.0F);
  EXPECT_FLOAT_EQ(detections[0].x2, 110.0F);
  EXPECT_FLOAT_EQ(detections[0].y2, 220.0F);
  EXPECT_FLOAT_EQ(detections[0].score, 0.75F);
  EXPECT_EQ(detections[0].type, "pear_ripe");
}

TEST(DetectionParser, IgnoresDetectionWithNonFiniteBoxCoordinate)
{
  Output output{};
  set_detection(output, 0, 0.9F, 0.0F);
  output[0] = std::numeric_limits<float>::quiet_NaN();

  const auto detections = fruit_perception::parse_detections(
    output.data(), {1, kRows, kColumns}, 0.5F);

  EXPECT_TRUE(detections.empty());
}

TEST(DetectionParser, RejectsUnexpectedOutputShape)
{
  Output output{};
  EXPECT_THROW(
    fruit_perception::parse_fruit_types(output.data(), {1, 84, 8400}, 0.5F),
    std::invalid_argument);
}

TEST(DetectionParser, IgnoresClassIndexOutsideTwelveClassBoundary)
{
  Output output{};
  set_detection(output, 0, 0.9F, 11.0F);
  set_detection(output, 1, 0.8F, 12.0F);
  EXPECT_EQ(parse(output), (std::vector<std::string>{"tomato_unripe"}));
}

TEST(DetectionParser, RejectsNonFiniteConfidenceThreshold)
{
  const Output output{};
  EXPECT_THROW(parse(output, std::numeric_limits<float>::quiet_NaN()), std::invalid_argument);
}

TEST(DetectionParser, IgnoresDetectionWithNonFiniteScore)
{
  Output output{};
  set_detection(output, 0, std::numeric_limits<float>::quiet_NaN(), 0.0F);
  set_detection(output, 1, 0.8F, 1.0F);
  EXPECT_EQ(parse(output), (std::vector<std::string>{"apple_unripe"}));
}

}  // namespace
