#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "fruit_hardware_bridge/fruit_result_mapping.hpp"

namespace
{

using Result = std::array<std::uint8_t, 2>;

TEST(FruitResultMappingTest, MapsAllModelClasses)
{
  const std::vector<std::pair<std::string, Result>> cases{
    {"apple_ripe", {1, 1}}, {"apple_unripe", {1, 0}},
    {"chili_ripe", {2, 1}}, {"chili_unripe", {2, 0}},
    {"pumpkin_ripe", {3, 1}}, {"pumpkin_unripe", {3, 0}},
    {"onion_ripe", {4, 1}}, {"onion_unripe", {4, 0}},
    {"pear_ripe", {5, 1}}, {"pear_unripe", {5, 0}},
    {"tomato_ripe", {6, 1}}, {"tomato_unripe", {6, 0}},
  };

  for (const auto & [label, expected] : cases) {
    EXPECT_EQ(fruit_hardware_bridge::encode_detection({label}), expected) << label;
  }
}

TEST(FruitResultMappingTest, UsesHighestConfidenceDetection)
{
  EXPECT_EQ(
    fruit_hardware_bridge::encode_detection({"pumpkin_ripe", "apple_unripe"}),
    (Result{3, 1}));
}

TEST(FruitResultMappingTest, EmptyDetectionClearsPreviousResult)
{
  EXPECT_EQ(fruit_hardware_bridge::encode_detection({}), (Result{0, 0}));
}

TEST(FruitResultMappingTest, UnknownDetectionClearsPreviousResult)
{
  EXPECT_EQ(fruit_hardware_bridge::encode_detection({"unknown"}), (Result{0, 0}));
}

}  // namespace
