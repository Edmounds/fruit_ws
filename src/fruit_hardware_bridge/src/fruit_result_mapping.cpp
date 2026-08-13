#include "fruit_hardware_bridge/fruit_result_mapping.hpp"

#include <array>
#include <string_view>
#include <utility>

namespace fruit_hardware_bridge
{
namespace
{

constexpr std::array<std::pair<std::string_view, EncodedDetection>, 12> kFruitResults{{
  {"apple_ripe", {1, 1}}, {"apple_unripe", {1, 0}},
  {"chili_ripe", {2, 1}}, {"chili_unripe", {2, 0}},
  {"pumpkin_ripe", {3, 1}}, {"pumpkin_unripe", {3, 0}},
  {"onion_ripe", {4, 1}}, {"onion_unripe", {4, 0}},
  {"pear_ripe", {5, 1}}, {"pear_unripe", {5, 0}},
  {"tomato_ripe", {6, 1}}, {"tomato_unripe", {6, 0}},
}};

}  // namespace

EncodedDetection encode_detection(const std::vector<std::string> & types)
{
  if (types.empty()) {
    return {0, 0};
  }

  for (const auto & [label, result] : kFruitResults) {
    if (label == types.front()) {
      return result;
    }
  }
  return {0, 0};
}

}  // namespace fruit_hardware_bridge
