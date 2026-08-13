#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace fruit_hardware_bridge
{

using EncodedDetection = std::array<std::uint8_t, 2>;

EncodedDetection encode_detection(const std::vector<std::string> & types);

}  // namespace fruit_hardware_bridge
