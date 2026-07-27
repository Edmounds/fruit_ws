#include <exception>
#include <iostream>
#include <string>

#include "pb_glim_map_tools/map_conversion.hpp"

namespace
{

void printUsage()
{
  std::cerr << "Usage: glim_ply_to_pcd --input MAP.ply --output MAP.pcd "
               "[--leaf-size 0.10]\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string input;
  std::string output;
  double leaf_size = 0.10;

  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if ((arg == "--input" || arg == "--output" || arg == "--leaf-size") && i + 1 >= argc) {
      printUsage();
      return 2;
    }
    if (arg == "--input") {
      input = argv[++i];
    } else if (arg == "--output") {
      output = argv[++i];
    } else if (arg == "--leaf-size") {
      try {
        leaf_size = std::stod(argv[++i]);
      } catch (const std::exception &) {
        std::cerr << "Invalid --leaf-size value\n";
        return 2;
      }
    } else if (arg == "--help" || arg == "-h") {
      printUsage();
      return 0;
    } else {
      std::cerr << "Unknown argument: " << arg << '\n';
      printUsage();
      return 2;
    }
  }

  if (input.empty() || output.empty()) {
    printUsage();
    return 2;
  }

  pb_glim_map_tools::ConversionResult result;
  std::string error;
  if (!pb_glim_map_tools::convertMap(input, output, leaf_size, result, error)) {
    std::cerr << "Conversion failed: " << error << '\n';
    return 1;
  }

  std::cout << "Input points: " << result.input_points << '\n'
            << "Finite points: " << result.finite_points << '\n'
            << "Output points: " << result.output_points << '\n'
            << "Map frame bounding box: min=[" << result.min.x << ", " << result.min.y << ", "
            << result.min.z << "] max=[" << result.max.x << ", " << result.max.y << ", "
            << result.max.z << "]\n";
  return 0;
}
