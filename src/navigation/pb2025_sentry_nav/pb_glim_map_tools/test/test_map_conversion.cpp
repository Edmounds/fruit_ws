#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "pb_glim_map_tools/map_conversion.hpp"

namespace fs = std::filesystem;

TEST(MapConversion, RemovesNanAndDownsamples)
{
  const auto dir = fs::temp_directory_path() / "pb_glim_map_tools_test";
  fs::create_directories(dir);
  const auto input = dir / "map.ply";
  const auto output = dir / "map.pcd";

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back({0.01F, 0.01F, 0.01F});
  cloud.push_back({0.02F, 0.02F, 0.02F});
  cloud.push_back({1.0F, 2.0F, 3.0F});
  cloud.push_back({std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F});
  cloud.is_dense = false;
  ASSERT_EQ(pcl::io::savePLYFileBinary(input.string(), cloud), 0);

  pb_glim_map_tools::ConversionResult result;
  std::string error;
  ASSERT_TRUE(pb_glim_map_tools::convertMap(input.string(), output.string(), 0.1, result, error))
    << error;
  EXPECT_EQ(result.input_points, 4U);
  EXPECT_EQ(result.finite_points, 3U);
  EXPECT_EQ(result.output_points, 2U);

  pcl::PointCloud<pcl::PointXYZ> saved;
  EXPECT_EQ(pcl::io::loadPCDFile(output.string(), saved), 0);
  EXPECT_EQ(saved.size(), 2U);
}

TEST(MapConversion, RejectsInvalidInput)
{
  pb_glim_map_tools::ConversionResult result;
  std::string error;
  EXPECT_FALSE(pb_glim_map_tools::convertMap("missing.ply", "unused.pcd", 0.1, result, error));
  EXPECT_FALSE(pb_glim_map_tools::convertMap("missing.ply", "unused.pcd", 0.0, result, error));
  EXPECT_FALSE(pb_glim_map_tools::convertMap(
    "missing.ply", "unused.pcd", std::numeric_limits<double>::quiet_NaN(), result, error));
}

TEST(MapConversion, RejectsEmptyMapAndUnwritableOutput)
{
  const auto dir = fs::temp_directory_path() / "pb_glim_map_tools_empty_test";
  fs::create_directories(dir);
  const auto empty_input = dir / "empty.ply";
  std::ofstream empty_ply(empty_input);
  empty_ply << "ply\nformat ascii 1.0\nelement vertex 0\n"
               "property float x\nproperty float y\nproperty float z\nend_header\n";
  empty_ply.close();
  pb_glim_map_tools::ConversionResult result;
  std::string error;
  EXPECT_FALSE(pb_glim_map_tools::convertMap(
    empty_input.string(), (dir / "empty.pcd").string(), 0.1, result, error));

  pcl::PointCloud<pcl::PointXYZ> valid;
  valid.push_back({1.0F, 2.0F, 3.0F});
  const auto valid_input = dir / "valid.ply";
  ASSERT_EQ(pcl::io::savePLYFileBinary(valid_input.string(), valid), 0);
  EXPECT_FALSE(pb_glim_map_tools::convertMap(
    valid_input.string(), (dir / "missing" / "map.pcd").string(), 0.1, result, error));
}
