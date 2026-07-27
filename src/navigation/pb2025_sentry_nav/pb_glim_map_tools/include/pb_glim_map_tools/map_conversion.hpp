#ifndef PB_GLIM_MAP_TOOLS__MAP_CONVERSION_HPP_
#define PB_GLIM_MAP_TOOLS__MAP_CONVERSION_HPP_

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pb_glim_map_tools
{

struct ConversionResult
{
  std::size_t input_points{};
  std::size_t finite_points{};
  std::size_t output_points{};
  pcl::PointXYZ min;
  pcl::PointXYZ max;
};

inline bool convertMap(
  const std::string & input, const std::string & output, double leaf_size,
  ConversionResult & result, std::string & error)
{
  if (!std::isfinite(leaf_size) || leaf_size <= 0.0) {
    error = "leaf size must be positive";
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPLYFile(input, *cloud) < 0) {
    error = "failed to read input PLY: " + input;
    return false;
  }
  result.input_points = cloud->size();

  std::vector<int> finite_indices;
  pcl::PointCloud<pcl::PointXYZ>::Ptr finite(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::removeNaNFromPointCloud(*cloud, *finite, finite_indices);
  result.finite_points = finite->size();
  if (finite->empty()) {
    error = "input map has no finite points";
    return false;
  }

  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(finite);
  const float leaf = static_cast<float>(leaf_size);
  voxel.setLeafSize(leaf, leaf, leaf);
  pcl::PointCloud<pcl::PointXYZ> filtered;
  voxel.filter(filtered);
  if (filtered.empty()) {
    error = "voxel filtering produced an empty map";
    return false;
  }

  result.output_points = filtered.size();
  pcl::getMinMax3D(filtered, result.min, result.max);
  if (pcl::io::savePCDFileBinary(output, filtered) < 0) {
    error = "failed to write output PCD: " + output;
    return false;
  }
  return true;
}

}  // namespace pb_glim_map_tools

#endif  // PB_GLIM_MAP_TOOLS__MAP_CONVERSION_HPP_
