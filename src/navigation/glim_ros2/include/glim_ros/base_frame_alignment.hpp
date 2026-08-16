#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <optional>
#include <string>

namespace glim {

inline Eigen::Isometry3d baseFrameWorldAlignment(
  const Eigen::Isometry3d& T_world_imu,
  const Eigen::Isometry3d& T_imu_base)
{
  return (T_world_imu * T_imu_base).inverse();
}

inline bool writeBaseFrameAlignment(
  std::ostream& stream,
  const Eigen::Isometry3d& T_map_world)
{
  stream << std::setprecision(17) << T_map_world.matrix() << '\n';
  return stream.good();
}

inline std::optional<Eigen::Isometry3d> readBaseFrameAlignment(std::istream& stream)
{
  Eigen::Matrix4d matrix;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      if (!(stream >> matrix(row, col))) {
        return std::nullopt;
      }
    }
  }

  const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
  const Eigen::Vector4d homogeneous_row(0.0, 0.0, 0.0, 1.0);
  if (!matrix.allFinite() ||
      !matrix.row(3).transpose().isApprox(homogeneous_row, 1e-9) ||
      !(rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-9) ||
      std::abs(rotation.determinant() - 1.0) > 1e-9) {
    return std::nullopt;
  }

  Eigen::Isometry3d T_map_world;
  T_map_world.matrix() = matrix;
  return T_map_world;
}

inline bool saveBaseFrameAlignment(
  const std::string& dump_path,
  const Eigen::Isometry3d& T_map_world)
{
  std::ofstream stream(dump_path + "/base_frame_alignment.txt");
  return writeBaseFrameAlignment(stream, T_map_world);
}

inline std::optional<Eigen::Isometry3d> loadBaseFrameAlignment(
  const std::string& dump_path)
{
  std::ifstream stream(dump_path + "/base_frame_alignment.txt");
  if (!stream) {
    return std::nullopt;
  }
  return readBaseFrameAlignment(stream);
}

}  // namespace glim
