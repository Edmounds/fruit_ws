#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <optional>
#include <string>

namespace glim {

inline Eigen::Isometry3d planarMapFrameNormalization(
  const Eigen::Isometry3d& T_native_base)
{
  const double yaw = std::atan2(
    T_native_base.linear()(1, 0), T_native_base.linear()(0, 0));

  Eigen::Isometry3d normalization = Eigen::Isometry3d::Identity();
  normalization.linear() =
    Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  normalization.translation() =
    -normalization.linear() * T_native_base.translation();
  return normalization;
}

inline Eigen::Isometry3d normalizedMapToOdom(
  const Eigen::Isometry3d& normalization,
  const Eigen::Isometry3d& T_world_native_odom)
{
  return normalization * T_world_native_odom * normalization.inverse();
}

inline bool writeMapFrameNormalization(
  std::ostream& stream,
  const Eigen::Isometry3d& normalization)
{
  stream << std::setprecision(17) << normalization.matrix() << '\n';
  return stream.good();
}

inline std::optional<Eigen::Isometry3d> readMapFrameNormalization(
  std::istream& stream)
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
  if (
    !matrix.allFinite() ||
    !matrix.row(3).transpose().isApprox(homogeneous_row, 1e-9) ||
    !(rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-9) ||
    std::abs(rotation.determinant() - 1.0) > 1e-9)
  {
    return std::nullopt;
  }

  Eigen::Isometry3d normalization;
  normalization.matrix() = matrix;
  return normalization;
}

inline bool saveMapFrameNormalization(
  const std::string& dump_path,
  const Eigen::Isometry3d& normalization)
{
  std::ofstream stream(dump_path + "/map_frame_normalization.txt");
  return writeMapFrameNormalization(stream, normalization);
}

inline std::optional<Eigen::Isometry3d> loadMapFrameNormalization(
  const std::string& dump_path)
{
  std::ifstream stream(dump_path + "/map_frame_normalization.txt");
  if (!stream) {
    return std::nullopt;
  }
  return readMapFrameNormalization(stream);
}

}  // namespace glim
