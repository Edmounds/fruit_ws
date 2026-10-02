#ifndef FRUIT_MISSION_CAMERA_TARGET_TRANSFORM_HPP_
#define FRUIT_MISSION_CAMERA_TARGET_TRANSFORM_HPP_

#include <array>
#include <cmath>
#include <stdexcept>

namespace fruit_mission
{

using Point3 = std::array<double, 3>;
using Rotation3 = std::array<Point3, 3>;

// P_E = R_E_C * P_C + t_E_C. All positions are mm, NOT arm-base coordinates.
// Supply R/t from config/camera_extrinsic.yaml; no duplicated calibration defaults.
inline Point3 transformCameraToEndEffectorMm(
  const Point3 & camera_mm, const Rotation3 & rotation, const Point3 & translation_mm)
{
  for (std::size_t i = 0; i < 3; ++i) {
    if (!std::isfinite(camera_mm[i]) || !std::isfinite(translation_mm[i])) {
      throw std::invalid_argument("point and translation must be finite");
    }
    for (const double value : rotation[i]) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("rotation must be finite");
      }
    }
  }
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      double dot = 0.0;
      for (std::size_t k = 0; k < 3; ++k) {
        dot += rotation[i][k] * rotation[j][k];
      }
      if (std::abs(dot - (i == j ? 1.0 : 0.0)) > 1e-4) {
        throw std::invalid_argument("rotation must be orthonormal");
      }
    }
  }
  const auto & a = rotation[0];
  const auto & b = rotation[1];
  const auto & c = rotation[2];
  const double determinant =
    a[0] * (b[1] * c[2] - b[2] * c[1]) -
    a[1] * (b[0] * c[2] - b[2] * c[0]) +
    a[2] * (b[0] * c[1] - b[1] * c[0]);
  if (std::abs(determinant - 1.0) > 1e-4) {
    throw std::invalid_argument("rotation must have determinant +1");
  }
  Point3 end_mm = translation_mm;
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t k = 0; k < 3; ++k) {
      end_mm[i] += rotation[i][k] * camera_mm[k];
    }
    if (!std::isfinite(end_mm[i])) {
      throw std::invalid_argument("transformed point must be finite");
    }
  }
  return end_mm;
}

}  // namespace fruit_mission

#endif  // FRUIT_MISSION_CAMERA_TARGET_TRANSFORM_HPP_
