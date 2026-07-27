#ifndef PB_DIFF_TELEOP_TWIST_JOY__TELEOP_COMMAND_HPP_
#define PB_DIFF_TELEOP_TWIST_JOY__TELEOP_COMMAND_HPP_

#include <algorithm>
#include <cstddef>
#include <vector>

namespace pb_diff_teleop_twist_joy
{

struct TeleopCommand
{
  double linear_x{};
  double angular_z{};
};

inline double readAxis(const std::vector<float> & axes, int index)
{
  return index >= 0 && static_cast<std::size_t>(index) < axes.size() ? axes[index] : 0.0;
}

inline TeleopCommand makeTeleopCommand(
  const std::vector<float> & axes, int linear_axis, int angular_axis,
  double linear_scale, double angular_scale)
{
  return {
    std::clamp(static_cast<double>(readAxis(axes, linear_axis)), -1.0, 1.0) * linear_scale,
    std::clamp(static_cast<double>(readAxis(axes, angular_axis)), -1.0, 1.0) * angular_scale};
}

}  // namespace pb_diff_teleop_twist_joy

#endif  // PB_DIFF_TELEOP_TWIST_JOY__TELEOP_COMMAND_HPP_
