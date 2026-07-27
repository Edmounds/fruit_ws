#ifndef PB_DIFF_PID_PURSUIT_CONTROLLER__DIFF_DRIVE_COMMAND_HPP_
#define PB_DIFF_PID_PURSUIT_CONTROLLER__DIFF_DRIVE_COMMAND_HPP_

#include <algorithm>
#include <cmath>

namespace pb_diff_pid_pursuit_controller
{

struct DiffDriveCommand
{
  double linear_x{};
  double angular_z{};
};

inline DiffDriveCommand makeDiffDriveCommand(
  double requested_linear, double requested_angular, double heading_error,
  double distance_to_goal, double final_yaw_error, double rotate_threshold,
  double goal_alignment_distance, double max_forward, double max_reverse,
  double max_angular)
{
  DiffDriveCommand command;
  const double angular_error =
    distance_to_goal <= goal_alignment_distance ? final_yaw_error : heading_error;
  command.angular_z = std::clamp(requested_angular, -max_angular, max_angular);

  if (distance_to_goal <= goal_alignment_distance || std::abs(heading_error) > rotate_threshold) {
    command.linear_x = 0.0;
    if (std::abs(command.angular_z) < 1e-9) {
      command.angular_z = std::clamp(angular_error, -max_angular, max_angular);
    }
    return command;
  }

  command.linear_x = requested_linear * std::max(0.0, std::cos(heading_error));
  command.linear_x = std::clamp(command.linear_x, -max_reverse, max_forward);
  return command;
}

}  // namespace pb_diff_pid_pursuit_controller

#endif  // PB_DIFF_PID_PURSUIT_CONTROLLER__DIFF_DRIVE_COMMAND_HPP_
