// Copyright 2026 ConQU2026

#ifndef JOY_CONVENTOR__AC_PICK_BUTTONS_HPP_
#define JOY_CONVENTOR__AC_PICK_BUTTONS_HPP_

#include <cstdint>
#include <vector>

namespace joy_conventor
{

std::vector<uint8_t> ac_pick_statuses_for_button_edges(
  const std::vector<int32_t> & buttons,
  const std::vector<int32_t> & previous_buttons,
  int left_button,
  int right_button);

bool return_initial_pose_requested_for_button_edge(
  const std::vector<int32_t> & buttons,
  const std::vector<int32_t> & previous_buttons,
  int button);

}  // namespace joy_conventor

#endif  // JOY_CONVENTOR__AC_PICK_BUTTONS_HPP_
