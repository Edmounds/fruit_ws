#include "fruit_perception/perception_mode.hpp"

#include <utility>

namespace fruit_perception
{
namespace
{
bool uses_sides(PerceptionMode mode)
{
  return mode == PerceptionMode::kSides;
}

bool uses_d435i(PerceptionMode mode)
{
  return mode == PerceptionMode::kQr || mode == PerceptionMode::kTree;
}
}  // namespace

std::optional<PerceptionMode> parse_perception_mode(const std::string & value)
{
  if (value == "OFF") {
    return PerceptionMode::kOff;
  }
  if (value == "SIDES") {
    return PerceptionMode::kSides;
  }
  if (value == "QR") {
    return PerceptionMode::kQr;
  }
  if (value == "TREE") {
    return PerceptionMode::kTree;
  }
  return std::nullopt;
}

const char * to_string(PerceptionMode mode)
{
  switch (mode) {
    case PerceptionMode::kOff:
      return "OFF";
    case PerceptionMode::kSides:
      return "SIDES";
    case PerceptionMode::kQr:
      return "QR";
    case PerceptionMode::kTree:
      return "TREE";
  }
  return "OFF";
}

bool runs_side_inference(PerceptionMode mode)
{
  return mode == PerceptionMode::kSides;
}

bool runs_d435i_inference(PerceptionMode mode)
{
  return mode == PerceptionMode::kTree;
}

bool usb_capture_response_succeeded(bool success, const std::string & message)
{
  return success || message == "Start Capturing" || message == "Stop Capturing";
}

std::vector<CameraAction> plan_camera_transition(
  PerceptionMode current, PerceptionMode target)
{
  std::vector<CameraAction> actions;
  if (uses_sides(current) && !uses_sides(target)) {
    actions.push_back(CameraAction::kStopSides);
  }
  if (uses_d435i(current) && !uses_d435i(target)) {
    actions.push_back(CameraAction::kStopD435i);
  }
  if (!uses_sides(current) && uses_sides(target)) {
    actions.push_back(CameraAction::kStartSides);
  }
  if (!uses_d435i(current) && uses_d435i(target)) {
    actions.push_back(CameraAction::kStartD435i);
  }
  return actions;
}

std::vector<CameraAction> plan_startup_reset()
{
  return {CameraAction::kStopSides, CameraAction::kStopD435i};
}

bool CameraModeState::begin_startup_reset()
{
  if (transitioning() || active_mode_) {
    return false;
  }
  target_mode_ = PerceptionMode::kOff;
  actions_ = plan_startup_reset();
  action_index_ = 0U;
  finish_if_no_actions();
  return true;
}

bool CameraModeState::begin(PerceptionMode target)
{
  if (transitioning() || !active_mode_) {
    return false;
  }
  target_mode_ = target;
  actions_ = plan_camera_transition(*active_mode_, target);
  action_index_ = 0U;
  finish_if_no_actions();
  return true;
}

std::optional<CameraAction> CameraModeState::current_action() const
{
  if (!transitioning() || action_index_ >= actions_.size()) {
    return std::nullopt;
  }
  return actions_[action_index_];
}

void CameraModeState::complete_action()
{
  if (!current_action()) {
    return;
  }
  ++action_index_;
  finish_if_no_actions();
}

void CameraModeState::fail()
{
  target_mode_.reset();
  actions_.clear();
  action_index_ = 0U;
}

bool CameraModeState::transitioning() const
{
  return target_mode_.has_value();
}

std::optional<PerceptionMode> CameraModeState::active_mode() const
{
  return active_mode_;
}

void CameraModeState::finish_if_no_actions()
{
  if (target_mode_ && action_index_ >= actions_.size()) {
    active_mode_ = target_mode_;
    target_mode_.reset();
    actions_.clear();
    action_index_ = 0U;
  }
}

}  // namespace fruit_perception
