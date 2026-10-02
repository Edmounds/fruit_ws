#include "fruit_perception/perception_mode.hpp"

#include <utility>

namespace fruit_perception
{
namespace
{
bool uses_camera(PerceptionMode mode)
{
  return mode == PerceptionMode::kQr || mode == PerceptionMode::kDetect;
}
}  // namespace

std::optional<PerceptionMode> parse_perception_mode(const std::string & value)
{
  if (value == "OFF") {
    return PerceptionMode::kOff;
  }
  if (value == "QR") {
    return PerceptionMode::kQr;
  }
  if (value == "DETECT") {
    return PerceptionMode::kDetect;
  }
  return std::nullopt;
}

const char * to_string(PerceptionMode mode)
{
  switch (mode) {
    case PerceptionMode::kOff:
      return "OFF";
    case PerceptionMode::kQr:
      return "QR";
    case PerceptionMode::kDetect:
      return "DETECT";
  }
  return "OFF";
}

bool runs_camera_inference(PerceptionMode mode)
{
  return mode == PerceptionMode::kDetect;
}

std::vector<CameraAction> plan_camera_transition(
  PerceptionMode current, PerceptionMode target)
{
  std::vector<CameraAction> actions;
  if (uses_camera(current) && !uses_camera(target)) {
    actions.push_back(CameraAction::kStopCamera);
  }
  if (!uses_camera(current) && uses_camera(target)) {
    actions.push_back(CameraAction::kStartCamera);
  }
  return actions;
}

std::vector<CameraAction> plan_startup_reset()
{
  return {CameraAction::kStopCamera};
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
