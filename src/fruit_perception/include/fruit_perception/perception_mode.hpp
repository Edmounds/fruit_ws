#ifndef FRUIT_PERCEPTION__PERCEPTION_MODE_HPP_
#define FRUIT_PERCEPTION__PERCEPTION_MODE_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace fruit_perception
{

enum class PerceptionMode {kOff, kQr, kDetect};
enum class CameraAction {kStopCamera, kStartCamera};

std::optional<PerceptionMode> parse_perception_mode(const std::string & value);
const char * to_string(PerceptionMode mode);
bool runs_camera_inference(PerceptionMode mode);
std::vector<CameraAction> plan_camera_transition(
  PerceptionMode current, PerceptionMode target);
std::vector<CameraAction> plan_startup_reset();

class CameraModeState
{
public:
  bool begin_startup_reset();
  bool begin(PerceptionMode target);
  std::optional<CameraAction> current_action() const;
  void complete_action();
  void fail();
  bool transitioning() const;
  std::optional<PerceptionMode> active_mode() const;

private:
  void finish_if_no_actions();

  std::optional<PerceptionMode> active_mode_;
  std::optional<PerceptionMode> target_mode_;
  std::vector<CameraAction> actions_;
  std::size_t action_index_{0U};
};

}  // namespace fruit_perception

#endif  // FRUIT_PERCEPTION__PERCEPTION_MODE_HPP_
