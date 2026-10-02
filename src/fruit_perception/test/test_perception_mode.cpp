#include <gtest/gtest.h>

#include <optional>
#include <vector>

#include "fruit_perception/perception_mode.hpp"

namespace
{
using fruit_perception::CameraAction;
using fruit_perception::PerceptionMode;

TEST(PerceptionMode, AcceptsOnlyThreeUppercaseValues)
{
  EXPECT_EQ(fruit_perception::parse_perception_mode("OFF"), PerceptionMode::kOff);
  EXPECT_EQ(fruit_perception::parse_perception_mode("QR"), PerceptionMode::kQr);
  EXPECT_EQ(fruit_perception::parse_perception_mode("DETECT"), PerceptionMode::kDetect);
  EXPECT_FALSE(fruit_perception::parse_perception_mode("SIDES"));
  EXPECT_FALSE(fruit_perception::parse_perception_mode("TREE"));
  EXPECT_FALSE(fruit_perception::parse_perception_mode("tree"));
  EXPECT_FALSE(fruit_perception::parse_perception_mode("D"));
}

TEST(PerceptionMode, MapsConsumersAndCameraGroups)
{
  EXPECT_FALSE(fruit_perception::runs_camera_inference(PerceptionMode::kOff));
  EXPECT_FALSE(fruit_perception::runs_camera_inference(PerceptionMode::kQr));
  EXPECT_TRUE(fruit_perception::runs_camera_inference(PerceptionMode::kDetect));
}

TEST(PerceptionMode, TransitionStartsAndStopsCamera)
{
  EXPECT_EQ(
    fruit_perception::plan_camera_transition(
      PerceptionMode::kOff, PerceptionMode::kDetect),
    (std::vector<CameraAction>{CameraAction::kStartCamera}));
  EXPECT_EQ(
    fruit_perception::plan_camera_transition(
      PerceptionMode::kDetect, PerceptionMode::kOff),
    (std::vector<CameraAction>{CameraAction::kStopCamera}));
}

TEST(PerceptionMode, QrDetectAndRepeatedModesDoNotRestartCamera)
{
  EXPECT_TRUE(
    fruit_perception::plan_camera_transition(
      PerceptionMode::kQr, PerceptionMode::kDetect).empty());
  EXPECT_TRUE(
    fruit_perception::plan_camera_transition(
      PerceptionMode::kDetect, PerceptionMode::kQr).empty());
  EXPECT_TRUE(
    fruit_perception::plan_camera_transition(
      PerceptionMode::kDetect, PerceptionMode::kDetect).empty());
}

TEST(PerceptionMode, StartupResetStopsCamera)
{
  EXPECT_EQ(
    fruit_perception::plan_startup_reset(),
    (std::vector<CameraAction>{CameraAction::kStopCamera}));
}

TEST(PerceptionMode, FailedTransitionDoesNotUpdateActiveMode)
{
  fruit_perception::CameraModeState state;
  ASSERT_TRUE(state.begin_startup_reset());
  state.complete_action();
  ASSERT_EQ(state.active_mode(), PerceptionMode::kOff);

  ASSERT_TRUE(state.begin(PerceptionMode::kDetect));
  ASSERT_TRUE(state.transitioning());
  state.fail();

  EXPECT_FALSE(state.transitioning());
  EXPECT_EQ(state.active_mode(), PerceptionMode::kOff);
}
}  // namespace
