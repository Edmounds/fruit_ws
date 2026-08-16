#include <gtest/gtest.h>

#include <optional>
#include <vector>

#include "fruit_perception/perception_mode.hpp"

namespace
{
using fruit_perception::CameraAction;
using fruit_perception::PerceptionMode;

TEST(PerceptionMode, AcceptsOnlyFourUppercaseValues)
{
  EXPECT_EQ(fruit_perception::parse_perception_mode("OFF"), PerceptionMode::kOff);
  EXPECT_EQ(fruit_perception::parse_perception_mode("SIDES"), PerceptionMode::kSides);
  EXPECT_EQ(fruit_perception::parse_perception_mode("QR"), PerceptionMode::kQr);
  EXPECT_EQ(fruit_perception::parse_perception_mode("TREE"), PerceptionMode::kTree);
  EXPECT_FALSE(fruit_perception::parse_perception_mode("tree"));
  EXPECT_FALSE(fruit_perception::parse_perception_mode("D"));
}

TEST(PerceptionMode, MapsConsumersAndCameraGroups)
{
  EXPECT_FALSE(fruit_perception::runs_side_inference(PerceptionMode::kOff));
  EXPECT_TRUE(fruit_perception::runs_side_inference(PerceptionMode::kSides));
  EXPECT_FALSE(fruit_perception::runs_side_inference(PerceptionMode::kQr));
  EXPECT_FALSE(fruit_perception::runs_side_inference(PerceptionMode::kTree));

  EXPECT_FALSE(fruit_perception::runs_d435i_inference(PerceptionMode::kQr));
  EXPECT_TRUE(fruit_perception::runs_d435i_inference(PerceptionMode::kTree));
}

TEST(PerceptionMode, StopsOutgoingCameraBeforeStartingIncomingCamera)
{
  EXPECT_EQ(
    fruit_perception::plan_camera_transition(
      PerceptionMode::kSides, PerceptionMode::kQr),
    (std::vector<CameraAction>{CameraAction::kStopSides, CameraAction::kStartD435i}));
  EXPECT_EQ(
    fruit_perception::plan_camera_transition(
      PerceptionMode::kTree, PerceptionMode::kSides),
    (std::vector<CameraAction>{CameraAction::kStopD435i, CameraAction::kStartSides}));
}

TEST(PerceptionMode, QrTreeAndRepeatedModesDoNotRestartCameras)
{
  EXPECT_TRUE(
    fruit_perception::plan_camera_transition(
      PerceptionMode::kQr, PerceptionMode::kTree).empty());
  EXPECT_TRUE(
    fruit_perception::plan_camera_transition(
      PerceptionMode::kTree, PerceptionMode::kQr).empty());
  EXPECT_TRUE(
    fruit_perception::plan_camera_transition(
      PerceptionMode::kSides, PerceptionMode::kSides).empty());
}

TEST(PerceptionMode, StartupResetStopsEveryCamera)
{
  EXPECT_EQ(
    fruit_perception::plan_startup_reset(),
    (std::vector<CameraAction>{CameraAction::kStopSides, CameraAction::kStopD435i}));
}

TEST(PerceptionMode, FailedTransitionDoesNotUpdateActiveMode)
{
  fruit_perception::CameraModeState state;
  ASSERT_TRUE(state.begin_startup_reset());
  state.complete_action();
  state.complete_action();
  ASSERT_EQ(state.active_mode(), PerceptionMode::kOff);

  ASSERT_TRUE(state.begin(PerceptionMode::kSides));
  ASSERT_TRUE(state.transitioning());
  state.fail();

  EXPECT_FALSE(state.transitioning());
  EXPECT_EQ(state.active_mode(), PerceptionMode::kOff);
}

TEST(PerceptionMode, AcceptsUsbCamCaptureResponsesFromInstalledDriver)
{
  EXPECT_TRUE(fruit_perception::usb_capture_response_succeeded(true, ""));
  EXPECT_TRUE(
    fruit_perception::usb_capture_response_succeeded(false, "Start Capturing"));
  EXPECT_TRUE(
    fruit_perception::usb_capture_response_succeeded(false, "Stop Capturing"));
  EXPECT_FALSE(fruit_perception::usb_capture_response_succeeded(false, "device error"));
}
}  // namespace
