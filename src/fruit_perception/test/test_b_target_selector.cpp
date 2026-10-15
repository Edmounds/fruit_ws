#include <gtest/gtest.h>

#include <limits>

#include "fruit_perception/b_target_selector.hpp"

namespace
{
using fruit_perception::BTargetCandidate;
using fruit_perception::BTargetSelector;

BTargetCandidate fruit(const std::string & type = "tomato_ripe", double confidence = 0.9)
{
  return {type, {0.2, 0.0, 0.5}, {320.0, 200.0}, 640U, confidence, 0.9,
    {220, 100, 200, 200}, true};
}

TEST(TargetVote, WaitsForTenFramesIncludingEmptyFrames)
{
  BTargetSelector selector({10U, 0.03, 2.0});
  for (int frame = 0; frame < 10; ++frame) {
    const auto result = selector.update(
      frame == 0 ? std::vector<BTargetCandidate>{fruit()} :
      std::vector<BTargetCandidate>{});
    EXPECT_EQ(result.ready, frame == 9);
    if (result.ready) {
      ASSERT_EQ(result.targets.size(), 1U);
      EXPECT_EQ(result.targets[0].type, "tomato_ripe");
      EXPECT_EQ(result.targets[0].observation_count, 1U);
      EXPECT_TRUE(result.targets[0].has_point);
    }
  }
}

TEST(TargetVote, PluralityDoesNotRequireMajorityOrFinalFrameSupport)
{
  BTargetSelector selector({10U, 0.03, 2.0});
  fruit_perception::BTargetUpdate result;
  for (int frame = 0; frame < 10; ++frame) {
    result = selector.update(
      frame < 3 ? std::vector<BTargetCandidate>{fruit()} :
      (frame >= 8 ? std::vector<BTargetCandidate>{fruit("tomato_unripe")} :
      std::vector<BTargetCandidate>{}));
  }
  ASSERT_EQ(result.targets.size(), 1U);
  EXPECT_EQ(result.targets[0].type, "tomato_ripe");
  EXPECT_EQ(result.targets[0].observation_count, 3U);
}

TEST(TargetVote, TiesUseMeanDetectorConfidenceAndUnresolvedTiesDoNotWin)
{
  for (const double unripe_confidence : {0.8, 0.9}) {
    BTargetSelector selector({10U, 0.03, 2.0});
    fruit_perception::BTargetUpdate result;
    for (int frame = 0; frame < 10; ++frame) {
      result = selector.update(
        {frame < 5 ? fruit() :
          fruit("tomato_unripe", unripe_confidence)});
    }
    ASSERT_EQ(result.targets.size(), 1U);
    EXPECT_EQ(result.targets[0].type, unripe_confidence == 0.9 ? "unknown" : "tomato_ripe");
  }
}

TEST(TargetVote, LowMaturityConfidenceDoesNotVote)
{
  BTargetSelector selector({10U, 0.03, 2.0});
  fruit_perception::BTargetUpdate result;
  for (int frame = 0; frame < 10; ++frame) {
    auto candidate = fruit(frame == 0 ? "tomato_unripe" : "tomato_ripe");
    candidate.maturity_confidence = frame == 0 ? 0.70 : 0.69;
    result = selector.update({candidate});
  }
  ASSERT_EQ(result.targets.size(), 1U);
  EXPECT_EQ(result.targets[0].type, "tomato_unripe");
}

TEST(TargetVote, BadVetoSurvivesDropoutAndOnlyAffectsAssociatedFruit)
{
  BTargetSelector selector({10U, 0.03, 2.0});
  fruit_perception::BTargetUpdate result;
  for (int frame = 0; frame < 10; ++frame) {
    auto neighbor = fruit("pear_ripe");
    neighbor.roi.x = 450;
    neighbor.pixel_center.x = 550;
    neighbor.point.x = 0.4;
    result = selector.update(
      {fruit(), neighbor}, frame == 1 ?
      std::vector<cv::Rect>{{300, 180, 20, 20}} : std::vector<cv::Rect>{});
  }
  ASSERT_EQ(result.targets.size(), 2U);
  EXPECT_EQ(result.targets[0].type, "bad");
  EXPECT_FALSE(result.targets[0].has_point);
  EXPECT_EQ(result.targets[1].type, "pear_ripe");
  EXPECT_TRUE(result.targets[1].has_point);
  // Finished votes stay frozen. A new observation explicitly clears bad memory.
  EXPECT_EQ(selector.update({fruit()}).targets[0].type, "bad");
  selector.reset();
  for (int frame = 0; frame < 10; ++frame) {
    result = selector.update({fruit()});
  }
  EXPECT_EQ(result.targets[0].type, "tomato_ripe");
}

TEST(TargetVote, BadBeforeFruitAndAmbiguousAttributionRemainConservative)
{
  BTargetSelector selector({10U, 0.03, 2.0});
  selector.update({}, {{300, 180, 20, 20}});
  fruit_perception::BTargetUpdate result;
  for (int frame = 1; frame < 10; ++frame) {
    auto overlapping = fruit("pear_ripe");
    overlapping.roi.x = 250;
    result = selector.update({fruit(), overlapping});
  }
  ASSERT_EQ(result.targets.size(), 2U);
  EXPECT_EQ(result.targets[0].type, "bad");
  EXPECT_EQ(result.targets[1].type, "bad");
}

TEST(TargetVote, DepthIsOptionalButInvalidCoordinatesNeverBecomeGraspPoints)
{
  BTargetSelector selector({10U, 0.03, 2.0});
  fruit_perception::BTargetUpdate result;
  for (int frame = 0; frame < 10; ++frame) {
    auto candidate = fruit();
    candidate.has_point = frame % 2 == 0;
    candidate.point.z = std::numeric_limits<double>::quiet_NaN();
    result = selector.update({candidate});
  }
  ASSERT_EQ(result.targets.size(), 1U);
  EXPECT_EQ(result.targets[0].type, "tomato_ripe");
  EXPECT_FALSE(result.targets[0].has_point);
}

TEST(TargetVote, PerRoiDefectFlagUsesTheSameLatchedVeto)
{
  BTargetSelector selector({10U, 0.03, 2.0});
  fruit_perception::BTargetUpdate result;
  for (int frame = 0; frame < 10; ++frame) {
    auto candidate = fruit();
    candidate.bad = frame == 2;
    candidate.has_point = false;
    result = selector.update({candidate});
  }
  ASSERT_EQ(result.targets.size(), 1U);
  EXPECT_EQ(result.targets[0].type, "bad");
  EXPECT_FALSE(result.targets[0].has_point);
}

TEST(TargetVote, AggregatesCoordinatesAndRejectsOutliers)
{
  BTargetSelector selector({10U, 0.03, 2.0});
  fruit_perception::BTargetUpdate result;
  for (int frame = 0; frame < 10; ++frame) {
    auto candidate = fruit();
    candidate.point.x = frame == 9 ? 0.8 : 0.2;
    result = selector.update({candidate});
  }
  ASSERT_EQ(result.targets.size(), 1U);
  EXPECT_TRUE(result.targets[0].has_point);
  EXPECT_EQ(result.targets[0].inlier_count, 9U);
  EXPECT_NEAR(result.targets[0].point.x, 0.2, 1e-9);
}

TEST(TargetVote, EmptyAndUncertainObservationsDoNotSupplyRipeTargets)
{
  BTargetSelector selector({10U, 0.03, 2.0});
  for (int frame = 0; frame < 10; ++frame) {
    const auto result = selector.update({});
    EXPECT_EQ(result.ready, frame == 9);
    EXPECT_TRUE(result.targets.empty());
  }
  selector.reset();
  auto candidate = fruit();
  candidate.maturity_confidence = 0.6;
  for (int frame = 0; frame < 10; ++frame) {
    const auto result = selector.update({candidate});
    if (result.ready) {
      ASSERT_EQ(result.targets.size(), 1U);
      EXPECT_EQ(result.targets[0].type, "unknown");
      EXPECT_FALSE(result.targets[0].has_point);
    }
  }
}

TEST(DefectGeometry, UsesDefectAreaRatherThanWholeFruitIoU)
{
  using fruit_perception::defect_belongs_to_fruit;
  EXPECT_TRUE(defect_belongs_to_fruit({300, 180, 10, 10}, {220, 100, 200, 200}));
  EXPECT_TRUE(defect_belongs_to_fruit({210, 180, 20, 20}, {220, 100, 200, 200}));
  EXPECT_FALSE(defect_belongs_to_fruit({209, 180, 20, 20}, {220, 100, 200, 200}));
  EXPECT_FALSE(defect_belongs_to_fruit({500, 180, 10, 10}, {220, 100, 200, 200}));
  EXPECT_FALSE(defect_belongs_to_fruit({}, {220, 100, 200, 200}));
}

TEST(PrimaryScore, ChoosesBeforeBadFilteringAndKeepsConfidenceTieBreak)
{
  fruit_perception::BTargetResult bad, normal;
  bad.type = "bad";
  bad.confidence = 0.9;
  bad.pixel_center.x = 320;
  bad.image_width = 640;
  normal = bad;
  normal.type = "tomato_ripe";
  normal.pixel_center.x = 40;
  EXPECT_EQ(fruit_perception::select_primary_target({bad, normal}, 0.7, 0.05)->type, "bad");
  normal.pixel_center.x = 320;
  normal.confidence = 0.92;
  EXPECT_EQ(
    fruit_perception::select_primary_target({bad, normal}, 0.7, 0.05)->type,
    "tomato_ripe");
  EXPECT_FALSE(fruit_perception::select_primary_target({}, 0.7, 0.05));
}

}  // namespace
