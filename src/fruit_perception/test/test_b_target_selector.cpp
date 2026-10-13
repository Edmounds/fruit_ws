#include <gtest/gtest.h>

#include "fruit_perception/b_target_selector.hpp"

namespace
{

TEST(BTargetSelector, ConfirmsCoordinatesWithoutWaitingForAnUnconfirmedTrack)
{
  fruit_perception::BTargetSelector selector({8U, 0.03, 2.0});
  fruit_perception::BTargetUpdate result;
  for (int frame = 0; frame < 8; ++frame) {
    std::vector<fruit_perception::BTargetCandidate> candidates{
      {"apple_ripe", {0.2 + frame * 0.001, 0.0, 0.5}, {100.0, 200.0}, 640U, 0.8, 0.9}};
    if (frame >= 6) {
      candidates.push_back(
        {"pear_ripe", {0.4, 0.0, 0.5}, {300.0, 200.0}, 640U, 0.9, 0.9});
    }
    result = selector.update(candidates);
    EXPECT_EQ(result.ready, frame == 7);
  }
  ASSERT_EQ(result.targets.size(), 1U);
  EXPECT_NEAR(result.targets.front().point.x, 0.2035, 1e-9);
  EXPECT_NEAR(result.targets.front().confidence, 0.8, 1e-9);
  EXPECT_EQ(result.targets.front().observation_count, 8U);
}

TEST(BTargetSelector, KeepsMixedLabelsAndAllConfirmedCoordinatesInConfidenceOrder)
{
  fruit_perception::BTargetSelector selector({8U, 0.03, 2.0});
  fruit_perception::BTargetUpdate result;
  for (int frame = 0; frame < 8; ++frame) {
    result = selector.update(
      {
        {"apple_ripe", {0.1, 0.0, 0.5}, {0.0, 0.0}, 640U, 0.7, 0.9},
        {"pear_ripe", {0.2, 0.0, 0.5}, {320.0, 300.0}, 640U, 0.9, 0.9},
        {"apple_ripe", {0.3, 0.0, 0.5}, {200.0, 200.0}, 640U, 0.8, 0.9}});
  }
  ASSERT_TRUE(result.ready);
  ASSERT_EQ(result.targets.size(), 3U);
  EXPECT_NEAR(result.targets[0].point.x, 0.2, 1e-9);
  EXPECT_NEAR(result.targets[1].point.x, 0.3, 1e-9);
  EXPECT_NEAR(result.targets[2].point.x, 0.1, 1e-9);
}

TEST(BTargetSelector, RetainsMinimumVoteAndMaturityConfidenceThresholds)
{
  fruit_perception::BTargetSelector selector({8U, 0.03, 2.0});
  for (int frame = 0; frame < 8; ++frame) {
    const auto rejected = selector.update(
      {
        {"apple_ripe", {0.2, 0.0, 0.5}, {100.0, 200.0}, 640U, 0.9, 0.69}});
    EXPECT_EQ(rejected.ready, frame == 7);
    EXPECT_TRUE(rejected.targets.empty());
  }
  selector.reset();
  for (int frame = 0; frame < 8; ++frame) {
    const auto result = selector.update(
      {
        {"apple_ripe", {0.2, 0.0, 0.5}, {100.0, 200.0}, 640U, 0.9,
          frame < 2 ? 0.69 : 0.7}});
    EXPECT_EQ(result.ready, frame == 7);
    if (frame == 7) {
      ASSERT_EQ(result.targets.size(), 1U);
    }
  }
}

TEST(BTargetSelector, EqualConfidenceKeepsInputOrderInsteadOfVoteCountOrder)
{
  fruit_perception::BTargetSelector selector({8U, 0.03, 2.0});
  fruit_perception::BTargetUpdate result;
  for (int frame = 0; frame < 8; ++frame) {
    std::vector<fruit_perception::BTargetCandidate> candidates;
    if (frame != 2 && frame != 4) {
      candidates.push_back(
        {"apple_ripe", {0.1, 0.0, 0.5}, {0.0, 0.0}, 640U, 0.8, 0.9});
    }
    if (frame >= 1) {
      candidates.push_back(
        {"apple_ripe", {0.3, 0.0, 0.5}, {320.0, 300.0}, 640U, 0.8, 0.9});
    }
    result = selector.update(candidates);
  }
  ASSERT_EQ(result.targets.size(), 2U);
  EXPECT_NEAR(result.targets[0].point.x, 0.1, 1e-9);
  EXPECT_EQ(result.targets[0].observation_count, 6U);
  EXPECT_EQ(result.targets[1].observation_count, 7U);
}

TEST(BTargetSelector, EmptyObservationIsReadyOnlyAfterWarmup)
{
  fruit_perception::BTargetSelector selector({8U, 0.03, 2.0});
  for (int frame = 0; frame < 8; ++frame) {
    const auto result = selector.update({});
    EXPECT_EQ(result.ready, frame == 7);
    EXPECT_TRUE(result.targets.empty());
  }
}

}  // namespace
