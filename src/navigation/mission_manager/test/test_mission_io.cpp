#include "mission_manager/mission_io.hpp"

#include <string>

#include <gtest/gtest.h>

namespace mission_manager {
namespace {

TEST(MissionIo, AcceptsTargetsWithoutGoal) {
  const std::string mission_yaml = R"(mission_name: target_only
frame_id: odom
planning_mode: straight_line
targets:
  - pose:
      x: 1.0
      y: 2.0
      yaw: 0.5
)";

  MissionDefinition mission;
  std::string error_message;

  ASSERT_TRUE(mission_from_yaml_string(mission_yaml, mission, error_message))
      << error_message;
  EXPECT_EQ(mission.mission_name, "target_only");
  ASSERT_EQ(mission.targets.size(), 1U);
  EXPECT_DOUBLE_EQ(mission.targets[0].pose.x, 1.0);
  EXPECT_DOUBLE_EQ(mission.targets[0].pose.y, 2.0);
  EXPECT_DOUBLE_EQ(mission.targets[0].pose.yaw, 0.5);
  EXPECT_FALSE(mission.has_goal);
  EXPECT_TRUE(mission.relative_goals.empty());
}

TEST(MissionIo, RejectsMissionWithoutExecutablePoint) {
  const std::string mission_yaml = R"(mission_name: empty_route
frame_id: odom
planning_mode: straight_line
targets: []
)";

  MissionDefinition mission;
  std::string error_message;

  EXPECT_FALSE(mission_from_yaml_string(mission_yaml, mission, error_message));
  EXPECT_NE(error_message.find("targets"), std::string::npos);
}

}  // namespace
}  // namespace mission_manager
