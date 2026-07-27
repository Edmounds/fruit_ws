#ifndef MISSION_MANAGER__MISSION_TYPES_HPP_
#define MISSION_MANAGER__MISSION_TYPES_HPP_

#include <map>
#include <string>
#include <vector>

namespace mission_manager {

/// 任务位姿，包含 2D 位置和航向角。
struct MissionPose {
  double x {0.0};
  double y {0.0};
  double yaw {0.0};
};

/// 中间目标点：waypoints 只约束导航到该 target 的路径，pose 是停车目标。
struct MissionTarget {
  std::vector<MissionPose> waypoints;
  MissionPose pose;
};

/// 完整的任务定义，包含多个有序的任务段以及全局配置。
struct MissionDefinition {
  std::string mission_name;
  std::string frame_id {"map"};
  std::string planning_mode {"straight_line"};
  bool has_start {false};
  MissionPose start;
  std::vector<MissionTarget> targets;
  std::vector<MissionPose> goal_waypoints;
  bool has_goal {false};
  MissionPose goal;
  std::map<std::string, MissionPose> relative_goals;
};

}  // namespace mission_manager

#endif  // MISSION_MANAGER__MISSION_TYPES_HPP_
