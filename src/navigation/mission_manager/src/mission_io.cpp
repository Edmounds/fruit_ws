#include "mission_manager/mission_io.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace mission_manager {

namespace {

bool pose_from_yaml(const YAML::Node& node,
                    const std::string& path,
                    MissionPose& pose,
                    std::string& error_message) {
  if (!node || !node.IsMap()) {
    error_message = "mission YAML pose 节点缺失或不是映射: " + path;
    return false;
  }

  auto parse_field = [&](const char* field_name, double& value) {
    const std::string field_path = path + "." + field_name;
    const YAML::Node field = node[field_name];
    if (!field || !field.IsScalar()) {
      error_message = "mission YAML pose 字段缺失或不是标量: " + field_path;
      return false;
    }
    try {
      value = field.as<double>();
    } catch (const std::exception&) {
      error_message = "mission YAML pose 字段不是数值: " + field_path;
      return false;
    }
    if (!std::isfinite(value)) {
      error_message = "mission YAML pose 字段不是有限数值: " + field_path;
      return false;
    }
    return true;
  };

  return parse_field("x", pose.x) && parse_field("y", pose.y) &&
         parse_field("yaw", pose.yaw);
}

bool relative_goal_pose_from_yaml(const YAML::Node& node,
                                  const std::string& path,
                                  MissionPose& pose,
                                  std::string& error_message) {
  if (!node || !node.IsMap()) {
    error_message = "mission YAML pose 节点缺失或不是映射: " + path;
    return false;
  }

  pose.x = 0.0;
  pose.y = 0.0;

  auto parse_field = [&](const char* field_name, double& value, bool required) {
    const std::string field_path = path + "." + field_name;
    const YAML::Node field = node[field_name];
    if (!field) {
      if (!required) {
        return true;
      }
      error_message = "mission YAML pose 字段缺失或不是标量: " + field_path;
      return false;
    }
    if (!field.IsScalar()) {
      error_message = "mission YAML pose 字段缺失或不是标量: " + field_path;
      return false;
    }
    try {
      value = field.as<double>();
    } catch (const std::exception&) {
      error_message = "mission YAML pose 字段不是数值: " + field_path;
      return false;
    }
    if (!std::isfinite(value)) {
      error_message = "mission YAML pose 字段不是有限数值: " + field_path;
      return false;
    }
    return true;
  };

  return parse_field("x", pose.x, false) &&
         parse_field("y", pose.y, false) &&
         parse_field("yaw", pose.yaw, true);
}

YAML::Node pose_to_yaml(const MissionPose& pose) {
  YAML::Node node;
  node["x"] = pose.x;
  node["y"] = pose.y;
  node["yaw"] = pose.yaw;
  return node;
}

bool waypoints_from_yaml(const YAML::Node& node,
                         const std::string& path,
                         std::vector<MissionPose>& waypoints,
                         std::string& error_message) {
  waypoints.clear();
  if (!node) {
    return true;
  }
  if (!node.IsSequence()) {
    error_message = "mission YAML waypoints 不是序列: " + path;
    return false;
  }

  std::size_t waypoint_index = 0;
  for (const auto& waypoint_node : node) {
    MissionPose waypoint;
    if (!pose_from_yaml(
          waypoint_node, path + "[" + std::to_string(waypoint_index) + "]",
          waypoint, error_message)) {
      return false;
    }
    waypoints.push_back(waypoint);
    ++waypoint_index;
  }
  return true;
}

YAML::Node waypoints_to_yaml(const std::vector<MissionPose>& waypoints) {
  YAML::Node node(YAML::NodeType::Sequence);
  for (const auto& waypoint : waypoints) {
    node.push_back(pose_to_yaml(waypoint));
  }
  return node;
}

bool reject_old_mission_format(const YAML::Node& root, std::string& error_message) {
  if (root["segments"] || root["relative_segments"] || root["navigation_sequence"]) {
    error_message =
        "old mission format unsupported: use start, targets, goal, "
        "goal_waypoints, and relative_goals";
    return false;
  }
  return true;
}

bool parse_yaml_root(const YAML::Node& root,
                     MissionDefinition& parsed_mission,
                     std::string& error_message) {
  if (!reject_old_mission_format(root, error_message)) {
    return false;
  }

  parsed_mission.mission_name =
      root["mission_name"] ? root["mission_name"].as<std::string>() : "unnamed_mission";
  parsed_mission.frame_id = root["frame_id"] ? root["frame_id"].as<std::string>() : "map";
  parsed_mission.planning_mode =
      root["planning_mode"] ? root["planning_mode"].as<std::string>() : "straight_line";

  if (root["start"]) {
    if (!pose_from_yaml(root["start"], "start", parsed_mission.start, error_message)) {
      return false;
    }
    parsed_mission.has_start = true;
  }

  if (root["targets"]) {
    if (!root["targets"].IsSequence()) {
      error_message = "mission YAML targets 不是序列";
      return false;
    }
    std::size_t index = 0;
    for (const auto& target_node : root["targets"]) {
      if (!target_node || !target_node.IsMap()) {
        error_message =
            "mission YAML target 节点缺失或不是映射: targets[" +
            std::to_string(index) + "]";
        return false;
      }

      MissionTarget target;
      if (!waypoints_from_yaml(
            target_node["waypoints"],
            "targets[" + std::to_string(index) + "].waypoints",
            target.waypoints,
            error_message)) {
        return false;
      }
      if (!pose_from_yaml(
            target_node["pose"],
            "targets[" + std::to_string(index) + "].pose",
            target.pose,
            error_message)) {
        return false;
      }
      parsed_mission.targets.push_back(target);
      ++index;
    }
  }

  if (!waypoints_from_yaml(
        root["goal_waypoints"], "goal_waypoints", parsed_mission.goal_waypoints,
        error_message)) {
    return false;
  }

  if (root["goal"]) {
    if (!pose_from_yaml(root["goal"], "goal", parsed_mission.goal, error_message)) {
      return false;
    }
    parsed_mission.has_goal = true;
  }

  if (root["relative_goals"]) {
    if (!root["relative_goals"].IsMap()) {
      error_message = "mission YAML relative_goals 不是映射";
      return false;
    }
    for (const auto& relative_node : root["relative_goals"]) {
      const std::string relative_name = relative_node.first.as<std::string>();
      if (relative_name.empty()) {
        error_message = "mission YAML relative_goals 名称不能为空";
        return false;
      }

      MissionPose pose;
      if (!relative_goal_pose_from_yaml(
            relative_node.second, "relative_goals." + relative_name, pose,
            error_message)) {
        return false;
      }
      parsed_mission.relative_goals[relative_name] = pose;
    }
  }

  return true;
}

}  // namespace

bool load_mission_file(const std::string& mission_path,
                       MissionDefinition& mission,
                       std::string& error_message) {
  std::ifstream stream(mission_path);
  if (!stream.is_open()) {
    error_message = "无法打开 mission 文件: " + mission_path;
    return false;
  }

  std::stringstream buffer;
  buffer << stream.rdbuf();
  return mission_from_yaml_string(buffer.str(), mission, error_message);
}

bool mission_from_yaml_string(const std::string& mission_yaml,
                              MissionDefinition& mission,
                              std::string& error_message) {
  try {
    YAML::Node root = YAML::Load(mission_yaml);
    if (!root || !root.IsMap()) {
      error_message = "mission YAML 为空或根节点不是映射";
      return false;
    }

    MissionDefinition parsed_mission;
    if (!parse_yaml_root(root, parsed_mission, error_message)) {
      return false;
    }
    if (
        parsed_mission.targets.empty() && !parsed_mission.has_goal &&
        parsed_mission.relative_goals.empty()) {
      error_message = "mission YAML 缺少 targets, final goal 或 relative_goals";
      return false;
    }
    if (
        parsed_mission.planning_mode != "straight_line" &&
        parsed_mission.planning_mode != "theta_star") {
      error_message = "mission planning_mode 必须为 straight_line 或 theta_star";
      return false;
    }
    mission = parsed_mission;
    return true;
  } catch (const std::exception& ex) {
    error_message = std::string("解析 mission YAML 失败: ") + ex.what();
    return false;
  }
}

bool save_mission_file(const std::string& mission_path,
                       const MissionDefinition& mission,
                       std::string& error_message) {
  std::ofstream stream(mission_path);
  if (!stream.is_open()) {
    error_message = "无法写入 mission 文件: " + mission_path;
    return false;
  }
  stream << mission_to_yaml_string(mission);
  stream.flush();
  if (!stream.good()) {
    error_message = "写入 mission 文件后状态异常: " + mission_path;
    return false;
  }
  return true;
}

std::string mission_to_yaml_string(const MissionDefinition& mission) {
  YAML::Node root;
  root["mission_name"] = mission.mission_name;
  root["frame_id"] = mission.frame_id.empty() ? "map" : mission.frame_id;
  root["planning_mode"] =
      mission.planning_mode.empty() ? "straight_line" : mission.planning_mode;

  if (mission.has_start) {
    root["start"] = pose_to_yaml(mission.start);
  }

  YAML::Node targets_node(YAML::NodeType::Sequence);
  for (const auto& target : mission.targets) {
    YAML::Node target_node;
    if (!target.waypoints.empty()) {
      target_node["waypoints"] = waypoints_to_yaml(target.waypoints);
    }
    target_node["pose"] = pose_to_yaml(target.pose);
    targets_node.push_back(target_node);
  }
  root["targets"] = targets_node;

  if (!mission.goal_waypoints.empty()) {
    root["goal_waypoints"] = waypoints_to_yaml(mission.goal_waypoints);
  }

  if (mission.has_goal) {
    root["goal"] = pose_to_yaml(mission.goal);
  }

  if (!mission.relative_goals.empty()) {
    YAML::Node relative_goals_node(YAML::NodeType::Map);
    for (const auto& [relative_name, pose] : mission.relative_goals) {
      relative_goals_node[relative_name] = pose_to_yaml(pose);
    }
    root["relative_goals"] = relative_goals_node;
  }

  YAML::Emitter emitter;
  emitter.SetIndent(2);
  emitter << root;
  return std::string(emitter.c_str()) + "\n";
}

}  // namespace mission_manager
