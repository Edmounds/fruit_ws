#include "mission_manager/mission_manager_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "mission_manager/mission_io.hpp"

namespace mission_manager {

namespace {

using namespace std::chrono_literals;

constexpr char kDefaultFrameId[] = "map";

std::string sanitize_file_stem(const std::string& input) {
  std::string output = input;
  std::transform(output.begin(), output.end(), output.begin(), [](unsigned char c) {
    if (std::isalnum(c) != 0 || c == '_' || c == '-') {
      return static_cast<char>(c);
    }
    return '_';
  });
  output.erase(
      std::remove_if(output.begin(), output.end(),
                     [](unsigned char c) { return std::isspace(c) != 0; }),
      output.end());
  if (output.empty()) {
    output = "mission";
  }
  return output;
}

std::string trim_copy(const std::string& input) {
  const auto first = std::find_if_not(input.begin(), input.end(), [](unsigned char c) {
    return std::isspace(c) != 0;
  });
  const auto last = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char c) {
    return std::isspace(c) != 0;
  }).base();
  if (first >= last) {
    return "";
  }
  return std::string(first, last);
}

geometry_msgs::msg::PoseStamped to_pose_stamped(const MissionPose& pose,
                                                const rclcpp::Time& stamp,
                                                const std::string& frame_id) {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.pose.position.x = pose.x;
  msg.pose.position.y = pose.y;
  msg.pose.position.z = 0.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = std::sin(pose.yaw * 0.5);
  msg.pose.orientation.w = std::cos(pose.yaw * 0.5);
  return msg;
}

nav_msgs::msg::Path build_path_from_points(
    const std::vector<MissionPose>& points, const rclcpp::Time& stamp,
    const std::string& frame_id) {
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  for (const auto& point : points) {
    path.poses.push_back(to_pose_stamped(point, stamp, frame_id));
  }
  return path;
}

std::string json_escape(const std::string& text) {
  std::ostringstream oss;
  for (const char c : text) {
    switch (c) {
      case '\\':
        oss << "\\\\";
        break;
      case '"':
        oss << "\\\"";
        break;
      case '\n':
        oss << "\\n";
        break;
      case '\r':
        oss << "\\r";
        break;
      case '\t':
        oss << "\\t";
        break;
      case '\b':
        oss << "\\b";
        break;
      case '\f':
        oss << "\\f";
        break;
      default:
        // 过滤 ASCII 控制字符
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
          oss << buf;
        } else {
          oss << c;
        }
        break;
    }
  }
  return oss.str();
}

std::string pose_json(const MissionPose& pose) {
  std::ostringstream oss;
  oss << "{\"x\":" << pose.x << ",\"y\":" << pose.y << ",\"yaw\":" << pose.yaw << "}";
  return oss.str();
}

}  // namespace

MissionManagerNode::MissionManagerNode() : Node("mission_manager_node") {
  declare_and_load_parameters();
  mission_.frame_id = planning_frame_id();
  mission_.planning_mode = planning_mode_;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  odom_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions odom_subscription_options;
  odom_subscription_options.callback_group = odom_callback_group_;
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&MissionManagerNode::odom_callback, this, std::placeholders::_1),
      odom_subscription_options);
  rclcpp::QoS latched_qos(rclcpp::KeepLast(1));
  latched_qos.transient_local();
  latched_qos.reliable();
  status_pub_ =
      this->create_publisher<std_msgs::msg::String>("/mission_manager/status", latched_qos);
  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/mission_markers", latched_qos);
  preview_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/mission_preview_path", rclcpp::QoS(1).transient_local().reliable());
  record_start_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/record_start_pose",
      std::bind(&MissionManagerNode::record_start_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  record_waypoint_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/record_waypoint",
      std::bind(&MissionManagerNode::record_waypoint_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  record_target_pose_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/record_target_pose",
      std::bind(&MissionManagerNode::record_target_pose_callback, this,
                std::placeholders::_1, std::placeholders::_2));
  record_goal_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/record_goal_pose",
      std::bind(&MissionManagerNode::record_goal_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  undo_waypoint_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/undo_waypoint",
      std::bind(&MissionManagerNode::undo_waypoint_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  clear_draft_waypoints_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/clear_draft_waypoints",
      std::bind(&MissionManagerNode::clear_draft_waypoints_callback, this,
                std::placeholders::_1, std::placeholders::_2));
  remove_last_target_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/remove_last_target",
      std::bind(&MissionManagerNode::remove_last_target_callback, this,
                std::placeholders::_1, std::placeholders::_2));
  clear_loaded_mission_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/clear_loaded_mission",
      std::bind(&MissionManagerNode::clear_loaded_mission_callback, this,
                std::placeholders::_1, std::placeholders::_2));
  save_mission_srv_ = this->create_service<mission_manager::srv::SaveMission>(
      "/save_mission",
      std::bind(&MissionManagerNode::save_mission_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  load_mission_srv_ = this->create_service<mission_manager::srv::LoadMission>(
      "/load_mission",
      std::bind(&MissionManagerNode::load_mission_callback, this, std::placeholders::_1,
                std::placeholders::_2));
  update_mission_point_srv_ =
      this->create_service<mission_manager::srv::UpdateMissionPoint>(
          "/update_mission_point",
          std::bind(&MissionManagerNode::update_mission_point_callback, this,
                    std::placeholders::_1, std::placeholders::_2));
  auto_load_last_mission();
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::declare_and_load_parameters() {
  std::string default_missions_dir;
  try {
    const auto package_share =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("mission_manager"));
    const auto installed_missions_dir = package_share / "missions";
    const auto workspace_root =
        package_share.parent_path().parent_path().parent_path().parent_path();
    const std::vector<std::filesystem::path> mission_dir_candidates {
        workspace_root / "mission_manager" / "missions",
        workspace_root / "src" / "rc2026_navigation" / "mission_manager" / "missions",
        workspace_root / "src" / "mission_manager" / "missions",
        installed_missions_dir,
    };
    for (const auto& candidate : mission_dir_candidates) {
      if (std::filesystem::exists(candidate)) {
        default_missions_dir = candidate.string();
        break;
      }
    }
    if (default_missions_dir.empty()) {
      default_missions_dir = installed_missions_dir.string();
    }
  } catch (const std::exception&) {
    default_missions_dir = "missions";
  }

  this->declare_parameter("missions_dir", default_missions_dir);
  this->declare_parameter("odom_topic", "odometry");
  this->declare_parameter("frame_id", "map");
  this->declare_parameter("planning_mode", "straight_line");
  this->declare_parameter("pose_avg_duration_sec", 0.5);
  this->declare_parameter("auto_load_last_mission", true);

  missions_dir_ = this->get_parameter("missions_dir").as_string();
  if (missions_dir_.empty()) {
    missions_dir_ = default_missions_dir;
  }
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  planning_mode_ = this->get_parameter("planning_mode").as_string();
  auto_load_last_mission_ = this->get_parameter("auto_load_last_mission").as_bool();
  const double pose_avg_duration_sec =
      std::max(0.1, this->get_parameter("pose_avg_duration_sec").as_double());
  pose_avg_duration_ = std::chrono::milliseconds(
      static_cast<int64_t>(std::llround(pose_avg_duration_sec * 1000.0)));
}

void MissionManagerNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::string error_message;
  bool success = false;
  const MissionPose map_pose = pose_from_msg_in_map(*msg, success, error_message);
  if (!success) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "%s", error_message.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(pose_mutex_);
  current_pose_ = map_pose;
  odom_received_ = true;

  const rclcpp::Time sample_stamp(msg->header.stamp);
  pose_buffer_.push_back(TimedMissionPose {sample_stamp, map_pose});
  const rclcpp::Duration window_duration = rclcpp::Duration::from_seconds(
      std::chrono::duration<double>(pose_avg_duration_).count());
  const rclcpp::Time window_start = sample_stamp - window_duration;
  while (!pose_buffer_.empty() && pose_buffer_.front().stamp < window_start) {
    pose_buffer_.pop_front();
  }
  pose_cv_.notify_all();
}

void MissionManagerNode::record_start_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  std::string error_message;
  MissionPose captured_pose;
  if (!wait_and_capture_pose(captured_pose, error_message)) {
    response->success = false;
    response->message = error_message;
  } else {
    const bool route_empty =
        mission_.targets.empty() && !mission_.has_goal && mission_.relative_goals.empty();
    if (route_empty) {
      mission_.frame_id = planning_frame_id();
      mission_.planning_mode = planning_mode_;
    }
    mission_.start = captured_pose;
    mission_.has_start = true;
    if (mission_.mission_name.empty()) {
      mission_.mission_name = "unnamed_mission";
    }
    execution_state_ = ExecutionState::kMissionLoaded;
    last_result_message_ = "已基于当前机器人位姿采集起点";
    response->success = true;
    response->message = last_result_message_;
  }
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::record_waypoint_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  std::string error_message;
  MissionPose waypoint;
  if (!wait_and_capture_pose(waypoint, error_message)) {
    response->success = false;
    response->message = error_message;
  } else {
    draft_waypoints_.push_back(waypoint);
    last_result_message_ = "已基于当前机器人位姿采集途径点";
    response->success = true;
    response->message = last_result_message_;
  }
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::record_target_pose_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  std::string error_message;
  MissionPose captured_pose;
  if (!wait_and_capture_pose(captured_pose, error_message)) {
    response->success = false;
    response->message = error_message;
  } else {
    const bool route_empty =
        mission_.targets.empty() && !mission_.has_goal && mission_.relative_goals.empty();
    if (route_empty) {
      mission_.frame_id = planning_frame_id();
      mission_.planning_mode = planning_mode_;
    }

    MissionTarget target;
    target.waypoints = draft_waypoints_;
    target.pose = captured_pose;
    mission_.targets.push_back(target);
    clear_draft_waypoints();
    if (mission_.mission_name.empty()) {
      mission_.mission_name = "unnamed_mission";
    }
    execution_state_ = ExecutionState::kMissionLoaded;
    last_result_message_ =
        "已采集目标点 target_index=" +
        std::to_string(static_cast<int>(mission_.targets.size()) - 1);
    response->success = true;
    response->message = last_result_message_;
  }
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::record_goal_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  std::string error_message;
  MissionPose captured_pose;
  if (!wait_and_capture_pose(captured_pose, error_message)) {
    response->success = false;
    response->message = error_message;
  } else {
    const bool route_empty =
        mission_.targets.empty() && !mission_.has_goal && mission_.relative_goals.empty();
    if (route_empty) {
      mission_.frame_id = planning_frame_id();
      mission_.planning_mode = planning_mode_;
    }
    mission_.goal = captured_pose;
    mission_.goal_waypoints = draft_waypoints_;
    mission_.has_goal = true;
    clear_draft_waypoints();
    if (mission_.mission_name.empty()) {
      mission_.mission_name = "unnamed_mission";
    }
    execution_state_ = ExecutionState::kMissionLoaded;
    last_result_message_ =
        "已采集终点 target_index=" + std::to_string(mission_.targets.size());
    response->success = true;
    response->message = last_result_message_;
  }
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::undo_waypoint_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  if (draft_waypoints_.empty()) {
    response->success = false;
    response->message = "当前没有可撤销的途径点";
  } else {
    draft_waypoints_.pop_back();
    last_result_message_ = "已撤销最后一个途径点";
    response->success = true;
    response->message = last_result_message_;
  }
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::clear_draft_waypoints_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  clear_draft_waypoints();
  last_result_message_ = "已清空待附加途径点";
  response->success = true;
  response->message = last_result_message_;
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::remove_last_target_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  if (mission_.has_goal) {
    mission_.has_goal = false;
    mission_.goal = MissionPose {};
    mission_.goal_waypoints.clear();
    last_result_message_ = "已删除终点";
    response->success = true;
    response->message = last_result_message_;
  } else if (mission_.targets.empty()) {
    response->success = false;
    response->message = "当前 mission 中没有已采集的目标点或终点";
  } else {
    mission_.targets.pop_back();
    if (mission_.targets.empty() && mission_.relative_goals.empty() && !mission_.has_goal) {
      mission_.mission_name.clear();
      execution_state_ = ExecutionState::kIdle;
    }
    last_result_message_ = "已删除最后一个目标点";
    response->success = true;
    response->message = last_result_message_;
  }
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::clear_loaded_mission_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  clear_loaded_mission();
  last_result_message_ = "已清空当前 mission";
  response->success = true;
  response->message = last_result_message_;
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::save_mission_callback(
    const std::shared_ptr<mission_manager::srv::SaveMission::Request> request,
    std::shared_ptr<mission_manager::srv::SaveMission::Response> response) {
  std::string saved_path;
  std::string error_message;
  response->success =
      save_current_mission(request->mission_name, saved_path, error_message);
  response->message = response->success ? last_result_message_ : error_message;
  response->file_path = saved_path;
  publish_status();
}

void MissionManagerNode::load_mission_callback(
    const std::shared_ptr<mission_manager::srv::LoadMission::Request> request,
    std::shared_ptr<mission_manager::srv::LoadMission::Response> response) {
  MissionDefinition mission;
  std::string file_path;
  std::string yaml_text;
  std::string error_message;
  if (load_named_mission(request->mission_name, mission, file_path, yaml_text,
                         error_message)) {
    mission_ = mission;
    clear_draft_waypoints();
    execution_state_ = ExecutionState::kMissionLoaded;
    last_loaded_path_ = file_path;
    last_result_message_ = "已加载 mission: " + mission_.mission_name;
    persist_last_mission_name(mission_.mission_name);
    response->success = true;
    response->message = last_result_message_;
    response->file_path = file_path;
    response->mission_yaml = yaml_text;
  } else {
    response->success = false;
    response->message = error_message;
  }
  publish_status();
  publish_markers();
  publish_preview_path();
}

void MissionManagerNode::update_mission_point_callback(
    const std::shared_ptr<mission_manager::srv::UpdateMissionPoint::Request> request,
    std::shared_ptr<mission_manager::srv::UpdateMissionPoint::Response> response) {
  if (
      !std::isfinite(request->x) || !std::isfinite(request->y) ||
      !std::isfinite(request->yaw)) {
    response->success = false;
    response->message = "x/y/yaw 必须是有限数值";
    return;
  }

  const auto target_count = static_cast<int32_t>(mission_.targets.size());
  MissionPose updated_pose;
  updated_pose.x = request->x;
  updated_pose.y = request->y;
  updated_pose.yaw = request->yaw;

  if (request->target_index >= 0 && request->target_index < target_count) {
    mission_.targets[static_cast<std::size_t>(request->target_index)].pose = updated_pose;
    last_result_message_ =
        "已更新目标点 target_index=" + std::to_string(request->target_index);
  } else if (request->target_index == target_count && mission_.has_goal) {
    mission_.goal = updated_pose;
    last_result_message_ =
        "已更新终点 target_index=" + std::to_string(request->target_index);
  } else {
    response->success = false;
    response->message = "target_index 无效或对应终点不存在";
    return;
  }

  response->success = true;
  response->message = last_result_message_;
  publish_status();
  publish_markers();
  publish_preview_path();
}

bool MissionManagerNode::capture_current_pose(MissionPose& pose_out,
                                              std::string& error_message) const {
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (!odom_received_) {
    error_message =
        "尚未收到 /Odometry 或无法换算到 " + planning_frame_id() + "，无法采集当前位姿";
    return false;
  }
  return capture_pose_from_recent_window(pose_out, error_message);
}

bool MissionManagerNode::capture_pose_from_recent_window(
    MissionPose& pose_out, std::string& error_message) const {
  if (pose_buffer_.empty()) {
    if (!odom_received_) {
      error_message =
          "尚未收到 /Odometry 或无法换算到 " + planning_frame_id() + "，无法采集当前位姿";
      return false;
    }
    pose_out = current_pose_;
    return true;
  }

  const double n = static_cast<double>(pose_buffer_.size());
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_sin = 0.0;
  double sum_cos = 0.0;
  for (const auto& sample : pose_buffer_) {
    sum_x += sample.pose.x;
    sum_y += sample.pose.y;
    sum_sin += std::sin(sample.pose.yaw);
    sum_cos += std::cos(sample.pose.yaw);
  }
  pose_out.x = sum_x / n;
  pose_out.y = sum_y / n;
  pose_out.yaw = std::atan2(sum_sin / n, sum_cos / n);
  return true;
}

bool MissionManagerNode::wait_and_capture_pose(MissionPose& pose_out,
                                               std::string& error_message) {
  std::unique_lock<std::mutex> lock(pose_mutex_);
  if (!odom_received_) {
    error_message =
        "尚未收到 /Odometry 或无法换算到 " + planning_frame_id() + "，无法采集当前位姿";
    return false;
  }

  pose_buffer_.clear();
  const auto deadline = std::chrono::steady_clock::now() + pose_avg_duration_;
  pose_cv_.wait_until(lock, deadline, [this]() { return !pose_buffer_.empty(); });
  while (std::chrono::steady_clock::now() < deadline) {
    pose_cv_.wait_until(lock, deadline);
  }

  if (!odom_received_) {
    error_message = "采样期间未收到有效 /Odometry，无法采集当前位姿";
    return false;
  }
  if (pose_buffer_.empty()) {
    error_message = "采样期间未收到有效 /Odometry，无法采集当前位姿";
    return false;
  }

  return capture_pose_from_recent_window(pose_out, error_message);
}

MissionPose MissionManagerNode::pose_from_msg_in_map(
    const nav_msgs::msg::Odometry& msg, bool& success, std::string& error_message) const {
  success = false;
  if (msg.header.frame_id.empty()) {
    error_message =
        "Odometry header.frame_id 为空，无法换算到 " + planning_frame_id();
    return MissionPose {};
  }

  geometry_msgs::msg::PoseStamped source_pose;
  source_pose.header = msg.header;
  source_pose.pose = msg.pose.pose;

  try {
    const auto map_to_source =
        tf_buffer_->lookupTransform(planning_frame_id(), msg.header.frame_id, tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped map_pose_msg;
    tf2::doTransform(source_pose, map_pose_msg, map_to_source);

    MissionPose pose;
    pose.x = map_pose_msg.pose.position.x;
    pose.y = map_pose_msg.pose.position.y;
    pose.yaw = tf2::getYaw(map_pose_msg.pose.orientation);
    success = true;
    return pose;
  } catch (const tf2::TransformException& ex) {
    error_message =
        std::string("无法将当前位姿换算到 ") + planning_frame_id() + ": " + ex.what();
    return MissionPose {};
  }
}

std::string MissionManagerNode::planning_frame_id() const {
  if (
      (mission_.has_start || !mission_.targets.empty() || mission_.has_goal ||
       !mission_.relative_goals.empty()) &&
      !mission_.frame_id.empty()) {
    return mission_.frame_id;
  }
  if (!frame_id_.empty()) {
    return frame_id_;
  }
  if (!mission_.frame_id.empty()) {
    return mission_.frame_id;
  }
  return kDefaultFrameId;
}

bool MissionManagerNode::save_current_mission(const std::string& mission_name,
                                              std::string& saved_path,
                                              std::string& error_message) {
  if (mission_.targets.empty() && !mission_.has_goal && mission_.relative_goals.empty()) {
    error_message = "当前 mission 需要至少一个目标点、终点或 relative_goal 才能保存";
    return false;
  }
  if (mission_.has_goal && !mission_.has_start) {
    error_message = "当前 route mission 需要先采集起点";
    return false;
  }
  if (!draft_waypoints_.empty()) {
    error_message = "当前还有未附加到目标点或终点的途径点，请先采集目标点/终点或清空后再保存";
    return false;
  }

  mission_.frame_id = planning_frame_id();
  mission_.planning_mode = planning_mode_.empty() ? "straight_line" : planning_mode_;
  mission_.mission_name = mission_name.empty() ? mission_.mission_name : mission_name;
  if (mission_.mission_name.empty()) {
    error_message = "mission_name 不能为空";
    return false;
  }

  std::filesystem::create_directories(missions_dir_);
  saved_path = mission_file_path(mission_.mission_name);
  if (!save_mission_file(saved_path, mission_, error_message)) {
    return false;
  }
  last_saved_path_ = saved_path;
  last_result_message_ = "已保存 mission 到 " + saved_path;
  persist_last_mission_name(mission_.mission_name);
  return true;
}

bool MissionManagerNode::load_named_mission(const std::string& mission_name,
                                            MissionDefinition& mission_out,
                                            std::string& file_path_out,
                                            std::string& yaml_out,
                                            std::string& error_message) {
  if (mission_name.empty()) {
    error_message = "mission_name 不能为空";
    return false;
  }

  file_path_out = mission_file_path(mission_name);
  if (!load_mission_file(file_path_out, mission_out, error_message)) {
    return false;
  }
  yaml_out = mission_to_yaml_string(mission_out);
  return true;
}

void MissionManagerNode::auto_load_last_mission() {
  if (!auto_load_last_mission_) {
    return;
  }

  const std::string mission_name = read_last_mission_name();
  if (mission_name.empty()) {
    return;
  }

  MissionDefinition mission;
  std::string file_path;
  std::string yaml_text;
  std::string error_message;
  if (!load_named_mission(mission_name, mission, file_path, yaml_text, error_message)) {
    RCLCPP_WARN(this->get_logger(), "failed to auto-load last mission [%s]: %s",
                mission_name.c_str(), error_message.c_str());
    return;
  }

  mission_ = mission;
  clear_draft_waypoints();
  execution_state_ = ExecutionState::kMissionLoaded;
  last_loaded_path_ = file_path;
  last_result_message_ = "已自动加载 mission: " + mission_.mission_name;
  RCLCPP_INFO(this->get_logger(), "auto-loaded last mission [%s] from %s",
              mission_.mission_name.c_str(), file_path.c_str());
}

std::string MissionManagerNode::read_last_mission_name() const {
  std::ifstream stream(last_mission_record_path());
  if (!stream.is_open()) {
    return "";
  }

  std::stringstream buffer;
  buffer << stream.rdbuf();
  return trim_copy(buffer.str());
}

void MissionManagerNode::persist_last_mission_name(const std::string& mission_name) const {
  const std::string trimmed_mission_name = trim_copy(mission_name);
  if (trimmed_mission_name.empty()) {
    return;
  }

  try {
    std::filesystem::create_directories(missions_dir_);
    std::ofstream stream(last_mission_record_path());
    if (!stream.is_open()) {
      RCLCPP_WARN(this->get_logger(), "failed to open last mission record: %s",
                  last_mission_record_path().c_str());
      return;
    }
    stream << trimmed_mission_name << '\n';
  } catch (const std::exception& exc) {
    RCLCPP_WARN(this->get_logger(), "failed to persist last mission name [%s]: %s",
                trimmed_mission_name.c_str(), exc.what());
  }
}



void MissionManagerNode::clear_draft_waypoints() {
  draft_waypoints_.clear();
}

void MissionManagerNode::clear_loaded_mission() {
  stop_execution("已清空任务并停止执行", ExecutionState::kIdle);
  mission_ = MissionDefinition {};
  mission_.frame_id = planning_frame_id();
  mission_.planning_mode = planning_mode_;
  clear_draft_waypoints();
  last_loaded_path_.clear();
  last_saved_path_.clear();
}

void MissionManagerNode::stop_execution(const std::string& final_message,
                                        ExecutionState new_state) {
  execution_state_ = new_state;
  last_result_message_ = final_message;
  publish_status();
}

void MissionManagerNode::publish_status() {
  std_msgs::msg::String msg;
  std::ostringstream oss;
  oss << "{";
  oss << "\"mission_name\":\"" << json_escape(mission_.mission_name) << "\",";
  oss << "\"frame_id\":\"" << json_escape(mission_.frame_id) << "\",";
  oss << "\"planning_mode\":\"" << json_escape(mission_.planning_mode) << "\",";
  oss << "\"planning_frame_id\":\"" << json_escape(planning_frame_id()) << "\",";
  oss << "\"targets_count\":" << mission_.targets.size() << ",";
  oss << "\"executable_count\":" << (mission_.targets.size() + (mission_.has_goal ? 1U : 0U))
      << ",";
  oss << "\"draft_has_start\":" << (mission_.has_start ? "true" : "false") << ",";
  oss << "\"draft_has_goal\":" << (mission_.has_goal ? "true" : "false") << ",";
  oss << "\"draft_waypoint_count\":" << draft_waypoints_.size() << ",";
  oss << "\"execution_state\":\"" << execution_state_string() << "\",";
  oss << "\"last_result_message\":\"" << json_escape(last_result_message_) << "\",";
  oss << "\"last_saved_path\":\"" << json_escape(last_saved_path_) << "\",";
  oss << "\"last_loaded_path\":\"" << json_escape(last_loaded_path_) << "\",";
  oss << "\"start\":" << (mission_.has_start ? pose_json(mission_.start) : std::string("null"))
      << ",";
  oss << "\"targets\":[";
  for (std::size_t index = 0; index < mission_.targets.size(); ++index) {
    const auto& target = mission_.targets[index];
    if (index > 0) {
      oss << ",";
    }
    oss << "{";
    oss << "\"target_index\":" << index << ",";
    oss << "\"waypoint_count\":" << target.waypoints.size() << ",";
    oss << "\"pose\":" << pose_json(target.pose);
    oss << "}";
  }
  oss << "],";
  oss << "\"goal\":" << (mission_.has_goal ? pose_json(mission_.goal) : std::string("null"))
      << ",";
  oss << "\"goal_waypoint_count\":" << mission_.goal_waypoints.size() << ",";
  oss << "\"executable_points\":[";
  for (std::size_t index = 0; index < mission_.targets.size(); ++index) {
    const auto& target = mission_.targets[index];
    if (index > 0) {
      oss << ",";
    }
    oss << "{";
    oss << "\"target_index\":" << index << ",";
    oss << "\"type\":\"target\",";
    oss << "\"waypoint_count\":" << target.waypoints.size() << ",";
    oss << "\"pose\":" << pose_json(target.pose);
    oss << "}";
  }
  if (mission_.has_goal) {
    if (!mission_.targets.empty()) {
      oss << ",";
    }
    oss << "{";
    oss << "\"target_index\":" << mission_.targets.size() << ",";
    oss << "\"type\":\"goal\",";
    oss << "\"waypoint_count\":" << mission_.goal_waypoints.size() << ",";
    oss << "\"pose\":" << pose_json(mission_.goal);
    oss << "}";
  }
  oss << "],";
  oss << "\"draft_start\":"
      << (mission_.has_start ? pose_json(mission_.start) : std::string("null")) << ",";
  oss << "\"draft_goal\":"
      << (mission_.has_goal ? pose_json(mission_.goal) : std::string("null"));
  oss << "}";
  msg.data = oss.str();
  status_pub_->publish(msg);
}

void MissionManagerNode::publish_markers() {
  visualization_msgs::msg::MarkerArray marker_array;
  int marker_id = 0;
  const auto stamp = this->now();

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = planning_frame_id();
  clear_marker.header.stamp = stamp;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  auto add_point_marker = [&](const MissionPose& pose, const std::string& ns, int r, int g,
                              int b, double scale) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_id();
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = pose.x;
    marker.pose.position.y = pose.y;
    marker.pose.position.z = 0.05;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = static_cast<float>(r) / 255.0F;
    marker.color.g = static_cast<float>(g) / 255.0F;
    marker.color.b = static_cast<float>(b) / 255.0F;
    marker.color.a = 0.95F;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    marker_array.markers.push_back(marker);
  };

  // 箭头 marker，用于可视化各点的 yaw 朝向
  auto add_arrow_marker = [&](const MissionPose& pose, const std::string& ns, int r, int g,
                              int b, double length) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_id();
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point start_point;
    start_point.x = pose.x;
    start_point.y = pose.y;
    start_point.z = 0.06;
    geometry_msgs::msg::Point end_point;
    end_point.x = pose.x + length * std::cos(pose.yaw);
    end_point.y = pose.y + length * std::sin(pose.yaw);
    end_point.z = 0.06;
    marker.points.push_back(start_point);
    marker.points.push_back(end_point);
    marker.scale.x = 0.04;  // 箭头杆粗细
    marker.scale.y = 0.08;  // 箭头头部宽度
    marker.scale.z = 0.0;
    marker.color.r = static_cast<float>(r) / 255.0F;
    marker.color.g = static_cast<float>(g) / 255.0F;
    marker.color.b = static_cast<float>(b) / 255.0F;
    marker.color.a = 0.95F;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    marker_array.markers.push_back(marker);
  };

  auto add_line_marker = [&](const std::vector<MissionPose>& chain, const std::string& ns, int r,
                             int g, int b) {
    if (chain.size() < 2) {
      return;
    }
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_id();
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color.r = static_cast<float>(r) / 255.0F;
    marker.color.g = static_cast<float>(g) / 255.0F;
    marker.color.b = static_cast<float>(b) / 255.0F;
    marker.color.a = 0.9F;
    for (const auto& pose : chain) {
      geometry_msgs::msg::Point point;
      point.x = pose.x;
      point.y = pose.y;
      point.z = 0.02;
      marker.points.push_back(point);
    }
    marker_array.markers.push_back(marker);
  };

  auto add_target_index_label_marker = [&](const MissionPose& pose, std::size_t target_index) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_id();
    marker.header.stamp = stamp;
    marker.ns = "mission_target_index_labels";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = pose.x;
    marker.pose.position.y = pose.y;
    marker.pose.position.z = 0.55;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.35;
    marker.color.r = 1.0F;
    marker.color.g = 1.0F;
    marker.color.b = 1.0F;
    marker.color.a = 0.95F;
    marker.text = std::to_string(target_index);
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    marker_array.markers.push_back(marker);
  };

  if (mission_.has_start) {
    add_point_marker(mission_.start, "mission_start", 0, 200, 0, 0.18);
    add_arrow_marker(mission_.start, "mission_start_heading", 0, 200, 0, 0.25);
  }

  for (std::size_t index = 0; index < mission_.targets.size(); ++index) {
    const auto& target = mission_.targets[index];
    std::vector<MissionPose> chain;
    if (index == 0U && mission_.has_start) {
      chain.push_back(mission_.start);
    } else if (index > 0U) {
      chain.push_back(mission_.targets[index - 1U].pose);
    }
    chain.insert(chain.end(), target.waypoints.begin(), target.waypoints.end());
    chain.push_back(target.pose);
    add_line_marker(chain, "mission_targets", 0, 180, 255);
    for (const auto& waypoint : target.waypoints) {
      add_point_marker(waypoint, "mission_waypoint", 64, 160, 255, 0.14);
      add_arrow_marker(waypoint, "mission_waypoint_heading", 64, 160, 255, 0.20);
    }
    add_point_marker(target.pose, "mission_target", 255, 80, 220, 0.20);
    add_arrow_marker(target.pose, "mission_target_heading", 255, 80, 220, 0.25);
    add_target_index_label_marker(target.pose, index);
  }

  if (mission_.has_goal) {
    std::vector<MissionPose> chain;
    if (!mission_.targets.empty()) {
      chain.push_back(mission_.targets.back().pose);
    } else if (mission_.has_start) {
      chain.push_back(mission_.start);
    }
    chain.insert(chain.end(), mission_.goal_waypoints.begin(), mission_.goal_waypoints.end());
    chain.push_back(mission_.goal);
    add_line_marker(chain, "mission_goal_route", 0, 220, 120);
    for (const auto& waypoint : mission_.goal_waypoints) {
      add_point_marker(waypoint, "mission_goal_waypoint", 64, 220, 160, 0.14);
      add_arrow_marker(waypoint, "mission_goal_waypoint_heading", 64, 220, 160, 0.20);
    }
    add_point_marker(mission_.goal, "mission_goal", 255, 32, 32, 0.18);
    add_arrow_marker(mission_.goal, "mission_goal_heading", 255, 32, 32, 0.25);
    add_target_index_label_marker(mission_.goal, mission_.targets.size());
  }

  if (mission_.has_start || !draft_waypoints_.empty()) {
    std::vector<MissionPose> draft_chain;
    if (!mission_.targets.empty()) {
      draft_chain.push_back(mission_.targets.back().pose);
    } else if (mission_.has_start) {
      draft_chain.push_back(mission_.start);
    }
    draft_chain.insert(draft_chain.end(), draft_waypoints_.begin(), draft_waypoints_.end());
    add_line_marker(draft_chain, "draft_route", 255, 200, 0);
    for (const auto& waypoint : draft_waypoints_) {
      add_point_marker(waypoint, "draft_waypoint", 255, 180, 0, 0.14);
      add_arrow_marker(waypoint, "draft_waypoint_heading", 255, 180, 0, 0.20);
    }
  }

  markers_pub_->publish(marker_array);
}

void MissionManagerNode::publish_preview_path() {
  std::vector<MissionPose> chain;
  if (mission_.has_start) {
    chain.push_back(mission_.start);
  }
  for (const auto& target : mission_.targets) {
    chain.insert(chain.end(), target.waypoints.begin(), target.waypoints.end());
    chain.push_back(target.pose);
  }
  chain.insert(chain.end(), draft_waypoints_.begin(), draft_waypoints_.end());
  if (mission_.has_goal) {
    chain.insert(chain.end(), mission_.goal_waypoints.begin(), mission_.goal_waypoints.end());
    chain.push_back(mission_.goal);
  }
  preview_path_pub_->publish(build_path_from_points(chain, this->now(), planning_frame_id()));
}

std::string MissionManagerNode::execution_state_string() const {
  switch (execution_state_) {
    case ExecutionState::kIdle:
      return "idle";
    case ExecutionState::kMissionLoaded:
      return "mission_loaded";
  }
  return "unknown";
}


std::string MissionManagerNode::mission_file_path(const std::string& mission_name) const {
  return (std::filesystem::path(missions_dir_) /
          (sanitize_file_stem(mission_name) + ".yaml"))
      .string();
}

std::string MissionManagerNode::last_mission_record_path() const {
  return (std::filesystem::path(missions_dir_) / ".last_mission").string();
}


}  // namespace mission_manager

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mission_manager::MissionManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
