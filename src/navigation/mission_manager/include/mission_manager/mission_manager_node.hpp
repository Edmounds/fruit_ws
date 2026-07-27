#ifndef MISSION_MANAGER__MISSION_MANAGER_NODE_HPP_
#define MISSION_MANAGER__MISSION_MANAGER_NODE_HPP_

#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "mission_manager/mission_types.hpp"
#include "mission_manager/srv/load_mission.hpp"
#include "mission_manager/srv/save_mission.hpp"
#include "mission_manager/srv/update_mission_point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace mission_manager {

/// Mission 管理节点，负责 route 采集、Mission 文件 I/O、执行状态机以及 RViz 可视化。
///
/// 线程安全说明：本节点假定运行在 SingleThreadedExecutor 下，内部状态（execution_state_,
/// mission_, draft_* 等）不加锁。仅 current_pose_ 使用 pose_mutex_ 保护，因为 odom 回调
/// 可能与其他回调在不同时刻访问。如果改用 MultiThreadedExecutor，必须为所有共享状态加锁。
class MissionManagerNode : public rclcpp::Node {
 public:
  MissionManagerNode();

 private:
  enum class ExecutionState {
    kIdle,
    kMissionLoaded,
  };

  void declare_and_load_parameters();
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void record_start_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void record_waypoint_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void record_target_pose_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void record_goal_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void undo_waypoint_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void clear_draft_waypoints_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void remove_last_target_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void clear_loaded_mission_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void save_mission_callback(
      const std::shared_ptr<mission_manager::srv::SaveMission::Request> request,
      std::shared_ptr<mission_manager::srv::SaveMission::Response> response);
  void load_mission_callback(
      const std::shared_ptr<mission_manager::srv::LoadMission::Request> request,
      std::shared_ptr<mission_manager::srv::LoadMission::Response> response);
  void update_mission_point_callback(
      const std::shared_ptr<mission_manager::srv::UpdateMissionPoint::Request> request,
      std::shared_ptr<mission_manager::srv::UpdateMissionPoint::Response> response);

  bool capture_current_pose(MissionPose& pose_out, std::string& error_message) const;
  MissionPose pose_from_msg_in_map(const nav_msgs::msg::Odometry& msg,
                                   bool& success,
                                   std::string& error_message) const;
  std::string planning_frame_id() const;
  bool save_current_mission(const std::string& mission_name,
                            std::string& saved_path,
                            std::string& error_message);
  bool load_named_mission(const std::string& mission_name,
                          MissionDefinition& mission_out,
                          std::string& file_path_out,
                          std::string& yaml_out,
                          std::string& error_message);
  void auto_load_last_mission();
  std::string read_last_mission_name() const;
  void persist_last_mission_name(const std::string& mission_name) const;
  bool capture_pose_from_recent_window(MissionPose& pose_out, std::string& error_message) const;
  bool wait_and_capture_pose(MissionPose& pose_out, std::string& error_message);

  void clear_draft_waypoints();
  void clear_loaded_mission();
  void stop_execution(const std::string& final_message, ExecutionState new_state);
  void publish_status();
  void publish_markers();
  void publish_preview_path();
  std::string execution_state_string() const;
  std::string mission_file_path(const std::string& mission_name) const;
  std::string last_mission_record_path() const;

  std::string missions_dir_;
  std::string odom_topic_;
  std::string frame_id_ {"map"};
  std::string planning_mode_ {"straight_line"};
  bool auto_load_last_mission_ {true};

  mutable std::mutex pose_mutex_;
  std::condition_variable pose_cv_;
  MissionPose current_pose_;
  bool odom_received_ {false};

  struct TimedMissionPose {
    rclcpp::Time stamp;
    MissionPose pose;
  };

  std::deque<TimedMissionPose> pose_buffer_;
  std::chrono::milliseconds pose_avg_duration_ {500};
  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;

  MissionDefinition mission_;
  std::vector<MissionPose> draft_waypoints_;

  ExecutionState execution_state_ {ExecutionState::kIdle};
  std::string last_result_message_;
  std::string last_saved_path_;
  std::string last_loaded_path_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr preview_path_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr record_start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr record_waypoint_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr record_target_pose_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr record_goal_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr undo_waypoint_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_draft_waypoints_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remove_last_target_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_loaded_mission_srv_;
  rclcpp::Service<mission_manager::srv::SaveMission>::SharedPtr save_mission_srv_;
  rclcpp::Service<mission_manager::srv::LoadMission>::SharedPtr load_mission_srv_;
  rclcpp::Service<mission_manager::srv::UpdateMissionPoint>::SharedPtr
      update_mission_point_srv_;
};

}  // namespace mission_manager

#endif  // MISSION_MANAGER__MISSION_MANAGER_NODE_HPP_
