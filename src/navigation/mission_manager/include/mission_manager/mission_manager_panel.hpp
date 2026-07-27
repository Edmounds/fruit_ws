#ifndef MISSION_MANAGER__MISSION_MANAGER_PANEL_HPP_
#define MISSION_MANAGER__MISSION_MANAGER_PANEL_HPP_

#include <mutex>
#include <string>

#include <QJsonArray>
#include <QJsonObject>
#include <QSize>
#include <QTimer>

#include "mission_manager/srv/load_mission.hpp"
#include "mission_manager/srv/save_mission.hpp"
#include "mission_manager/srv/update_mission_point.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rc2026_interfaces/action/navigation_control.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

class QLineEdit;
class QPushButton;
class QPlainTextEdit;
class QTableWidget;
class QTableWidgetItem;

namespace mission_manager {

class MissionManagerPanel : public rviz_common::Panel {
 public:
  explicit MissionManagerPanel(QWidget* parent = nullptr);

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config& config) override;
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 private:
  void refresh_ui();
  void on_record_start();
  void on_record_waypoint();
  void on_record_target();
  void on_record_goal();
  void on_undo_last_step();
  void on_save_mission();
  void on_load_mission();
  void on_continue_navigation();
  void on_reset_navigation();
  void on_targets_table_item_changed(QTableWidgetItem* item);

 private:
  bool ensure_ros_interfaces();
  void build_ui();
  void set_feedback_text(const QString& text);
  void update_targets_table(const QJsonObject& status);
  void update_executable_points_table(const QJsonArray& executable_points);
  QJsonObject pose_for_target_index(int target_index) const;
  void restore_targets_table_from_status();
  void consume_pending_status_json();
  void restore_persisted_mission_name();
  void persist_current_mission_name() const;
  void persist_mission_name(const QString& mission_name) const;
  void maybe_auto_load_mission();
  void call_trigger_service(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client,
                            const QString& action_name);
  void handle_status_message(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr record_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr record_waypoint_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr record_target_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr record_goal_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr undo_waypoint_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr remove_last_target_client_;
  rclcpp::Client<mission_manager::srv::SaveMission>::SharedPtr save_mission_client_;
  rclcpp::Client<mission_manager::srv::LoadMission>::SharedPtr load_mission_client_;
  rclcpp::Client<mission_manager::srv::UpdateMissionPoint>::SharedPtr
      update_mission_point_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_navigation_client_;
  rclcpp_action::Client<rc2026_interfaces::action::NavigationControl>::SharedPtr
      navigation_control_client_;
  rclcpp_action::ClientGoalHandle<rc2026_interfaces::action::NavigationControl>::SharedPtr
      current_navigation_goal_handle_;

  QTimer* refresh_timer_ {nullptr};
  QLineEdit* mission_name_edit_ {nullptr};
  QTableWidget* targets_table_ {nullptr};
  QTableWidget* executable_points_table_ {nullptr};
  QPlainTextEdit* feedback_box_ {nullptr};
  QJsonArray executable_points_;
  QJsonObject latest_status_;
  QString status_mission_name_;
  int current_target_index_ {0};
  int draft_waypoint_count_ {0};
  int executable_count_ {0};
  bool navigation_goal_in_flight_ {false};
  bool pending_advance_navigation_index_ {false};
  bool pending_restore_targets_table_ {false};
  bool auto_load_requested_ {false};
  bool updating_targets_table_ {false};

  std::mutex pending_mutex_;
  std::string pending_status_json_;
  std::string pending_feedback_;
};

}  // namespace mission_manager

#endif  // MISSION_MANAGER__MISSION_MANAGER_PANEL_HPP_
