// Copyright 2026 fruit_ws team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FRUIT_WAYPOINT_RVIZ_PLUGIN__WAYPOINT_MAKER_PANEL_HPP_
#define FRUIT_WAYPOINT_RVIZ_PLUGIN__WAYPOINT_MAKER_PANEL_HPP_

#include <QComboBox>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QString>
#include <QTabWidget>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"

namespace fruit_waypoint_rviz_plugin
{

class WaypointMakerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WaypointMakerPanel(QWidget * parent = nullptr);
  ~WaypointMakerPanel() override = default;

  void onInitialize() override;

private Q_SLOTS:
  void loadTestFile();
  void recordTestWaypoint();
  void recordTestGoal();
  void undoTestPoint();
  void saveTestPoints();
  void runNextTest();
  void resetTestCursor();

private:
  void recordTestPoint(const QString & point_type);
  void sendCommand(const QJsonObject & command);
  void onStateMessage(const std_msgs::msg::String::SharedPtr message);
  void applyState(const QString & state_json);
  void setBusy(bool busy);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscription_;

  QTabWidget * tabs_{nullptr};
  QLabel * status_label_{nullptr};
  QLabel * test_label_{nullptr};
  QLineEdit * test_file_edit_{nullptr};
  QLineEdit * test_name_edit_{nullptr};
  QComboBox * final_yaw_mode_combo_{nullptr};
  QPushButton * load_test_file_button_{nullptr};
  QPushButton * record_waypoint_button_{nullptr};
  QPushButton * record_goal_button_{nullptr};
  QPushButton * undo_test_button_{nullptr};
  QPushButton * save_test_button_{nullptr};
  QPushButton * run_next_button_{nullptr};
  QPushButton * reset_cursor_button_{nullptr};
};

}  // namespace fruit_waypoint_rviz_plugin

#endif  // FRUIT_WAYPOINT_RVIZ_PLUGIN__WAYPOINT_MAKER_PANEL_HPP_
