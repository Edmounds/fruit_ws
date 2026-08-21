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

#include "fruit_waypoint_rviz_plugin/waypoint_maker_panel.hpp"

#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QMetaObject>
#include <QVBoxLayout>

#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace fruit_waypoint_rviz_plugin
{

WaypointMakerPanel::WaypointMakerPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root_layout = new QVBoxLayout(this);
  status_label_ = new QLabel("Waiting for waypoint recorder state", this);
  status_label_->setWordWrap(true);
  root_layout->addWidget(status_label_);

  tabs_ = new QTabWidget(this);
  root_layout->addWidget(tabs_);

  auto * test_tab = new QWidget(this);
  auto * test_layout = new QVBoxLayout(test_tab);
  auto * test_form = new QFormLayout();
  test_file_edit_ = new QLineEdit(test_tab);
  test_file_edit_->setReadOnly(true);
  final_yaw_mode_combo_ = new QComboBox(test_tab);
  final_yaw_mode_combo_->addItem("Blend on path", "blend_on_path");
  final_yaw_mode_combo_->addItem("Rotate at goal", "rotate_at_goal");
  load_test_file_button_ = new QPushButton("Load Waypoint YAML", test_tab);
  test_form->addRow("YAML", test_file_edit_);
  test_form->addRow("Final yaw", final_yaw_mode_combo_);
  test_layout->addLayout(test_form);
  test_layout->addWidget(load_test_file_button_);

  next_name_label_ = new QLabel("Next name: waiting for recorder", test_tab);
  next_name_label_->setWordWrap(true);
  test_layout->addWidget(next_name_label_);

  test_label_ = new QLabel("Loaded waypoints: 0", test_tab);
  test_label_->setWordWrap(true);
  test_layout->addWidget(test_label_);

  record_waypoint_button_ = new QPushButton("Set Waypoint", test_tab);
  record_goal_button_ = new QPushButton("Set Goal", test_tab);
  undo_test_button_ = new QPushButton("Undo Point", test_tab);
  save_test_button_ = new QPushButton("Save Waypoint YAML", test_tab);
  run_next_button_ = new QPushButton("Run Next Goal", test_tab);
  reset_cursor_button_ = new QPushButton("Reset Run Cursor", test_tab);

  auto * test_record_row = new QHBoxLayout();
  test_record_row->addWidget(record_waypoint_button_);
  test_record_row->addWidget(record_goal_button_);
  test_record_row->addWidget(undo_test_button_);
  test_layout->addLayout(test_record_row);

  auto * test_run_row = new QHBoxLayout();
  test_run_row->addWidget(run_next_button_);
  test_run_row->addWidget(reset_cursor_button_);
  test_layout->addLayout(test_run_row);
  test_layout->addWidget(save_test_button_);
  test_layout->addStretch();
  tabs_->addTab(test_tab, "Route");

  connect(
    load_test_file_button_, &QPushButton::clicked, this,
    &WaypointMakerPanel::loadTestFile);
  connect(
    record_waypoint_button_, &QPushButton::clicked, this,
    &WaypointMakerPanel::recordTestWaypoint);
  connect(
    record_goal_button_, &QPushButton::clicked, this,
    &WaypointMakerPanel::recordTestGoal);
  connect(undo_test_button_, &QPushButton::clicked, this, &WaypointMakerPanel::undoTestPoint);
  connect(save_test_button_, &QPushButton::clicked, this, &WaypointMakerPanel::saveTestPoints);
  connect(run_next_button_, &QPushButton::clicked, this, &WaypointMakerPanel::runNextTest);
  connect(reset_cursor_button_, &QPushButton::clicked, this, &WaypointMakerPanel::resetTestCursor);
}

void WaypointMakerPanel::onInitialize()
{
  auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    status_label_->setText("RViz ROS node is unavailable");
    setBusy(true);
    return;
  }
  node_ = node_abstraction->get_raw_node();
  command_publisher_ = node_->create_publisher<std_msgs::msg::String>(
    "/fruit/waypoint_maker/command",
    rclcpp::QoS(10));
  state_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/fruit/waypoint_maker/state",
    rclcpp::QoS(10).transient_local().reliable(),
    [this](std_msgs::msg::String::SharedPtr message) {
      this->onStateMessage(std::move(message));
    });
}

void WaypointMakerPanel::loadTestFile()
{
  const QString selected = QFileDialog::getOpenFileName(
    this,
    "Load waypoint-maker YAML",
    test_file_edit_->text(),
    "YAML files (*.yaml *.yml)");
  if (selected.isEmpty()) {
    return;
  }
  test_file_edit_->setText(selected);
  sendCommand({{"command", "load_test_file"}, {"path", selected}});
}

void WaypointMakerPanel::recordTestWaypoint()
{
  recordTestPoint("waypoint");
}

void WaypointMakerPanel::recordTestGoal()
{
  recordTestPoint("goal");
}

void WaypointMakerPanel::recordTestPoint(const QString & point_type)
{
  QJsonObject command;
  command["command"] = "record";
  command["mode"] = "test";
  command["auto_name"] = true;
  command["point_type"] = point_type;
  sendCommand(command);
}

void WaypointMakerPanel::undoTestPoint()
{
  sendCommand({{"command", "undo"}, {"mode", "test"}});
}

void WaypointMakerPanel::saveTestPoints()
{
  sendCommand({{"command", "save"}, {"mode", "test"}});
}

void WaypointMakerPanel::runNextTest()
{
  sendCommand(
    {
      {"command", "run_next_test"},
      {"final_yaw_mode", final_yaw_mode_combo_->currentData().toString()},
    });
}

void WaypointMakerPanel::resetTestCursor()
{
  sendCommand({{"command", "reset_test_cursor"}});
}

void WaypointMakerPanel::sendCommand(const QJsonObject & command)
{
  if (!command_publisher_) {
    status_label_->setText("Waypoint command publisher is not ready");
    return;
  }
  std_msgs::msg::String message;
  message.data = QJsonDocument(command).toJson(QJsonDocument::Compact).toStdString();
  command_publisher_->publish(message);
}

void WaypointMakerPanel::onStateMessage(const std_msgs::msg::String::SharedPtr message)
{
  const QString state_json = QString::fromStdString(message->data);
  QMetaObject::invokeMethod(
    this,
    [this, state_json]() {
      this->applyState(state_json);
    },
    Qt::QueuedConnection);
}

void WaypointMakerPanel::applyState(const QString & state_json)
{
  QJsonParseError error;
  const QJsonDocument document = QJsonDocument::fromJson(state_json.toUtf8(), &error);
  if (error.error != QJsonParseError::NoError || !document.isObject()) {
    status_label_->setText("Invalid waypoint recorder state");
    return;
  }

  const QJsonObject root = document.object();
  const QString status = root.value("status").toString("idle");
  const QString message = root.value("message").toString();
  const QString display_status = status == "running_test" ? "running_route" : status;
  status_label_->setText(display_status + ": " + message);
  setBusy(
    status == "sampling" || status == "running_route" || status == "running_test");

  QJsonObject test = root.value("waypoints").toObject();
  if (test.isEmpty()) {
    test = root.value("test").toObject();
  }
  const QString output_path = test.value("output_path").toString();
  if (!output_path.isEmpty()) {
    test_file_edit_->setText(output_path);
  }
  const QString next_goal = test.value("next_goal").toString();
  const QString final_yaw_mode = test.value("final_yaw_mode").toString();
  const QString auto_name_prefix = test.value("auto_name_prefix").toString();
  const QString next_name = test.value("next_name").toString();
  const int final_yaw_mode_index = final_yaw_mode_combo_->findData(final_yaw_mode);
  if (final_yaw_mode_index >= 0) {
    final_yaw_mode_combo_->setCurrentIndex(final_yaw_mode_index);
  }
  test_label_->setText(
    QString("Loaded waypoints: %1\nRun cursor: %2\nNext goal: %3\nFinal yaw: %4")
    .arg(test.value("count").toInt())
    .arg(test.value("cursor").toInt())
    .arg(next_goal.isEmpty() ? "none" : next_goal)
    .arg(final_yaw_mode_combo_->currentText()));
  if (next_name.isEmpty()) {
    next_name_label_->setText(
      auto_name_prefix.isEmpty()
      ? "Next name: unavailable (set auto_name_prefix)"
      : "Next name: none");
  } else {
    next_name_label_->setText(QString("Next name: %1").arg(next_name));
  }
}

void WaypointMakerPanel::setBusy(bool busy)
{
  load_test_file_button_->setDisabled(busy);
  record_waypoint_button_->setDisabled(busy);
  record_goal_button_->setDisabled(busy);
  undo_test_button_->setDisabled(busy);
  save_test_button_->setDisabled(busy);
  run_next_button_->setDisabled(busy);
  reset_cursor_button_->setDisabled(busy);
  final_yaw_mode_combo_->setDisabled(busy);
}

}  // namespace fruit_waypoint_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  fruit_waypoint_rviz_plugin::WaypointMakerPanel,
  rviz_common::Panel)
