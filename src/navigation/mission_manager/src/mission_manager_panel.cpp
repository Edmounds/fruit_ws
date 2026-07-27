#include "mission_manager/mission_manager_panel.hpp"

#include <cmath>
#include <utility>

#include <QAbstractItemView>
#include <QFrame>
#include <QGridLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QStringList>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>
#include <QWidget>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace mission_manager {

namespace {

constexpr int kTargetIndexRole = Qt::UserRole + 1;
constexpr int kPointTypeRole = Qt::UserRole + 2;

QString pose_to_compact_text(const QJsonObject& pose) {
  return QString("(%1, %2, %3)")
      .arg(pose.value("x").toDouble(), 0, 'f', 2)
      .arg(pose.value("y").toDouble(), 0, 'f', 2)
      .arg(pose.value("yaw").toDouble(), 0, 'f', 2);
}

QString pose_value_text(const QJsonObject& pose, const char* field_name) {
  return QString::number(pose.value(field_name).toDouble(), 'f', 4);
}

}  // namespace

MissionManagerPanel::MissionManagerPanel(QWidget* parent)
    : rviz_common::Panel(parent), refresh_timer_(new QTimer(this)) {
  build_ui();
  restore_persisted_mission_name();
  connect(refresh_timer_, &QTimer::timeout, this, &MissionManagerPanel::refresh_ui);
  refresh_timer_->start(200);
}

void MissionManagerPanel::onInitialize() {
  ensure_ros_interfaces();
}

void MissionManagerPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("mission_name", mission_name_edit_->text());
  persist_current_mission_name();
}

void MissionManagerPanel::load(const rviz_common::Config& config) {
  rviz_common::Panel::load(config);
  restore_persisted_mission_name();
  if (!mission_name_edit_->text().trimmed().isEmpty()) {
    return;
  }

  QString mission_name;
  if (config.mapGetString("mission_name", &mission_name)) {
    mission_name_edit_->setText(mission_name);
  }
}

QSize MissionManagerPanel::sizeHint() const {
  return QSize(360, 600);
}

QSize MissionManagerPanel::minimumSizeHint() const {
  return QSize(240, 180);
}

void MissionManagerPanel::refresh_ui() {
  if (ensure_ros_interfaces()) {
    maybe_auto_load_mission();
  }
  consume_pending_status_json();
}

void MissionManagerPanel::on_record_start() {
  call_trigger_service(record_start_client_, "起点");
}

void MissionManagerPanel::on_record_waypoint() {
  call_trigger_service(record_waypoint_client_, "途径点");
}

void MissionManagerPanel::on_record_target() {
  call_trigger_service(record_target_client_, "目标点");
}

void MissionManagerPanel::on_record_goal() {
  call_trigger_service(record_goal_client_, "终点");
}

void MissionManagerPanel::on_undo_last_step() {
  if (draft_waypoint_count_ > 0) {
    call_trigger_service(undo_waypoint_client_, "撤销途径点");
    return;
  }

  if (executable_count_ > 0) {
    call_trigger_service(remove_last_target_client_, "撤销目标点/终点");
    return;
  }

  set_feedback_text("没有可撤销的途径点、目标点或终点");
}

void MissionManagerPanel::on_save_mission() {
  const std::string mission_name = mission_name_edit_->text().trimmed().toStdString();
  if (mission_name.empty()) {
    set_feedback_text("请先填写 mission 名称");
    return;
  }
  auto_load_requested_ = true;
  persist_current_mission_name();
  if (!raw_node_) {
    set_feedback_text("RViz ROS 节点尚未初始化");
    return;
  }
  if (!save_mission_client_->wait_for_service(std::chrono::milliseconds(250))) {
    set_feedback_text("等待 /save_mission 服务超时");
    return;
  }
  auto request = std::make_shared<mission_manager::srv::SaveMission::Request>();
  request->mission_name = mission_name;
  save_mission_client_->async_send_request(
      request,
      [this](rclcpp::Client<mission_manager::srv::SaveMission>::SharedFuture future) {
        const auto response = future.get();
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_feedback_ =
            response->success ? ("保存成功: " + response->file_path) : response->message;
      });
}

void MissionManagerPanel::on_load_mission() {
  const std::string mission_name = mission_name_edit_->text().trimmed().toStdString();
  if (mission_name.empty()) {
    set_feedback_text("请先填写 mission 名称");
    return;
  }
  auto_load_requested_ = true;
  current_target_index_ = 0;
  update_executable_points_table(executable_points_);
  persist_current_mission_name();
  if (!raw_node_) {
    set_feedback_text("RViz ROS 节点尚未初始化");
    return;
  }
  if (!load_mission_client_->wait_for_service(std::chrono::milliseconds(250))) {
    set_feedback_text("等待 /load_mission 服务超时");
    return;
  }
  auto request = std::make_shared<mission_manager::srv::LoadMission::Request>();
  request->mission_name = mission_name;
  load_mission_client_->async_send_request(
      request,
      [this](rclcpp::Client<mission_manager::srv::LoadMission>::SharedFuture future) {
        const auto response = future.get();
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_feedback_ =
            response->success ? ("加载成功: " + response->file_path) : response->message;
      });
}

void MissionManagerPanel::on_continue_navigation() {
  const std::string mission_name = mission_name_edit_->text().trimmed().toStdString();
  if (mission_name.empty()) {
    set_feedback_text("请先填写 mission 名称");
    return;
  }
  if (current_target_index_ >= executable_points_.size()) {
    set_feedback_text("导航已完成");
    return;
  }
  const QJsonObject executable_point =
      executable_points_.at(current_target_index_).toObject();
  const int target_index = executable_point.value("target_index").toInt(-1);
  if (target_index < 0) {
    set_feedback_text("当前导航 target_index 无效");
    return;
  }
  const QString entry_description = QString("target_index=%1").arg(target_index);

  auto_load_requested_ = true;
  persist_current_mission_name();
  if (!raw_node_) {
    set_feedback_text("RViz ROS 节点尚未初始化");
    return;
  }
  if (!navigation_control_client_) {
    set_feedback_text("NavigationControl Action Client 未初始化");
    return;
  }
  if (!navigation_control_client_->wait_for_action_server(std::chrono::seconds(3))) {
    set_feedback_text("等待 /navigation_control action 超时 (3s)");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (navigation_goal_in_flight_) {
      pending_feedback_ = "导航任务正在执行，请等待当前任务结束后再发送";
      return;
    }
    navigation_goal_in_flight_ = true;
  }

  rc2026_interfaces::action::NavigationControl::Goal goal;
  goal.mission_name = mission_name;
  goal.target_index = target_index;
  goal.relative_goal_name = "";

  rclcpp_action::Client<rc2026_interfaces::action::NavigationControl>::SendGoalOptions options;
  const int sequence_index = current_target_index_;
  const int sequence_size = executable_points_.size();
  const std::string entry_description_text = entry_description.toStdString();
  options.goal_response_callback =
      [this, mission_name, sequence_index, sequence_size, entry_description_text](
          auto goal_handle) {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (!goal_handle) {
      navigation_goal_in_flight_ = false;
      current_navigation_goal_handle_.reset();
      RCLCPP_WARN(raw_node_->get_logger(), "NavigationControl goal rejected: %s",
                  mission_name.c_str());
      pending_feedback_ = "导航任务被拒绝：已有导航任务正在执行";
      return;
    }
    current_navigation_goal_handle_ = goal_handle;
    RCLCPP_INFO(raw_node_->get_logger(), "NavigationControl goal accepted: %s",
                mission_name.c_str());
    pending_feedback_ =
        "已发送导航目标 " + std::to_string(sequence_index + 1) + "/" +
        std::to_string(sequence_size) + ": " + entry_description_text;
  };
  options.feedback_callback = [this](auto, const auto feedback) {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_feedback_ = feedback->execution_state + " [" +
                        std::to_string(feedback->active_target_index) + "] " +
                        feedback->message;
  };
  options.result_callback = [this](const auto& result) {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    navigation_goal_in_flight_ = false;
    current_navigation_goal_handle_.reset();
    pending_feedback_ = result.result ? result.result->message : "导航任务结束但结果为空";
    if (
        result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
        result.result->success) {
      pending_advance_navigation_index_ = true;
    }
    RCLCPP_INFO(raw_node_->get_logger(), "NavigationControl result: %s",
                pending_feedback_.c_str());
  };
  RCLCPP_INFO(raw_node_->get_logger(),
              "Sending NavigationControl goal: %s %s",
              mission_name.c_str(), entry_description_text.c_str());
  navigation_control_client_->async_send_goal(goal, options);
}

void MissionManagerPanel::on_reset_navigation() {
  current_target_index_ = 0;
  update_executable_points_table(executable_points_);
  call_trigger_service(reset_navigation_client_, "重置导航");
}

void MissionManagerPanel::on_targets_table_item_changed(QTableWidgetItem* item) {
  if (updating_targets_table_ || !item || item->column() < 3 || item->column() > 5) {
    return;
  }

  bool target_index_ok = false;
  const int target_index = item->data(kTargetIndexRole).toInt(&target_index_ok);
  const QString point_type = item->data(kPointTypeRole).toString();
  if (
      !target_index_ok || target_index < 0 ||
      (point_type != "target" && point_type != "goal")) {
    restore_targets_table_from_status();
    return;
  }

  bool parse_ok = false;
  const double value = item->text().trimmed().toDouble(&parse_ok);
  if (!parse_ok || !std::isfinite(value)) {
    set_feedback_text("坐标输入无效，请输入有限数值");
    restore_targets_table_from_status();
    return;
  }

  QJsonObject pose = pose_for_target_index(target_index);
  if (pose.isEmpty()) {
    set_feedback_text("无法找到对应目标点，已恢复表格");
    restore_targets_table_from_status();
    return;
  }

  const char* field_name = "x";
  if (item->column() == 4) {
    field_name = "y";
  } else if (item->column() == 5) {
    field_name = "yaw";
  }
  pose[field_name] = value;

  if (!raw_node_) {
    set_feedback_text("RViz ROS 节点尚未初始化");
    restore_targets_table_from_status();
    return;
  }
  if (
      !update_mission_point_client_ ||
      !update_mission_point_client_->wait_for_service(std::chrono::milliseconds(250))) {
    set_feedback_text("等待 /update_mission_point 服务超时");
    restore_targets_table_from_status();
    return;
  }

  auto request =
      std::make_shared<mission_manager::srv::UpdateMissionPoint::Request>();
  request->target_index = target_index;
  request->x = pose.value("x").toDouble();
  request->y = pose.value("y").toDouble();
  request->yaw = pose.value("yaw").toDouble();
  update_mission_point_client_->async_send_request(
      request,
      [this](
          rclcpp::Client<mission_manager::srv::UpdateMissionPoint>::SharedFuture future) {
        const auto response = future.get();
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_feedback_ = response->success
                                ? (response->message + "，点击保存写入文件")
                                : response->message;
        if (!response->success) {
          pending_restore_targets_table_ = true;
        }
      });
}

bool MissionManagerPanel::ensure_ros_interfaces() {
  if (raw_node_) {
    return true;
  }

  auto* display_context = getDisplayContext();
  if (!display_context) {
    return false;
  }

  auto ros_node_abstraction = display_context->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    return false;
  }

  raw_node_ = ros_node_abstraction->get_raw_node();
  if (!raw_node_) {
    return false;
  }

  record_start_client_ = raw_node_->create_client<std_srvs::srv::Trigger>("/record_start_pose");
  record_waypoint_client_ = raw_node_->create_client<std_srvs::srv::Trigger>("/record_waypoint");
  record_target_client_ =
      raw_node_->create_client<std_srvs::srv::Trigger>("/record_target_pose");
  record_goal_client_ = raw_node_->create_client<std_srvs::srv::Trigger>("/record_goal_pose");
  undo_waypoint_client_ = raw_node_->create_client<std_srvs::srv::Trigger>("/undo_waypoint");
  remove_last_target_client_ =
      raw_node_->create_client<std_srvs::srv::Trigger>("/remove_last_target");
  save_mission_client_ =
      raw_node_->create_client<mission_manager::srv::SaveMission>("/save_mission");
  load_mission_client_ =
      raw_node_->create_client<mission_manager::srv::LoadMission>("/load_mission");
  update_mission_point_client_ =
      raw_node_->create_client<mission_manager::srv::UpdateMissionPoint>(
          "/update_mission_point");
  reset_navigation_client_ =
      raw_node_->create_client<std_srvs::srv::Trigger>("/reset_navigation");
  navigation_control_client_ =
      rclcpp_action::create_client<rc2026_interfaces::action::NavigationControl>(
          raw_node_, "/navigation_control");

  rclcpp::QoS status_qos(rclcpp::KeepLast(1));
  status_qos.transient_local();
  status_qos.reliable();
  status_sub_ = raw_node_->create_subscription<std_msgs::msg::String>(
      "/mission_manager/status", status_qos,
      std::bind(&MissionManagerPanel::handle_status_message, this, std::placeholders::_1));
  return true;
}

void MissionManagerPanel::build_ui() {
  auto* panel_layout = new QVBoxLayout(this);
  panel_layout->setContentsMargins(0, 0, 0, 0);

  auto* scroll_area = new QScrollArea(this);
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);
  scroll_area->setMinimumSize(0, 0);
  scroll_area->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
  scroll_area->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  auto* content_widget = new QWidget(scroll_area);
  content_widget->setMinimumSize(0, 0);
  auto* main_layout = new QVBoxLayout(content_widget);
  main_layout->setContentsMargins(4, 4, 4, 4);
  main_layout->setSpacing(4);

  auto* controls_layout = new QGridLayout();
  controls_layout->setContentsMargins(0, 0, 0, 0);
  controls_layout->setHorizontalSpacing(4);
  controls_layout->setVerticalSpacing(4);
  mission_name_edit_ = new QLineEdit(content_widget);
  mission_name_edit_->setPlaceholderText("mission name");
  controls_layout->addWidget(mission_name_edit_, 0, 0, 1, 2);

  auto* record_start_button = new QPushButton("起点", content_widget);
  auto* record_waypoint_button = new QPushButton("途径点", content_widget);
  auto* record_target_button = new QPushButton("目标点", content_widget);
  auto* record_goal_button = new QPushButton("终点", content_widget);
  auto* undo_last_step_button = new QPushButton("撤销", content_widget);
  auto* save_button = new QPushButton("保存", content_widget);
  auto* load_button = new QPushButton("加载", content_widget);
  auto* continue_navigation_button = new QPushButton("继续导航", content_widget);
  auto* reset_navigation_button = new QPushButton("重置导航", content_widget);
  controls_layout->addWidget(record_start_button, 1, 0);
  controls_layout->addWidget(record_waypoint_button, 1, 1);
  controls_layout->addWidget(record_target_button, 2, 0);
  controls_layout->addWidget(record_goal_button, 2, 1);
  controls_layout->addWidget(undo_last_step_button, 3, 0);
  controls_layout->addWidget(save_button, 3, 1);
  controls_layout->addWidget(load_button, 4, 0);
  controls_layout->addWidget(continue_navigation_button, 4, 1);
  controls_layout->addWidget(reset_navigation_button, 5, 0, 1, 2);
  main_layout->addLayout(controls_layout);

  feedback_box_ = new QPlainTextEdit(content_widget);
  feedback_box_->setReadOnly(true);
  feedback_box_->setMinimumHeight(48);
  feedback_box_->setMaximumHeight(80);
  main_layout->addWidget(feedback_box_);

  targets_table_ = new QTableWidget(content_widget);
  targets_table_->setColumnCount(6);
  targets_table_->setHorizontalHeaderLabels(QStringList {"#", "类型", "WP", "X", "Y", "Yaw"});
  targets_table_->setEditTriggers(
      QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed |
      QAbstractItemView::AnyKeyPressed);
  targets_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  targets_table_->setSelectionMode(QAbstractItemView::SingleSelection);
  targets_table_->setAlternatingRowColors(true);
  targets_table_->verticalHeader()->setVisible(false);
  targets_table_->verticalHeader()->setDefaultSectionSize(22);
  targets_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  targets_table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);
  targets_table_->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Stretch);
  targets_table_->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Stretch);
  targets_table_->setMinimumHeight(96);
  main_layout->addWidget(targets_table_);
  update_targets_table(QJsonObject {});

  executable_points_table_ = new QTableWidget(content_widget);
  executable_points_table_->setColumnCount(4);
  executable_points_table_->setHorizontalHeaderLabels(
      QStringList {"#", "类型", "位姿", "状态"});
  executable_points_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  executable_points_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  executable_points_table_->setSelectionMode(QAbstractItemView::SingleSelection);
  executable_points_table_->setAlternatingRowColors(true);
  executable_points_table_->verticalHeader()->setVisible(false);
  executable_points_table_->verticalHeader()->setDefaultSectionSize(22);
  executable_points_table_->horizontalHeader()->setSectionResizeMode(
      QHeaderView::ResizeToContents);
  executable_points_table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
  executable_points_table_->setMinimumHeight(88);
  main_layout->addWidget(executable_points_table_);
  update_executable_points_table(QJsonArray {});

  scroll_area->setWidget(content_widget);
  panel_layout->addWidget(scroll_area);
  setLayout(panel_layout);

  connect(record_start_button, &QPushButton::clicked, this,
          &MissionManagerPanel::on_record_start);
  connect(record_waypoint_button, &QPushButton::clicked, this,
          &MissionManagerPanel::on_record_waypoint);
  connect(record_target_button, &QPushButton::clicked, this,
          &MissionManagerPanel::on_record_target);
  connect(record_goal_button, &QPushButton::clicked, this,
          &MissionManagerPanel::on_record_goal);
  connect(undo_last_step_button, &QPushButton::clicked, this,
          &MissionManagerPanel::on_undo_last_step);
  connect(save_button, &QPushButton::clicked, this, &MissionManagerPanel::on_save_mission);
  connect(load_button, &QPushButton::clicked, this, &MissionManagerPanel::on_load_mission);
  connect(continue_navigation_button, &QPushButton::clicked, this,
          &MissionManagerPanel::on_continue_navigation);
  connect(reset_navigation_button, &QPushButton::clicked, this,
          &MissionManagerPanel::on_reset_navigation);
  connect(mission_name_edit_, &QLineEdit::editingFinished, this,
          &MissionManagerPanel::persist_current_mission_name);
  connect(mission_name_edit_, &QLineEdit::textEdited, this,
          [this](const QString& text) { persist_mission_name(text); });
  connect(targets_table_, &QTableWidget::itemChanged, this,
          &MissionManagerPanel::on_targets_table_item_changed);
}

void MissionManagerPanel::set_feedback_text(const QString& text) {
  feedback_box_->appendPlainText(text);
}

void MissionManagerPanel::update_targets_table(const QJsonObject& status) {
  if (!targets_table_) {
    return;
  }

  updating_targets_table_ = true;
  const QSignalBlocker signal_blocker(targets_table_);

  auto set_item = [this](
                      int row, int column, const QString& text, bool editable,
                      int target_index, const QString& point_type) {
    auto* item = new QTableWidgetItem(text);
    const Qt::Alignment alignment =
        column >= 3 ? (Qt::AlignRight | Qt::AlignVCenter) : Qt::AlignCenter;
    item->setTextAlignment(alignment);
    Qt::ItemFlags flags = Qt::ItemIsSelectable | Qt::ItemIsEnabled;
    if (editable) {
      flags |= Qt::ItemIsEditable;
    }
    item->setFlags(flags);
    item->setData(kTargetIndexRole, target_index);
    item->setData(kPointTypeRole, point_type);
    targets_table_->setItem(row, column, item);
  };

  targets_table_->setRowCount(0);
  const QJsonObject start = status.value("start").toObject();
  const QJsonArray targets = status.value("targets").toArray();
  const QJsonObject goal = status.value("goal").toObject();
  const bool has_start = !start.isEmpty();
  const bool has_goal = !goal.isEmpty();
  if (!has_start && targets.isEmpty() && !has_goal) {
    targets_table_->setRowCount(1);
    for (int column = 0; column < targets_table_->columnCount(); ++column) {
      set_item(0, column, column == 1 ? "无点" : "", false, -1, "");
    }
    updating_targets_table_ = false;
    return;
  }

  const int row_count = targets.size() + (has_start ? 1 : 0) + (has_goal ? 1 : 0);
  targets_table_->setRowCount(row_count);
  int row = 0;
  if (has_start) {
    set_item(row, 0, "", false, -1, "start");
    set_item(row, 1, "start", false, -1, "start");
    set_item(row, 2, "", false, -1, "start");
    set_item(row, 3, pose_value_text(start, "x"), false, -1, "start");
    set_item(row, 4, pose_value_text(start, "y"), false, -1, "start");
    set_item(row, 5, pose_value_text(start, "yaw"), false, -1, "start");
    ++row;
  }
  for (int target_row = 0; target_row < targets.size(); ++target_row) {
    const QJsonObject target = targets.at(target_row).toObject();
    const int target_index = target.value("target_index").toInt(target_row);
    const QJsonObject pose = target.value("pose").toObject();
    set_item(row, 0, QString::number(target_index), false, target_index, "target");
    set_item(row, 1, "target", false, target_index, "target");
    set_item(row, 2, QString::number(target.value("waypoint_count").toInt()), false,
             target_index, "target");
    set_item(row, 3, pose_value_text(pose, "x"), true, target_index, "target");
    set_item(row, 4, pose_value_text(pose, "y"), true, target_index, "target");
    set_item(row, 5, pose_value_text(pose, "yaw"), true, target_index, "target");
    ++row;
  }
  if (has_goal) {
    const int target_index = targets.size();
    set_item(row, 0, QString::number(target_index), false, target_index, "goal");
    set_item(row, 1, "goal", false, target_index, "goal");
    set_item(row, 2, QString::number(status.value("goal_waypoint_count").toInt()), false,
             target_index, "goal");
    set_item(row, 3, pose_value_text(goal, "x"), true, target_index, "goal");
    set_item(row, 4, pose_value_text(goal, "y"), true, target_index, "goal");
    set_item(row, 5, pose_value_text(goal, "yaw"), true, target_index, "goal");
  }
  updating_targets_table_ = false;
}

void MissionManagerPanel::update_executable_points_table(
    const QJsonArray& executable_points) {
  if (!executable_points_table_) {
    return;
  }

  auto set_item = [this](int row, int column, const QString& text) {
    auto* item = new QTableWidgetItem(text);
    const Qt::Alignment alignment =
        column == 2 ? (Qt::AlignLeft | Qt::AlignVCenter) : Qt::AlignCenter;
    item->setTextAlignment(alignment);
    executable_points_table_->setItem(row, column, item);
  };

  executable_points_table_->setRowCount(0);
  if (executable_points.isEmpty()) {
    executable_points_table_->setRowCount(1);
    set_item(0, 0, "");
    set_item(0, 1, "");
    set_item(0, 2, "无导航条目");
    set_item(0, 3, "");
    return;
  }

  executable_points_table_->setRowCount(executable_points.size());
  for (int row = 0; row < executable_points.size(); ++row) {
    const QJsonObject entry = executable_points.at(row).toObject();
    const QString type = entry.value("type").toString();
    QString state_text;
    if (row < current_target_index_) {
      state_text = "完成";
    } else if (row == current_target_index_) {
      state_text = "下一步";
    }

    set_item(row, 0, QString::number(entry.value("target_index").toInt(row)));
    set_item(row, 1, type);
    set_item(row, 2, pose_to_compact_text(entry.value("pose").toObject()));
    set_item(row, 3, state_text);
  }
}

QJsonObject MissionManagerPanel::pose_for_target_index(int target_index) const {
  const QJsonArray targets = latest_status_.value("targets").toArray();
  for (const auto& target_value : targets) {
    const QJsonObject target = target_value.toObject();
    if (target.value("target_index").toInt(-1) == target_index) {
      return target.value("pose").toObject();
    }
  }
  if (target_index == targets.size()) {
    return latest_status_.value("goal").toObject();
  }
  return QJsonObject {};
}

void MissionManagerPanel::restore_targets_table_from_status() {
  update_targets_table(latest_status_);
}

void MissionManagerPanel::consume_pending_status_json() {
  std::string status_json;
  std::string feedback_text;
  bool advance_navigation_index = false;
  bool restore_targets_table = false;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    status_json.swap(pending_status_json_);
    feedback_text.swap(pending_feedback_);
    advance_navigation_index = pending_advance_navigation_index_;
    pending_advance_navigation_index_ = false;
    restore_targets_table = pending_restore_targets_table_;
    pending_restore_targets_table_ = false;
  }

  if (advance_navigation_index && current_target_index_ < executable_points_.size()) {
    ++current_target_index_;
  }

  if (!feedback_text.empty()) {
    set_feedback_text(QString::fromStdString(feedback_text));
  }

  if (status_json.empty()) {
    if (restore_targets_table) {
      restore_targets_table_from_status();
    }
    if (advance_navigation_index) {
      update_executable_points_table(executable_points_);
    }
    return;
  }

  const QJsonDocument document =
      QJsonDocument::fromJson(QByteArray::fromStdString(status_json));
  if (!document.isObject()) {
    set_feedback_text("解析 /mission_manager/status 失败");
    return;
  }

  const QJsonObject object = document.object();
  latest_status_ = object;
  const QString mission_name = object.value("mission_name").toString();
  if (!mission_name.isEmpty() && mission_name != status_mission_name_) {
    status_mission_name_ = mission_name;
    current_target_index_ = 0;
  }
  update_targets_table(object);
  draft_waypoint_count_ = object.value("draft_waypoint_count").toInt(0);
  executable_points_ = object.value("executable_points").toArray();
  executable_count_ = object.value("executable_count").toInt(executable_points_.size());
  if (current_target_index_ > executable_points_.size()) {
    current_target_index_ = executable_points_.size();
  }
  update_executable_points_table(executable_points_);
  const bool mission_loaded_from_file =
      !object.value("last_loaded_path").toString().isEmpty() ||
      !object.value("last_saved_path").toString().isEmpty();
  if (mission_loaded_from_file) {
    persist_mission_name(mission_name);
  }
}

void MissionManagerPanel::restore_persisted_mission_name() {
  QSettings settings("rc2026", "mission_manager_panel");
  const QString mission_name =
      settings.value("last_mission_name").toString().trimmed();
  if (!mission_name.isEmpty()) {
    mission_name_edit_->setText(mission_name);
  }
}

void MissionManagerPanel::persist_current_mission_name() const {
  persist_mission_name(mission_name_edit_->text());
}

void MissionManagerPanel::persist_mission_name(const QString& mission_name) const {
  const QString trimmed_mission_name = mission_name.trimmed();
  if (trimmed_mission_name.isEmpty()) {
    return;
  }

  QSettings settings("rc2026", "mission_manager_panel");
  settings.setValue("last_mission_name", trimmed_mission_name);
  settings.sync();
}

void MissionManagerPanel::maybe_auto_load_mission() {
  if (auto_load_requested_ || !load_mission_client_) {
    return;
  }

  const QString mission_name_text = mission_name_edit_->text().trimmed();
  if (mission_name_text.isEmpty()) {
    return;
  }

  if (!load_mission_client_->service_is_ready()) {
    return;
  }

  auto_load_requested_ = true;
  current_target_index_ = 0;
  update_executable_points_table(executable_points_);
  const std::string mission_name = mission_name_text.toStdString();
  auto request = std::make_shared<mission_manager::srv::LoadMission::Request>();
  request->mission_name = mission_name;
  load_mission_client_->async_send_request(
      request,
      [this](rclcpp::Client<mission_manager::srv::LoadMission>::SharedFuture future) {
        const auto response = future.get();
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_feedback_ = response->success ? ("自动加载成功: " + response->file_path)
                                              : ("自动加载失败: " + response->message);
      });
}

void MissionManagerPanel::call_trigger_service(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client,
    const QString& action_name) {
  if (!raw_node_) {
    set_feedback_text("RViz ROS 节点尚未初始化");
    return;
  }
  if (!client || !client->wait_for_service(std::chrono::milliseconds(250))) {
    set_feedback_text(QString("等待服务超时: %1").arg(action_name));
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(
      request, [this, action_name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_feedback_ = QString("%1: %2")
                                .arg(action_name,
                                     QString::fromStdString(response->message))
                                .toStdString();
      });
}

void MissionManagerPanel::handle_status_message(const std_msgs::msg::String::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(pending_mutex_);
  pending_status_json_ = msg->data;
}

}  // namespace mission_manager

PLUGINLIB_EXPORT_CLASS(mission_manager::MissionManagerPanel, rviz_common::Panel)
