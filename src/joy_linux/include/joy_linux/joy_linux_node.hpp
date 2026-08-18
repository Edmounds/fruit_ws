// Copyright 2026 ConQU2026

/*
 * joy_linux_node headers
 */

#ifndef JOY_LINUX__JOY_LINUX_NODE_HPP_
#define JOY_LINUX__JOY_LINUX_NODE_HPP_

#include <linux/input.h>
#include <linux/joystick.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>

namespace joy_linux
{

class JoyLinux : public rclcpp::Node
{
public:
  explicit JoyLinux(const rclcpp::NodeOptions & options);
  virtual ~JoyLinux();

private:
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  std::string get_dev_by_joy_name(const std::string & joy_name);
  void set_feedback(const std::shared_ptr<sensor_msgs::msg::JoyFeedbackArray> msg);
  void poll_joystick();

  bool open_;
  bool sticky_buttons_;
  bool default_trig_val_;
  std::string joy_dev_;
  std::string joy_dev_name_;
  std::string joy_dev_ff_;
  double deadzone_;
  double autorepeat_rate_;
  double coalesce_interval_;
  int event_count_;
  int pub_count_;
  double lastDiagTime_;

  int ff_fd_;
  struct ff_effect joy_effect_;
  bool update_feedback_;

  std::shared_ptr<diagnostic_updater::Updater> diagnostic_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JoyFeedbackArray>::SharedPtr sub_;

  std::thread joy_thread_;
  std::atomic<bool> stop_joy_thread_;
};

}  // namespace joy_linux

#endif  // JOY_LINUX__JOY_LINUX_NODE_HPP_
