// Copyright 2025 Lihan Chen
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

#include "loam_interface/loam_interface.hpp"

#include <cmath>
#include <stdexcept>

#include "loam_interface/extrinsic_math.hpp"

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace loam_interface
{

LoamInterfaceNode::LoamInterfaceNode(const rclcpp::NodeOptions & options)
: Node("loam_interface", options)
{
  this->declare_parameter<std::string>("state_estimation_topic", "");
  this->declare_parameter<std::string>("registered_scan_topic", "");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("lidar_frame", "");
  this->declare_parameter<std::string>("imu_frame", "livox_frame");
  this->declare_parameter<bool>("stabilize_initial_odom", false);
  this->declare_parameter<double>("initial_stability_timeout_sec", 5.0);
  this->declare_parameter<double>("initial_stability_window_sec", 1.5);
  this->declare_parameter<int>("initial_stability_min_samples", 15);
  this->declare_parameter<double>("initial_stability_max_translation_speed", 0.03);
  this->declare_parameter<double>("initial_stability_max_rotation_speed", 0.08);
  this->declare_parameter<double>("initial_stability_position_stddev", 0.02);
  this->declare_parameter<double>("initial_stability_yaw_stddev", 0.03);

  this->get_parameter("state_estimation_topic", state_estimation_topic_);
  this->get_parameter("registered_scan_topic", registered_scan_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("imu_frame", imu_frame_);
  this->get_parameter("stabilize_initial_odom", stabilize_initial_odom_);
  this->get_parameter("initial_stability_timeout_sec", initial_stability_timeout_sec_);
  this->get_parameter("initial_stability_window_sec", initial_stability_window_sec_);
  this->get_parameter("initial_stability_min_samples", initial_stability_min_samples_);
  this->get_parameter(
    "initial_stability_max_translation_speed", initial_stability_max_translation_speed_);
  this->get_parameter(
    "initial_stability_max_rotation_speed", initial_stability_max_rotation_speed_);
  this->get_parameter(
    "initial_stability_position_stddev", initial_stability_position_stddev_);
  this->get_parameter("initial_stability_yaw_stddev", initial_stability_yaw_stddev_);

  if (
    initial_stability_timeout_sec_ <= 0.0 || initial_stability_window_sec_ <= 0.0 ||
    initial_stability_min_samples_ < 2 || initial_stability_max_translation_speed_ <= 0.0 ||
    initial_stability_max_rotation_speed_ <= 0.0 ||
    initial_stability_position_stddev_ <= 0.0 || initial_stability_yaw_stddev_ <= 0.0)
  {
    throw std::invalid_argument("initial odometry stability parameters must be positive");
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 5);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("lidar_odometry", 5);

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    registered_scan_topic_, 5,
    std::bind(&LoamInterfaceNode::pointCloudCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    state_estimation_topic_, 5,
    std::bind(&LoamInterfaceNode::odometryCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Point-LIO body/IMU frame '%s' -> LiDAR frame '%s' will be read from TF",
    imu_frame_.c_str(), lidar_frame_.c_str());
  if (stabilize_initial_odom_) {
    RCLCPP_INFO(
      get_logger(),
      "Initial odometry stabilization enabled: timeout=%.2fs window=%.2fs samples=%d",
      initial_stability_timeout_sec_, initial_stability_window_sec_,
      initial_stability_min_samples_);
  }
}

void LoamInterfaceNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!odom_to_camera_init_initialized_ || !odometry_initialized_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Dropping Point-LIO cloud until base_link -> %s and odometry are initialized",
      lidar_frame_.c_str());
    return;
  }
  // cloud_registered is in camera_init (= IMU/body at t0). Map into odom.
  auto out = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl_ros::transformPointCloud(odom_frame_, tf_odom_to_camera_init_, *msg, *out);
  pcd_pub_->publish(*out);
}

void LoamInterfaceNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  // aft_mapped_to_init pose is camera_init -> body (Mid360 internal IMU), not LiDAR.
  tf2::Transform tf_camera_init_to_body;
  tf2::fromMsg(msg->pose.pose, tf_camera_init_to_body);

  if (!initializeExtrinsics(msg->header.stamp)) {
    return;
  }

  if (!odom_to_camera_init_initialized_) {
    if (initial_stability_failed_) {
      return;
    }
    if (stabilize_initial_odom_) {
      if (!updateInitialPoseStability(msg->header.stamp, tf_camera_init_to_body)) {
        return;
      }
      tf_odom_to_camera_init_ =
        odomToCameraInit(base_to_lidar_, meanInitialPose(), lidar_in_imu_);
      RCLCPP_INFO(
        get_logger(), "Initial odometry is stable; publishing odom from averaged origin");
    } else {
      tf_odom_to_camera_init_ =
        odomToCameraInit(base_to_lidar_, tf_camera_init_to_body, lidar_in_imu_);
    }
    odom_to_camera_init_initialized_ = true;
    RCLCPP_INFO(
      get_logger(), "Initialized odom<-camera_init from TF: %s->%s->%s",
      base_frame_.c_str(), lidar_frame_.c_str(), imu_frame_.c_str());
  }

  const tf2::Transform tf_odom_to_lidar =
    odomToLidar(tf_odom_to_camera_init_, tf_camera_init_to_body, lidar_in_imu_);

  nav_msgs::msg::Odometry out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = odom_frame_;
  out.child_frame_id = lidar_frame_;

  const auto & origin = tf_odom_to_lidar.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(tf_odom_to_lidar.getRotation());

  odom_pub_->publish(out);
  odometry_initialized_ = true;
}

bool LoamInterfaceNode::initializeExtrinsics(const rclcpp::Time & stamp)
{
  if (extrinsics_initialized_) {
    return true;
  }
  try {
    auto tf_stamped = tf_buffer_->lookupTransform(
      base_frame_, lidar_frame_, stamp, rclcpp::Duration::from_seconds(0.5));
    tf2::fromMsg(tf_stamped.transform, base_to_lidar_);

    tf_stamped = tf_buffer_->lookupTransform(
      imu_frame_, lidar_frame_, stamp, rclcpp::Duration::from_seconds(0.5));
    tf2::fromMsg(tf_stamped.transform, lidar_in_imu_);
    extrinsics_initialized_ = true;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "TF lookup failed: %s Retrying...", ex.what());
    return false;
  }
}

bool LoamInterfaceNode::updateInitialPoseStability(
  const rclcpp::Time & stamp, const tf2::Transform & camera_init_to_body)
{
  if (!initial_stability_started_) {
    initial_stability_started_ = true;
    initial_stability_start_time_ = stamp;
  }
  const double elapsed = (stamp - initial_stability_start_time_).seconds();
  if (elapsed > initial_stability_timeout_sec_) {
    initial_stability_failed_ = true;
    initial_pose_samples_.clear();
    RCLCPP_ERROR(
      get_logger(),
      "Initial odometry did not stabilize within %.2fs; odom remains unavailable",
      initial_stability_timeout_sec_);
    return false;
  }

  const auto & position = camera_init_to_body.getOrigin();
  const auto & orientation = camera_init_to_body.getRotation();
  if (
    !std::isfinite(position.x()) || !std::isfinite(position.y()) ||
    !std::isfinite(position.z()) || !std::isfinite(orientation.x()) ||
    !std::isfinite(orientation.y()) || !std::isfinite(orientation.z()) ||
    !std::isfinite(orientation.w()) || orientation.length2() <= 0.0)
  {
    initial_pose_samples_.clear();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Waiting for finite Point-LIO pose before initializing odometry");
    return false;
  }

  if (!initial_pose_samples_.empty() && stamp <= initial_pose_samples_.back().stamp) {
    initial_pose_samples_.clear();
  }

  initial_pose_samples_.push_back({stamp, camera_init_to_body});
  while (
    initial_pose_samples_.size() > 1 &&
    (stamp - initial_pose_samples_[1].stamp).seconds() >= initial_stability_window_sec_)
  {
    initial_pose_samples_.pop_front();
  }

  const double sample_span = (stamp - initial_pose_samples_.front().stamp).seconds();
  if (
    sample_span < initial_stability_window_sec_ ||
    static_cast<int>(initial_pose_samples_.size()) < initial_stability_min_samples_)
  {
    return false;
  }

  const auto & first_pose = initial_pose_samples_.front().camera_init_to_body;
  const auto & last_pose = initial_pose_samples_.back().camera_init_to_body;
  const double translation_speed =
    (last_pose.getOrigin() - first_pose.getOrigin()).length() / sample_span;
  const double rotation_speed =
    first_pose.getRotation().angleShortestPath(last_pose.getRotation()) / sample_span;
  if (
    translation_speed > initial_stability_max_translation_speed_ ||
    rotation_speed > initial_stability_max_rotation_speed_)
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Waiting for stationary odometry window: linear=%.4fm/s angular=%.4frad/s",
      translation_speed, rotation_speed);
    return false;
  }

  const tf2::Transform mean_pose = meanInitialPose();
  double position_squared_sum = 0.0;
  double yaw_squared_sum = 0.0;
  const double mean_yaw = tf2::getYaw(mean_pose.getRotation());
  for (const auto & sample : initial_pose_samples_) {
    const tf2::Vector3 position_error =
      sample.camera_init_to_body.getOrigin() - mean_pose.getOrigin();
    position_squared_sum += position_error.length2();
    const double yaw_error = std::atan2(
      std::sin(tf2::getYaw(sample.camera_init_to_body.getRotation()) - mean_yaw),
      std::cos(tf2::getYaw(sample.camera_init_to_body.getRotation()) - mean_yaw));
    yaw_squared_sum += yaw_error * yaw_error;
  }
  const double sample_count = static_cast<double>(initial_pose_samples_.size());
  const double position_stddev = std::sqrt(position_squared_sum / sample_count);
  const double yaw_stddev = std::sqrt(yaw_squared_sum / sample_count);
  if (
    position_stddev > initial_stability_position_stddev_ ||
    yaw_stddev > initial_stability_yaw_stddev_)
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Waiting for stable odometry variance: position_stddev=%.4fm yaw_stddev=%.4frad",
      position_stddev, yaw_stddev);
    return false;
  }
  return true;
}

tf2::Transform LoamInterfaceNode::meanInitialPose() const
{
  tf2::Vector3 mean_position(0.0, 0.0, 0.0);
  tf2::Quaternion quaternion_sum(0.0, 0.0, 0.0, 0.0);
  const tf2::Quaternion reference = initial_pose_samples_.front().camera_init_to_body.getRotation();
  for (const auto & sample : initial_pose_samples_) {
    mean_position += sample.camera_init_to_body.getOrigin();
    tf2::Quaternion quaternion = sample.camera_init_to_body.getRotation();
    if (reference.dot(quaternion) < 0.0) {
      quaternion = tf2::Quaternion(
        -quaternion.x(), -quaternion.y(), -quaternion.z(), -quaternion.w());
    }
    quaternion_sum += quaternion;
  }
  mean_position /= static_cast<double>(initial_pose_samples_.size());
  quaternion_sum.normalize();
  return tf2::Transform(quaternion_sum, mean_position);
}

}  // namespace loam_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(loam_interface::LoamInterfaceNode)
