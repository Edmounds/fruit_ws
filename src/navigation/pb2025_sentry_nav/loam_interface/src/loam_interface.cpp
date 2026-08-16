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

#include "loam_interface/extrinsic_math.hpp"

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
  this->declare_parameter<std::vector<double>>(
    "extrinsic_T", std::vector<double>{-0.011, -0.02329, 0.04412});
  this->declare_parameter<std::vector<double>>(
    "extrinsic_R",
    std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});

  this->get_parameter("state_estimation_topic", state_estimation_topic_);
  this->get_parameter("registered_scan_topic", registered_scan_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("imu_frame", imu_frame_);

  std::vector<double> extrinsic_t;
  std::vector<double> extrinsic_r;
  this->get_parameter("extrinsic_T", extrinsic_t);
  this->get_parameter("extrinsic_R", extrinsic_r);
  lidar_in_imu_ = makeLidarInImuTransform(extrinsic_t, extrinsic_r);

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
    "Point-LIO body/IMU frame '%s' -> LiDAR frame '%s' using Mid360 extrinsic_T=[%.5f, %.5f, %.5f]",
    imu_frame_.c_str(), lidar_frame_.c_str(),
    extrinsic_t[0], extrinsic_t[1], extrinsic_t[2]);
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

  if (!odom_to_camera_init_initialized_) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.5));
      tf2::Transform tf_base_to_lidar;
      tf2::fromMsg(tf_stamped.transform, tf_base_to_lidar);
      tf_odom_to_camera_init_ =
        odomToCameraInit(tf_base_to_lidar, tf_camera_init_to_body, lidar_in_imu_);
      odom_to_camera_init_initialized_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Initialized odom<-camera_init from base->%s and %s->%s extrinsic",
        lidar_frame_.c_str(), imu_frame_.c_str(), lidar_frame_.c_str());
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      return;
    }
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

}  // namespace loam_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(loam_interface::LoamInterfaceNode)
