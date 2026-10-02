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

#ifndef LOAM_INTERFACE__LOAM_INTERFACE_HPP_
#define LOAM_INTERFACE__LOAM_INTERFACE_HPP_

#include <deque>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace loam_interface
{

class LoamInterfaceNode : public rclcpp::Node
{
public:
  explicit LoamInterfaceNode(const rclcpp::NodeOptions & options);

private:
  struct InitialPoseSample
  {
    rclcpp::Time stamp;
    tf2::Transform camera_init_to_body;
  };

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  bool initializeExtrinsics(const rclcpp::Time & stamp);

  bool updateInitialPoseStability(
    const rclcpp::Time & stamp, const tf2::Transform & camera_init_to_body);

  tf2::Transform meanInitialPose() const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr base_odom_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string state_estimation_topic_;
  std::string registered_scan_topic_;
  std::string odom_frame_;
  std::string lidar_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  tf2::Transform base_to_lidar_;
  tf2::Transform lidar_in_imu_;

  bool stabilize_initial_odom_{false};
  double initial_stability_timeout_sec_{5.0};
  double initial_stability_window_sec_{1.5};
  int initial_stability_min_samples_{15};
  double initial_stability_max_translation_speed_{0.03};
  double initial_stability_max_rotation_speed_{0.08};
  double initial_stability_position_stddev_{0.02};
  double initial_stability_yaw_stddev_{0.03};
  bool extrinsics_initialized_{false};
  bool initial_stability_started_{false};
  bool initial_stability_failed_{false};
  rclcpp::Time initial_stability_start_time_;
  std::deque<InitialPoseSample> initial_pose_samples_;
  bool odom_to_camera_init_initialized_{false};
  bool odometry_initialized_{false};
  // Constant after initialization: aligns the selected Point-LIO origin pose with base_link.
  tf2::Transform tf_odom_to_camera_init_;

  bool previous_base_odom_initialized_{false};
  rclcpp::Time previous_base_odom_time_{0, 0, RCL_ROS_TIME};
  tf2::Transform previous_base_odom_transform_;
};

}  // namespace loam_interface

#endif  // LOAM_INTERFACE__LOAM_INTERFACE_HPP_
