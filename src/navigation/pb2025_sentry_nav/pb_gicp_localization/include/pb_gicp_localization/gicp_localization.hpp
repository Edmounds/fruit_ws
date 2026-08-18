#ifndef PB_GICP_LOCALIZATION__GICP_LOCALIZATION_HPP_
#define PB_GICP_LOCALIZATION__GICP_LOCALIZATION_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/io/pcd_io.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "pb_gicp_localization/registration_validation.hpp"

namespace pb_gicp_localization
{

class GicpLocalizationNode : public rclcpp::Node
{
public:
  explicit GicpLocalizationNode(const rclcpp::NodeOptions & options);
  ~GicpLocalizationNode() override;

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  void requestRegistration();
  void registrationWorker();
  void performRegistration(const pcl::PointCloud<pcl::PointXYZ> & scan);
  void publishTransform();
  void publishAlignedCloud(
    const pcl::PointCloud<pcl::PointCovariance> & source, const Eigen::Isometry3d & map_to_odom);
  void publishDiagnostic(
    bool accepted, const std::string & message, std::size_t source_points,
    std::size_t inliers, double mean_error, double inlier_ratio,
    double translation_jump, double rotation_jump);

  int num_threads_;
  int num_neighbors_;
  int lost_after_rejects_;
  double global_leaf_size_;
  double registered_leaf_size_;
  double max_dist_sq_;
  double input_scan_timeout_;
  double initial_registration_delay_sec_;
  bool registration_once_;
  std::vector<double> initial_map_to_odom_;
  ValidationThresholds thresholds_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string localization_map_;
  std::string input_cloud_topic_;
  rclcpp::Time last_scan_time_;
  Eigen::Isometry3d result_t_{Eigen::Isometry3d::Identity()};
  std::mutex result_mutex_;

  std::mutex cloud_mutex_;
  std::mutex scan_time_mutex_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    registration_;

  std::atomic<bool> shutdown_{false};
  std::atomic<bool> has_accepted_once_{false};
  std::atomic<int> consecutive_rejects_{0};
  std::chrono::steady_clock::time_point registration_start_time_;
  std::mutex worker_mutex_;
  std::condition_variable worker_cv_;
  bool registration_pending_{false};
  pcl::PointCloud<pcl::PointXYZ> pending_scan_;
  std::thread registration_thread_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr source_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gicp_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gicp_path_pub_;
  nav_msgs::msg::Path gicp_path_;
  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace pb_gicp_localization

#endif  // PB_GICP_LOCALIZATION__GICP_LOCALIZATION_HPP_
