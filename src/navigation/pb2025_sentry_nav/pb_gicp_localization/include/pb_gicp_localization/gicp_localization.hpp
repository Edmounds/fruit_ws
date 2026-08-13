#ifndef PB_GICP_LOCALIZATION__GICP_LOCALIZATION_HPP_
#define PB_GICP_LOCALIZATION__GICP_LOCALIZATION_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
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

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  void performRegistration();
  void publishTransform();
  void publishDiagnostic(
    bool accepted, const std::string & message, std::size_t source_points,
    std::size_t inliers, double mean_error);

  int num_threads_;
  int num_neighbors_;
  double global_leaf_size_;
  double registered_leaf_size_;
  double max_dist_sq_;
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

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace pb_gicp_localization

#endif  // PB_GICP_LOCALIZATION__GICP_LOCALIZATION_HPP_
