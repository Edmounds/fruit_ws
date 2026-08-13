#include "pb_gicp_localization/gicp_localization.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <small_gicp/util/downsampling_omp.hpp>

namespace pb_gicp_localization
{
namespace
{
sensor_msgs::msg::PointCloud2 toMsg(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, const std::string & frame,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame;
  msg.header.stamp = stamp;
  return msg;
}
}  // namespace

GicpLocalizationNode::GicpLocalizationNode(const rclcpp::NodeOptions & options)
: Node("pb_gicp_localization", options)
{
  num_threads_ = declare_parameter("num_threads", 4);
  num_neighbors_ = declare_parameter("num_neighbors", 20);
  lost_after_rejects_ = declare_parameter("lost_after_rejects", 10);
  global_leaf_size_ = declare_parameter("global_leaf_size", 0.10);
  registered_leaf_size_ = declare_parameter("registered_leaf_size", 0.05);
  max_dist_sq_ = declare_parameter("max_dist_sq", 3.0);
  map_frame_ = declare_parameter("map_frame", "map");
  odom_frame_ = declare_parameter("odom_frame", "odom");
  localization_map_ = declare_parameter("localization_map", "");
  input_cloud_topic_ = declare_parameter("input_cloud_topic", "registered_scan");
  initial_map_to_odom_ = declare_parameter(
    "initial_map_to_odom", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  thresholds_.min_inliers = static_cast<std::size_t>(declare_parameter("min_inliers", 500));
  thresholds_.min_inlier_ratio = declare_parameter("min_inlier_ratio", 0.20);
  thresholds_.max_mean_error = declare_parameter("max_mean_error", 1.0);
  thresholds_.max_translation_jump = declare_parameter("max_translation_jump", 0.5);
  thresholds_.max_rotation_jump = declare_parameter("max_rotation_jump", 0.35);
  thresholds_.max_translation_jump_initial =
    declare_parameter("max_translation_jump_initial", 1.0);
  thresholds_.max_rotation_jump_initial = declare_parameter("max_rotation_jump_initial", 0.35);

  if (num_threads_ <= 0 || num_neighbors_ <= 0 || lost_after_rejects_ <= 0) {
    throw std::invalid_argument(
      "num_threads, num_neighbors, and lost_after_rejects must be positive");
  }
  if (!std::isfinite(global_leaf_size_) || !std::isfinite(registered_leaf_size_) ||
    !std::isfinite(max_dist_sq_) || global_leaf_size_ <= 0.0 ||
    registered_leaf_size_ <= 0.0 || max_dist_sq_ <= 0.0)
  {
    throw std::invalid_argument("voxel leaf sizes and max_dist_sq must be positive");
  }
  if (thresholds_.min_inliers == 0 || !std::isfinite(thresholds_.min_inlier_ratio) ||
    !std::isfinite(thresholds_.max_mean_error) ||
    !std::isfinite(thresholds_.max_translation_jump) ||
    !std::isfinite(thresholds_.max_rotation_jump) ||
    !std::isfinite(thresholds_.max_translation_jump_initial) ||
    !std::isfinite(thresholds_.max_rotation_jump_initial) ||
    thresholds_.min_inlier_ratio <= 0.0 || thresholds_.min_inlier_ratio > 1.0 ||
    thresholds_.max_mean_error < 0.0 || thresholds_.max_translation_jump < 0.0 ||
    thresholds_.max_rotation_jump < 0.0 || thresholds_.max_translation_jump_initial < 0.0 ||
    thresholds_.max_rotation_jump_initial < 0.0)
  {
    throw std::invalid_argument("invalid registration validation thresholds");
  }
  if (initial_map_to_odom_.size() != 6) {
    throw std::invalid_argument("initial_map_to_odom must contain [x, y, z, roll, pitch, yaw]");
  }
  if (!std::all_of(
      initial_map_to_odom_.begin(), initial_map_to_odom_.end(),
      [](double value) {return std::isfinite(value);}))
  {
    throw std::invalid_argument("initial_map_to_odom values must be finite");
  }
  result_t_.translation() << initial_map_to_odom_[0], initial_map_to_odom_[1],
    initial_map_to_odom_[2];
  result_t_.linear() =
    (Eigen::AngleAxisd(initial_map_to_odom_[5], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(initial_map_to_odom_[4], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(initial_map_to_odom_[3], Eigen::Vector3d::UnitX())).toRotationMatrix();

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  registration_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();
  loadGlobalMap(localization_map_);

  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_, num_threads_);
  if (!target_ || target_->size() < static_cast<std::size_t>(num_neighbors_)) {
    throw std::runtime_error("localization map has too few valid points after downsampling");
  }
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
  rclcpp::QoS latched_cloud_qos(1);
  latched_cloud_qos.transient_local();
  map_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "gicp/localization_map", latched_cloud_qos);
  source_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("gicp/source_cloud", 1);
  aligned_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("gicp/aligned_cloud", 1);
  gicp_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("gicp/pose", 10);
  gicp_path_pub_ = create_publisher<nav_msgs::msg::Path>("gicp/path", 10);
  gicp_path_.header.frame_id = map_frame_;

  {
    pcl::PointCloud<pcl::PointXYZ> viz_map;
    for (const auto & pt : target_->points) {
      viz_map.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
    }
    map_cloud_pub_->publish(toMsg(viz_map, map_frame_, now()));
  }

  pcd_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&GicpLocalizationNode::registeredPcdCallback, this, std::placeholders::_1));
  register_timer_ = create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&GicpLocalizationNode::requestRegistration, this));
  transform_timer_ = create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&GicpLocalizationNode::publishTransform, this));
  registration_thread_ = std::thread(&GicpLocalizationNode::registrationWorker, this);
}

GicpLocalizationNode::~GicpLocalizationNode()
{
  shutdown_.store(true);
  worker_cv_.notify_all();
  if (registration_thread_.joinable()) {
    registration_thread_.join();
  }
}

void GicpLocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (file_name.empty() || pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) < 0) {
    throw std::runtime_error("failed to load localization PCD: " + file_name);
  }
  std::vector<int> valid_indices;
  pcl::PointCloud<pcl::PointXYZ> finite;
  pcl::removeNaNFromPointCloud(*global_map_, finite, valid_indices);
  *global_map_ = std::move(finite);
  if (global_map_->size() < static_cast<std::size_t>(num_neighbors_)) {
    throw std::runtime_error("localization PCD is empty or has too few finite points");
  }
  RCLCPP_INFO(
    get_logger(), "Loaded map-frame localization map with %zu points", global_map_->size());
}

void GicpLocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (msg->header.frame_id != odom_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Dropping scan in frame '%s'; expected '%s'",
      msg->header.frame_id.c_str(), odom_frame_.c_str());
    return;
  }
  pcl::PointCloud<pcl::PointXYZ> scan;
  pcl::fromROSMsg(*msg, scan);
  std::vector<int> valid_indices;
  pcl::PointCloud<pcl::PointXYZ> finite_scan;
  pcl::removeNaNFromPointCloud(scan, finite_scan, valid_indices);
  if (finite_scan.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Dropping scan with no finite points");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    *accumulated_cloud_ += finite_scan;
  }
  {
    std::lock_guard<std::mutex> lock(scan_time_mutex_);
    last_scan_time_ = msg->header.stamp;
  }
}

void GicpLocalizationNode::requestRegistration()
{
  if (registration_busy_.load()) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZ> scan;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (accumulated_cloud_->empty()) {
      return;
    }
    scan.swap(*accumulated_cloud_);
  }
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    pending_scan_ = std::move(scan);
    registration_pending_ = true;
  }
  worker_cv_.notify_one();
}

void GicpLocalizationNode::registrationWorker()
{
  while (!shutdown_.load()) {
    pcl::PointCloud<pcl::PointXYZ> scan;
    {
      std::unique_lock<std::mutex> lock(worker_mutex_);
      worker_cv_.wait(
        lock, [this] {return shutdown_.load() || registration_pending_;});
      if (shutdown_.load()) {
        return;
      }
      scan = std::move(pending_scan_);
      registration_pending_ = false;
    }
    registration_busy_.store(true);
    try {
      performRegistration(scan);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "GICP registration exception: %s", ex.what());
      publishDiagnostic(false, ex.what(), 0, 0, 0.0, 0.0, 0.0, 0.0);
    }
    registration_busy_.store(false);
  }
}

void GicpLocalizationNode::performRegistration(const pcl::PointCloud<pcl::PointXYZ> & scan)
{
  auto source = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    scan, registered_leaf_size_, num_threads_);
  if (!source || source->size() < static_cast<std::size_t>(num_neighbors_)) {
    consecutive_rejects_.fetch_add(1);
    publishDiagnostic(
      false, "source scan too small", source ? source->size() : 0, 0, 0.0, 0.0, 0.0, 0.0);
    return;
  }
  small_gicp::estimate_covariances_omp(*source, num_neighbors_, num_threads_);

  Eigen::Isometry3d previous;
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    previous = result_t_;
  }

  registration_->reduction.num_threads = num_threads_;
  registration_->rejector.max_dist_sq = max_dist_sq_;
  registration_->optimizer.max_iterations = 10;
  const auto result = registration_->align(*target_, *source, *target_tree_, previous);

  RegistrationMetrics metrics;
  metrics.converged = result.converged;
  metrics.source_points = source->size();
  metrics.inliers = result.num_inliers;
  metrics.total_error = result.error;
  metrics.candidate = result.T_target_source;
  std::string reason;
  const double mean_error = result.num_inliers > 0 ? result.error / result.num_inliers : 0.0;
  const double inlier_ratio = metrics.source_points > 0 ?
    static_cast<double>(metrics.inliers) / static_cast<double>(metrics.source_points) : 0.0;
  const double translation_jump =
    (metrics.candidate.translation() - previous.translation()).norm();
  const double rotation_jump = rotationDistance(previous, metrics.candidate);
  const bool first_acceptance = !has_accepted_once_.load();
  if (!validateRegistration(metrics, previous, thresholds_, reason, first_acceptance)) {
    const int rejects = consecutive_rejects_.fetch_add(1) + 1;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Rejected GICP result: %s (inliers=%zu/%zu, mean_error=%.3f, rejects=%d)",
      reason.c_str(), result.num_inliers, source->size(), mean_error, rejects);
    publishDiagnostic(
      false, reason, source->size(), result.num_inliers, mean_error, inlier_ratio,
      translation_jump, rotation_jump);
    // Still publish source in odom for tuning even when rejected.
    {
      pcl::PointCloud<pcl::PointXYZ> source_xyz;
      source_xyz.reserve(source->size());
      for (const auto & pt : source->points) {
        source_xyz.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
      }
      source_cloud_pub_->publish(toMsg(source_xyz, odom_frame_, now()));
    }
    return;
  }

  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    result_t_ = result.T_target_source;
  }
  has_accepted_once_.store(true);
  consecutive_rejects_.store(0);
  publishAlignedCloud(*source, result.T_target_source);
  publishDiagnostic(
    true, first_acceptance ? "initial registration accepted" : "registration accepted",
    source->size(), result.num_inliers, mean_error, inlier_ratio, translation_jump, rotation_jump);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = now();
  pose.header.frame_id = map_frame_;
  pose.pose.position.x = result.T_target_source.translation().x();
  pose.pose.position.y = result.T_target_source.translation().y();
  pose.pose.position.z = result.T_target_source.translation().z();
  const Eigen::Quaterniond q(result.T_target_source.rotation());
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  gicp_pose_pub_->publish(pose);
  gicp_path_.header.stamp = pose.header.stamp;
  gicp_path_.poses.push_back(pose);
  if (gicp_path_.poses.size() > 2000) {
    gicp_path_.poses.erase(gicp_path_.poses.begin(), gicp_path_.poses.begin() + 500);
  }
  gicp_path_pub_->publish(gicp_path_);
}

void GicpLocalizationNode::publishAlignedCloud(
  const pcl::PointCloud<pcl::PointCovariance> & source, const Eigen::Isometry3d & map_to_odom)
{
  pcl::PointCloud<pcl::PointXYZ> source_xyz;
  source_xyz.reserve(source.size());
  for (const auto & pt : source.points) {
    source_xyz.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
  }
  source_cloud_pub_->publish(toMsg(source_xyz, odom_frame_, now()));

  pcl::PointCloud<pcl::PointXYZ> aligned;
  const Eigen::Affine3f map_tf(map_to_odom.matrix().cast<float>());
  pcl::transformPointCloud(source_xyz, aligned, map_tf);
  aligned_cloud_pub_->publish(toMsg(aligned, map_frame_, now()));
}

void GicpLocalizationNode::publishTransform()
{
  Eigen::Isometry3d result;
  rclcpp::Time scan_time;
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    result = result_t_;
  }
  {
    std::lock_guard<std::mutex> lock(scan_time_mutex_);
    scan_time = last_scan_time_;
  }
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = scan_time.nanoseconds() == 0 ? now() : scan_time;
  transform.header.frame_id = map_frame_;
  transform.child_frame_id = odom_frame_;
  transform.transform.translation.x = result.translation().x();
  transform.transform.translation.y = result.translation().y();
  transform.transform.translation.z = result.translation().z();
  const Eigen::Quaterniond rotation(result.rotation());
  transform.transform.rotation.x = rotation.x();
  transform.transform.rotation.y = rotation.y();
  transform.transform.rotation.z = rotation.z();
  transform.transform.rotation.w = rotation.w();
  tf_broadcaster_->sendTransform(transform);
}

void GicpLocalizationNode::publishDiagnostic(
  bool accepted, const std::string & message, std::size_t source_points,
  std::size_t inliers, double mean_error, double inlier_ratio,
  double translation_jump, double rotation_jump)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "pb_gicp_localization/registration";
  status.hardware_id = "GLIM map";
  const int rejects = consecutive_rejects_.load();
  if (accepted) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  } else if (rejects >= lost_after_rejects_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "localization lost: " + message;
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = message;
  }
  if (accepted) {
    status.message = message;
  }
  auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      status.values.push_back(kv);
    };
  add("accepted", accepted ? "1" : "0");
  add("has_accepted_once", has_accepted_once_.load() ? "1" : "0");
  add("consecutive_rejects", std::to_string(rejects));
  add("source_points", std::to_string(source_points));
  add("inliers", std::to_string(inliers));
  add("inlier_ratio", std::to_string(inlier_ratio));
  add("mean_error", std::to_string(mean_error));
  add("translation_jump", std::to_string(translation_jump));
  add("rotation_jump", std::to_string(rotation_jump));
  array.status.push_back(status);
  diagnostics_pub_->publish(array);
}

}  // namespace pb_gicp_localization

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pb_gicp_localization::GicpLocalizationNode)
