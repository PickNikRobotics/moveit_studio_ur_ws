#include <picknik_registration/ransac_registration.hpp>
#include <pcl/registration/ndt.h>
#include <tl_expected/expected.hpp>
#include <moveit_studio_behavior_interface/async_behavior_base.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
// #include <moveit_studio_vision/geometry_types.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tl_expected/expected.hpp>
#include <moveit_studio_vision_msgs/msg/mask3_d.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <moveit_studio_vision/common/get_all_indices.hpp>
#include <moveit_studio_vision/common/select_point_indices.hpp>
#include <moveit_studio_vision/pointcloud/point_cloud_tools.hpp>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/kdtree.h>

namespace picknik_registration
{
namespace
{

std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>
getFilteredPointCloudFromMessage(const sensor_msgs::msg::PointCloud2& cloud_msg)
{
  const auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::fromROSMsg(cloud_msg, *cloud);

  const pcl::PointIndices valid_indices = moveit_studio::selectPointIndices(*cloud, moveit_studio::point_cloud_tools::getAllIndices(cloud),
                                                             moveit_studio::NeitherNanNorNearZeroPointValidator<pcl::PointXYZRGB>);
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::copyPointCloud(*cloud, valid_indices, *filtered_cloud);

  return filtered_cloud;
}

}  // namespace

RANSACRegistration::RANSACRegistration(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::AsyncBehaviorBase(name, config, shared_resources)
{
}


BT::PortsList RANSACRegistration::providedPorts()
{
  return { BT::InputPort<sensor_msgs::msg::PointCloud2>("base_point_cloud", "{point_cloud}",
                                                        "Point cloud to align with the target point cloud."),
           BT::InputPort<sensor_msgs::msg::PointCloud2>("target_point_cloud", "{target_point_cloud}",
                                                        "Point cloud to which align the base point cloud."),
           BT::InputPort<int>("max_iterations", 30,
                              "Maximum number of attempts to find the transform. Setting a higher number of iterations "
                              "will allow the solver to converge even if the initial estimate of the transform was far "
                              "from the actual transform, but it may take longer to complete."),
           BT::InputPort<double>("transformation_epsilon", 0.001,
                                 "Minimum transformation difference for termination condition <double>"),
           BT::InputPort<double>("step_size", 0.1,
                                 "Maximum step size for More-Thuente line search <double>"),
           BT::InputPort<double>("resolution", 1.0,
                                 "Resolution of NDT grid structure (VoxelGridCovariance) <double>"),
           BT::InputPort<double>("inlier_threshold", 1.0,
                                 "Inlier threshold for RANSAC <double>"),
           BT::InputPort<double>("uniform_sampling_radius", 0.02,
                                 "Radius for uniform sampling of keypoints <double>"),
           BT::InputPort<int>("k_search", 10,
                              "Number of nearest neighbors to use for normal estimation <int>"),
           BT::InputPort<double>("feature_radius", 0.05,
                                 "Radius for feature estimation <double>"),
           BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_pose_in_base_frame", "{target_pose}",
                                                           "The pose of the target point cloud relative to the frame "
                                                           "of the base point cloud.") };
}

BT::KeyValueVector RANSACRegistration::metadata()
{
  // TODO(...)
  return { {"description", "Finds the pose of a target point cloud relative to the base frame of a base point cloud using the Normal Distributions Transform (NDT) algoritm"} };
}


tl::expected<bool, std::string> RANSACRegistration::doWork()
{

  const auto base_point_cloud_msg = getInput<sensor_msgs::msg::PointCloud2>("base_point_cloud");
  const auto target_point_cloud_msg = getInput<sensor_msgs::msg::PointCloud2>("target_point_cloud");
  const auto max_iterations = getInput<int>("max_iterations");
  const auto transformation_epsilon = getInput<double>("transformation_epsilon");
  const auto inlier_threshold = getInput<double>("inlier_threshold");
  const auto uniform_sampling_radius = getInput<double>("uniform_sampling_radius");
  const auto k_search = getInput<int>("k_search");
  const auto feature_radius = getInput<double>("feature_radius");

  if (const auto error = moveit_studio::behaviors::maybe_error(base_point_cloud_msg, target_point_cloud_msg, max_iterations, transformation_epsilon, inlier_threshold, uniform_sampling_radius, k_search, feature_radius); error)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Logger"), "Failed to get required value from input data port: %s", error.value().c_str());
    return tl::make_unexpected("Failed to get required value from input data port: " + error.value());
  }

  const auto base_cloud = getFilteredPointCloudFromMessage(base_point_cloud_msg.value());
  if (base_cloud->empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("Logger"), "Base point cloud has no valid points");
    return tl::make_unexpected("Base point cloud has no valid points");
  }
  const auto target_cloud = getFilteredPointCloudFromMessage(target_point_cloud_msg.value());
  if (target_cloud->empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("Logger"), "Target point cloud has no valid points");
    return tl::make_unexpected("Target point cloud has no valid points");
  }

  // Estimating surface normals
  pcl::PointCloud<pcl::Normal>::Ptr base_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);

  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_est;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  normal_est.setSearchMethod(tree);

  normal_est.setInputCloud(base_cloud);
  normal_est.setKSearch(k_search.value());
  normal_est.compute(*base_normals);

  normal_est.setInputCloud(target_cloud);
  normal_est.setKSearch(k_search.value());
  normal_est.compute(*target_normals);

  // Logging normals
  RCLCPP_ERROR(rclcpp::get_logger("Logger"), "Computed %ld base normals and %ld target normals", base_normals->size(), target_normals->size());

  // Keypoint selection
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_keypoints(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_keypoints(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;
  uniform_sampling.setInputCloud(base_cloud);
  uniform_sampling.setRadiusSearch(uniform_sampling_radius.value());
  uniform_sampling.filter(*base_keypoints);

  uniform_sampling.setInputCloud(target_cloud);
  uniform_sampling.setRadiusSearch(uniform_sampling_radius.value());
  uniform_sampling.filter(*target_keypoints);

  // Logging keypoints
  RCLCPP_ERROR(rclcpp::get_logger("Logger"), "Selected %ld base keypoints and %ld target keypoints", base_keypoints->size(), target_keypoints->size());

  // Estimating normals for keypoints
  pcl::PointCloud<pcl::Normal>::Ptr base_keypoint_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr target_keypoint_normals(new pcl::PointCloud<pcl::Normal>);

  pcl::PointIndices::Ptr base_keypoint_indices(new pcl::PointIndices);
  pcl::PointIndices::Ptr target_keypoint_indices(new pcl::PointIndices);

  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud(base_cloud);
  for (size_t i = 0; i < base_keypoints->points.size(); ++i)
  {
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    kdtree.nearestKSearch(base_keypoints->points[i], 1, indices, distances);
    base_keypoint_indices->indices.push_back(indices[0]);
  }
  pcl::copyPointCloud(*base_normals, *base_keypoint_indices, *base_keypoint_normals);

  kdtree.setInputCloud(target_cloud);
  for (size_t i = 0; i < target_keypoints->points.size(); ++i)
  {
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    kdtree.nearestKSearch(target_keypoints->points[i], 1, indices, distances);
    target_keypoint_indices->indices.push_back(indices[0]);
  }
  pcl::copyPointCloud(*target_normals, *target_keypoint_indices, *target_keypoint_normals);

  // Feature estimation
  pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr base_features(new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>);

  fpfh_est.setSearchMethod(tree);

  fpfh_est.setInputCloud(base_keypoints);
  fpfh_est.setInputNormals(base_keypoint_normals);
  fpfh_est.setRadiusSearch(feature_radius.value());
  fpfh_est.compute(*base_features);

  fpfh_est.setInputCloud(target_keypoints);
  fpfh_est.setInputNormals(target_keypoint_normals);
  fpfh_est.setRadiusSearch(feature_radius.value());
  fpfh_est.compute(*target_features);

  // Logging features
  RCLCPP_ERROR(rclcpp::get_logger("Logger"), "Computed %ld base features and %ld target features", base_features->size(), target_features->size());

  // RANSAC alignment
  pcl::SampleConsensusPrerejective<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> align;
  align.setInputSource(base_keypoints);
  align.setSourceFeatures(base_features);
  align.setInputTarget(target_keypoints);
  align.setTargetFeatures(target_features);
  align.setMaximumIterations(max_iterations.value());
  align.setNumberOfSamples(3);
  align.setCorrespondenceRandomness(5);
  align.setSimilarityThreshold(0.9f);
  align.setMaxCorrespondenceDistance(inlier_threshold.value());
  align.setInlierFraction(0.25f);

  pcl::PointCloud<pcl::PointXYZRGB> output_cloud;
  align.align(output_cloud);

  if (!align.hasConverged())
  {
    RCLCPP_ERROR(rclcpp::get_logger("Logger"), "RANSAC could not converge to a transform that aligns both point clouds");
    return tl::make_unexpected("RANSAC could not converge to a transform that aligns both point clouds");
  }

  geometry_msgs::msg::PoseStamped out;
  out.pose = tf2::toMsg(Eigen::Isometry3d{ align.getFinalTransformation().cast<double>() });
  out.header.frame_id = base_point_cloud_msg.value().header.frame_id;
  out.header.stamp = base_point_cloud_msg.value().header.stamp;
  setOutput("target_pose_in_base_frame", out);

  return true;
}


}  // namespace picknik_registration
