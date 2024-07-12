#include <picknik_registration/ndt_registration.hpp>
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

NDTRegistration::NDTRegistration(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::AsyncBehaviorBase(name, config, shared_resources)
{
}


BT::PortsList NDTRegistration::providedPorts()
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
                                 "Resolution of NDT grid structure (VoxelGridCovariance) <double>"),
           BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_pose_in_base_frame", "{target_pose}",
                                                           "The pose of the target point cloud relative to the frame "
                                                           "of the base point cloud.") };
}
BT::KeyValueVector NDTRegistration::metadata()
{
  // TODO(...)
  return { {"description", "Finds the pose of a target point cloud relative to the base frame of a base point cloud using the Normal Distributions Transform (NDT) algoritm"} };
}


tl::expected<bool, std::string> NDTRegistration::doWork()
{
  const auto base_point_cloud_msg = getInput<sensor_msgs::msg::PointCloud2>("base_point_cloud");
  const auto target_point_cloud_msg = getInput<sensor_msgs::msg::PointCloud2>("target_point_cloud");
  const auto max_iterations = getInput<int>("max_iterations");
  const auto transformation_epsilon = getInput<double>("transformation_epsilon");
  const auto step_size = getInput<double>("step_size");
  const auto resolution = getInput<double>("resolution");

  if (const auto error =
          moveit_studio::behaviors::maybe_error(base_point_cloud_msg, target_point_cloud_msg, max_iterations, transformation_epsilon, step_size, resolution);
      error)
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + error.value());
  }

  const auto base_cloud = getFilteredPointCloudFromMessage(base_point_cloud_msg.value());
  if (base_cloud->empty())
  {
    return tl::make_unexpected("Base point cloud has no valid points");
  }
  const auto target_cloud = getFilteredPointCloudFromMessage(target_point_cloud_msg.value());
  if (target_cloud->empty())
  {
    return tl::make_unexpected("Target point cloud has no valid points");
  }
  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (transformation_epsilon.value());
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (step_size.value());
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (resolution.value());

  // Setting max number of registration iterations.
  ndt.setMaximumIterations (max_iterations.value());

  // Setting point cloud to be aligned.
  ndt.setInputSource (base_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (target_cloud);


  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  ndt.align (*output_cloud);

  if (!ndt.hasConverged())
  {
    return tl::make_unexpected("NDT could not converge to a transform that aligns both point clouds");
  }

  geometry_msgs::msg::PoseStamped out;
  out.pose = tf2::toMsg(Eigen::Isometry3d{ ndt.getFinalTransformation().cast<double>() });
  out.header.frame_id = base_point_cloud_msg.value().header.frame_id;
  out.header.stamp = base_point_cloud_msg.value().header.stamp;
  setOutput("target_pose_in_base_frame", out);

  return true;
}


}  // namespace picknik_registration
