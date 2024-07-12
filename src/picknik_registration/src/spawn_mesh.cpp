#include <picknik_registration/spawn_mesh.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <shape_msgs/msg/mesh.hpp>

#include <moveit_studio_behavior_interface/check_for_error.hpp>
namespace picknik_registration
{
SpawnSTL::SpawnSTL(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}


BT::PortsList SpawnSTL::providedPorts()
{
  // TODO(...)
  return { BT::InputPort<std::string>("file_name", "~/user_ws/file.stl", "The STL file to load"),
           BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "{pose}", "PoseStamped of the Object in the desired frame.")};
}

BT::KeyValueVector SpawnSTL::metadata()
{
  // TODO(...)
  return { { "subcategory", "Custom" }, {"description", "This is my new updated description"} };
}

BT::NodeStatus SpawnSTL::tick()
{
  const auto pose_stamped = getInput<geometry_msgs::msg::PoseStamped>("pose");
  const auto file_name = getInput<std::string>("file_name");

  // Check that all required input data ports were set
  if (const auto error = moveit_studio::behaviors::maybe_error(pose_stamped, file_name))
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " + error.value());
    return BT::NodeStatus::FAILURE;
  }
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Create a MoveGroupInterface object for the robot's planning group
  static const std::string PLANNING_GROUP = "manipulator";

  const auto& object_pose = pose_stamped.value();

  auto mesh = shapes::createMeshFromResource("file://" + file_name.value());
  // Create a CollisionObject message
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = object_pose.header.frame_id;
  collision_object.id = "stl_object";

  shape_msgs::msg::Mesh  mesh_msg;
  shapes::ShapeMsg mesh_msg_shape;
  shapes::constructMsgFromShape(mesh, mesh_msg_shape);
  mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_msg_shape);

  collision_object.meshes.push_back(mesh_msg);
  collision_object.mesh_poses.push_back(object_pose.pose);
  collision_object.operation = collision_object.ADD;

  // Add the collision object to the planning scene
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface.addCollisionObjects(collision_objects);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace picknik_registration
