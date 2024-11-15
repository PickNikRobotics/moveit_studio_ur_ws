#include <translate_pose/translate_pose.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace translate_pose
{
TranslatePose::TranslatePose(const std::string& name, const BT::NodeConfiguration& config,
                             const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList TranslatePose::providedPorts()
{
  return BT::PortsList({ BT::InputPort<geometry_msgs::msg::PoseStamped>("pose_stamped"), BT::InputPort<double>("x"),
                         BT::InputPort<double>("y"), BT::InputPort<double>("z"),
                         BT::OutputPort<geometry_msgs::msg::PoseStamped>("new_pose") });
}

BT::KeyValueVector TranslatePose::metadata()
{
  return { {"subcategory", "Pose Handling"}, { "description", "Translates a PoseStamped, given x,y,z" } };
}

BT::NodeStatus TranslatePose::tick()
{
  shared_resources_->logger->publishWarnMessage("TranslatePose", "Called tick()");

  auto maybe_pose = getInput<geometry_msgs::msg::PoseStamped>("pose_stamped");
  auto maybe_x = getInput<double>("x");
  auto maybe_y = getInput<double>("y");
  auto maybe_z = getInput<double>("z");

  if (!maybe_pose || !maybe_x || !maybe_y || !maybe_z)
  {
    shared_resources_->logger->publishFailureMessage("TranslatePose", "Failed to get input data");
    return BT::NodeStatus::FAILURE;
  }

  double x = maybe_x.value();
  double y = maybe_y.value();
  double z = maybe_z.value();
  geometry_msgs::msg::PoseStamped pose = maybe_pose.value();

  shared_resources_->logger->publishWarnMessage("TranslatePose", "Should translate pose by x=" + std::to_string(x) +
                                                                     ", y=" + std::to_string(y) +
                                                                     ", z=" + std::to_string(z));
  pose.pose.position.x += x;
  pose.pose.position.y += y;
  pose.pose.position.z += z;

  setOutput("new_pose", pose);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace translate_pose
