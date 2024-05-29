#include <setup_mtc_place_pose/setup_mtc_place_pose.hpp>

namespace setup_mtc_place_pose
{
SetupMtcPlacePose::SetupMtcPlacePose(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}


BT::PortsList SetupMtcPlacePose::providedPorts()
{
  // TODO(...)
  return BT::PortsList({});
}

BT::KeyValueVector SetupMtcPlacePose::metadata()
{
  // TODO(...)
  return { {"description", "Adds a GeneratePlacePose stage"} };
}

BT::NodeStatus SetupMtcPlacePose::tick()
{
  // TODO(...)
  // Return SUCCESS once the work has been completed.
  return BT::NodeStatus::SUCCESS;
}

}  // namespace setup_mtc_place_pose
