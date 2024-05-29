#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <setup_mtc_place_pose/setup_mtc_place_pose.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace setup_mtc_place_pose
{
class SetupMtcPlacePoseBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<SetupMtcPlacePose>(factory, "SetupMtcPlacePose", shared_resources);
    
  }
};
}  // namespace setup_mtc_place_pose

PLUGINLIB_EXPORT_CLASS(setup_mtc_place_pose::SetupMtcPlacePoseBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
