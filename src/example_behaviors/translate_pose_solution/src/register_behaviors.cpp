#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <translate_pose/translate_pose.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace translate_pose
{
class TranslatePoseBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<TranslatePose>(factory, "TranslatePose", shared_resources);
    
  }
};
}  // namespace translate_pose

PLUGINLIB_EXPORT_CLASS(translate_pose::TranslatePoseBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
