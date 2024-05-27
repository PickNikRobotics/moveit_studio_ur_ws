#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <picknik_registration/ndt_registration.hpp>
#include <picknik_registration/ransac_registration.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace picknik_registration
{
class PicknikRegistrationBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<NDTRegistration>(factory, "NDTRegistration", shared_resources);
    moveit_studio::behaviors::registerBehavior<RANSACRegistration>(factory, "RANSACRegistration", shared_resources);
    
  }
};
}  // namespace picknik_registration

PLUGINLIB_EXPORT_CLASS(picknik_registration::PicknikRegistrationBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
