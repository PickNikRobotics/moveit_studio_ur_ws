#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <call_my_service/call_my_service.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace call_my_service
{
class CallMyServiceBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<CallMyService>(factory, "CallMyService", shared_resources);
    
  }
};
}  // namespace call_my_service

PLUGINLIB_EXPORT_CLASS(call_my_service::CallMyServiceBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
