#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <trigger_pstop_reset_service/trigger_pstop_reset_service.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace trigger_pstop_reset_service
{
class TriggerPStopResetServiceBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<TriggerPStopResetService>(factory, "TriggerPStopResetService", shared_resources);
    
  }
};
}  // namespace trigger_pstop_reset_service

PLUGINLIB_EXPORT_CLASS(trigger_pstop_reset_service::TriggerPStopResetServiceBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
