#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <trigger_pstop_reset_service/trigger_pstop_reset_service.hpp>
#include <sam2_segmentation/sam2_segmentation.hpp>
#include <clipseg_segmentation/clipseg_segmentation.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace custom_behaviors{
class TriggerPStopResetServiceBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<TriggerPStopResetService>(factory, "TriggerPStopResetService", shared_resources);
    moveit_studio::behaviors::registerBehavior<SAM2Segmentation>(factory, "SAM2Segmentation", shared_resources);
    moveit_studio::behaviors::registerBehavior<ClipSegSegmentation>(factory, "ClipSegSegmentation", shared_resources);

  }
};
}
PLUGINLIB_EXPORT_CLASS(custom_behaviors::TriggerPStopResetServiceBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
