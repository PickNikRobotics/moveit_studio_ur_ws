#include <trigger_pstop_reset_service/trigger_pstop_reset_service.hpp>

namespace trigger_pstop_reset_service
{
TriggerPstopResetService::TriggerPstopResetService(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}


BT::PortsList TriggerPstopResetService::providedPorts()
{
  // TODO(...)
  return BT::PortsList({});
}

BT::KeyValueVector TriggerPstopResetService::metadata()
{
  // TODO(...)
  return { {"description", "Resets the UR P-stop status by calling the reset service named /recover_from_protective_stop."} };
}

BT::NodeStatus TriggerPstopResetService::tick()
{
  // TODO(...)
  // Return SUCCESS once the work has been completed.
  return BT::NodeStatus::SUCCESS;
}

}  // namespace trigger_pstop_reset_service
