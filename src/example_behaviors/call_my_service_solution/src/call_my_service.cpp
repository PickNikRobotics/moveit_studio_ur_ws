#include <call_my_service/call_my_service.hpp>

// Include the template implementation for ServiceClientBehaviorBase<T>.
#include <moveit_studio_behavior_interface/impl/service_client_behavior_base_impl.hpp>

namespace call_my_service
{
CallMyService::CallMyService(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : ServiceClientBehaviorBase<service_interface::srv::ServiceInterface>(name, config, shared_resources)
{
}

BT::PortsList CallMyService::providedPorts()
{
  // This node has three input ports and one output port
  return BT::PortsList({
    BT::InputPort<std::string>("service_name"),
    BT::OutputPort<bool>("result"),
  });
}

BT::KeyValueVector CallMyService::metadata()
{
  return { { "subcategory", "ROS Messaging" },
           { "description", "Example of calling a ROS2 service." } };
}

tl::expected<std::string, std::string> CallMyService::getServiceName()
{
  const auto service_name = getInput<std::string>("service_name");
  if (const auto error = moveit_studio::behaviors::maybe_error(service_name))
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + error.value());
  }
  return service_name.value();
}

tl::expected<ServiceInterface::Request, std::string> CallMyService::createRequest(){
  return service_interface::build<ServiceInterface::Request>();
}

tl::expected<bool, std::string> CallMyService::processResponse(const ServiceInterface::Response& response){
  setOutput<bool>("result", response.success);
  return { response.success };
}
}  // namespace call_my_service
