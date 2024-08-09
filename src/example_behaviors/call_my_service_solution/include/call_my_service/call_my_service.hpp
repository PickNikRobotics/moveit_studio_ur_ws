#pragma once

#include <moveit_studio_behavior_interface/service_client_behavior_base.hpp>
#include <service_interface/srv/service_interface.hpp>

using moveit_studio::behaviors::BehaviorContext;
using moveit_studio::behaviors::ServiceClientBehaviorBase;
using ServiceInterface = service_interface::srv::ServiceInterface;

namespace call_my_service
{
class CallMyService final : public ServiceClientBehaviorBase<ServiceInterface>
{
public:
  CallMyService(const std::string& name, const BT::NodeConfiguration& config,
            const std::shared_ptr<BehaviorContext>& shared_resources);

  static BT::KeyValueVector metadata();
  
  static BT::PortsList providedPorts();

private:
  /** @brief User-provided function to get the name of the service when initializing the service client. */
  tl::expected<std::string, std::string> getServiceName() override;

  /** @brief User-provided function to create the service request. */
  tl::expected<ServiceInterface::Request, std::string> createRequest() override;

  /** @brief Optional user-provided function to process the service response after the service has finished. */
  tl::expected<bool, std::string> processResponse(const ServiceInterface::Response& response) override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member */
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace call_my_service
