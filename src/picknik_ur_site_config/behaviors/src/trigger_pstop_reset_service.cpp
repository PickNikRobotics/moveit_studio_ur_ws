#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <trigger_pstop_reset_service/trigger_pstop_reset_service.hpp>

namespace
{
    constexpr auto kDefaultPStopServiceName = "/recover_from_protective_stop";
    constexpr auto kPortServiceName = "service_name";
}// namespace

namespace trigger_pstop_reset_service
{
    TriggerPStopResetService::TriggerPStopResetService(const std::string& name, const BT::NodeConfiguration& config,
                                                       const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>&
                                                       shared_resources)
        : ServiceClientBehaviorBase<PStopService>(name, config, shared_resources)
    {
    }

    tl::expected<std::string, std::string> TriggerPStopResetService::getServiceName()
    {
        // Get service name from the input port.
        const auto service_name = getInput<std::string>(kPortServiceName);
        // Check that the port has a value on it, if not, return an error.
        if (const auto error = moveit_studio::behaviors::maybe_error(service_name))
        {
            return tl::make_unexpected("Failed to get required value from input data port: " + error.value());
        }

        service_name_ = service_name.value();
        return service_name_;
    }

    tl::expected<std::chrono::duration<double>, std::string> TriggerPStopResetService::getResponseTimeout()
    {
        // Create timeout for service call.
        return std::chrono::duration<double>{2.0};
    }

    tl::expected<std_srvs::srv::Trigger::Request, std::string> TriggerPStopResetService::createRequest()
    {
        // Create request message.
        return PStopService::Request{};
    }

    tl::expected<bool, std::string> TriggerPStopResetService::processResponse(
        const std_srvs::srv::Trigger::Response& trigger_response)
    {
        // If the service response could not be processed, returns an error message, otherwise, return true to indicate success.
        return trigger_response.success
                   ? tl::expected<bool, std::string>(true)
                   : tl::make_unexpected(
                       "Failed to call P-Stop service `" + service_name_ + "`: " + trigger_response.message);
    }

    std::shared_future<tl::expected<bool, std::string>>& TriggerPStopResetService::getFuture()
    {
        return future_;
    }


    BT::PortsList TriggerPStopResetService::providedPorts()
    {
        return {
            BT::InputPort<std::string>(kPortServiceName, kDefaultPStopServiceName,
                                       "Name of the service to send a request to.")
        };
    }

    BT::KeyValueVector TriggerPStopResetService::metadata()
    {
        return {
            {
                "description",
                std::string(
                    "Resets the UR P-stop status by calling the reset service named service name specified by the input port named `")
                .append(kPortServiceName).append("`.")
            }
        };
    }
} // namespace trigger_pstop_reset_service
