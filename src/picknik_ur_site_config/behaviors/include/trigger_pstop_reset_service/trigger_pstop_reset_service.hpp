#pragma once

#include <moveit_studio_behavior_interface/service_client_behavior_base.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace trigger_pstop_reset_service
{
/**
 * @brief Resets the UR P-stop status by calling the reset service named /recover_from_protective_stop.
 */
using PStopService = std_srvs::srv::Trigger;
class TriggerPStopResetService : public moveit_studio::behaviors::ServiceClientBehaviorBase<PStopService>
{
public:
/**
   * @brief Constructor for the trigger_pstop_reset_service behavior.
   * @param name The name of a particular instance of this Behavior. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @param config This contains runtime configuration info for this Behavior, such as the mapping between the Behavior's data ports on the behavior tree's blackboard. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @details An important limitation is that the members of the base Behavior class are not instantiated until after the initialize() function is called, so these classes should not be used within the constructor.
   */
 TriggerPStopResetService(const std::string& name, const BT::NodeConfiguration& config,
 const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

 /**
  * @brief Implementation of the required providedPorts() function for the trigger_pstop_reset_service Behavior.
  * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function named providedPorts() which defines their input and output ports. If the Behavior does not use any ports, this function must return an empty BT::PortsList.
  * This function returns a list of ports with their names and port info, which is used internally by the behavior tree.
  * @return trigger_pstop_reset_service does not use expose any ports, so this function returns an empty list.
  */
 static BT::PortsList providedPorts();

 /**
  * @brief Implementation of the metadata() function for displaying metadata, such as Behavior description and
  * subcategory, in the MoveIt Studio Developer Tool.
  * @return A BT::KeyValueVector containing the Behavior metadata.
  */
 static BT::KeyValueVector metadata();

private:
 /**
 * @brief Specify the name of the service that will be called by the behaviors
 * @return Returns the name of the service. If not successful, returns an error message.
 */
 tl::expected<std::string, std::string> getServiceName() override;
 /**
 * @brief Sets the timeout for the service response.
 * @details If the timeout expires before a response is received, the behavior will fail. A negative duration indicates no timeout.
 * @return Returns the service response timeout duration.
 */
 tl::expected<std::chrono::duration<double>, std::string> getResponseTimeout() override;
 /**
 * @brief Create a service request.
 * @return Returns a service request message. If not successful, returns an error message.
 */
 tl::expected<std_srvs::srv::Trigger::Request, std::string> createRequest() override;
 /**
 * @brief Determines if the service request succeeded or failed based on the response message.
 * @param response Response message received from the service server.
 * @return Returns true if the response indicates success. If not successful, returns an error message.
 */
 tl::expected<bool, std::string> processResponse(const std_srvs::srv::Trigger::Response&) override;

 /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
 std::shared_future<tl::expected<bool, std::string>>& getFuture() override;

 /**
 * @brief Holds a copy of the service name specified by the input port.
 */
 std::string service_name_;

 /**
  * @brief Holds the result of calling the service asynchronously.
  */
 std::shared_future<tl::expected<bool, std::string>> future_;

  
};
}  // namespace trigger_pstop_reset_service
