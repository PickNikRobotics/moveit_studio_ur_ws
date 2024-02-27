// Copyright 2023 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <moveit_studio_ur_pstop_manager/protective_stop_manager_node.hpp>

namespace moveit_studio::ur_pstop_manager
{
constexpr auto kNodeName = "protective_stop_manager";
static const auto kLogger = rclcpp::get_logger(kNodeName);
constexpr auto kIsProgramRunningService = "/dashboard_client/program_running";
constexpr auto kSwitchControllerService = "/controller_manager/switch_controller";
constexpr auto kResendRobotProgramService = "/io_and_status_controller/resend_robot_program";
constexpr auto kUnlockProtectiveStopService = "/dashboard_client/unlock_protective_stop";
constexpr auto kStopProgramService = "/dashboard_client/stop";
constexpr auto kRecoveryServiceName = "recover_from_protective_stop";
constexpr auto kSafetyModeService = "/dashboard_client/get_safety_mode";
constexpr auto kFaultStatusTopic = "~/robot_fault_status";
constexpr auto kServiceCallTimeout = std::chrono::duration<double>(3.0);

constexpr auto kParameterControllersDefaultActive = "controllers_default_active";
constexpr auto kParameterControllersDefaultNotActive = "controllers_default_not_active";

constexpr auto kMaxResetPStopAttempts = 6;

ProtectiveStopManager::ProtectiveStopManager(const rclcpp::NodeOptions& options)
  : rclcpp::Node("protective_stop_manager", options)
  , reentrant_callback_group_(create_callback_group(rclcpp::CallbackGroupType::Reentrant))
  , recovery_service_(create_service<std_srvs::srv::Trigger>(
        kRecoveryServiceName,
        [this](const std_srvs::srv::Trigger::Request::SharedPtr request,
               std_srvs::srv::Trigger::Response::SharedPtr response) { recoverFromProtectiveStop(request, response); },
        rmw_qos_profile_services_default, reentrant_callback_group_))
  , is_program_running_client_(create_client<ur_dashboard_msgs::srv::IsProgramRunning>(
        kIsProgramRunningService, rmw_qos_profile_services_default, reentrant_callback_group_))
  , switch_controller_client_(create_client<SwitchController>(
        kSwitchControllerService, rmw_qos_profile_services_default, reentrant_callback_group_))
  , resend_program_client_(create_client<std_srvs::srv::Trigger>(
        kResendRobotProgramService, rmw_qos_profile_services_default, reentrant_callback_group_))
  , stop_program_client_(create_client<std_srvs::srv::Trigger>(kStopProgramService, rmw_qos_profile_services_default,
                                                               reentrant_callback_group_))
  , unlock_pstop_client_(create_client<std_srvs::srv::Trigger>(
        kUnlockProtectiveStopService, rmw_qos_profile_services_default, reentrant_callback_group_))
  , get_safety_mode_client_(create_client<ur_dashboard_msgs::srv::GetSafetyMode>(
        kSafetyModeService, rmw_qos_profile_services_default, reentrant_callback_group_))
  , fault_status_publisher_(create_publisher<moveit_studio_agent_msgs::msg::FaultStatus>(kFaultStatusTopic, 1))
  , fault_status_timer_(create_wall_timer(kServiceCallTimeout, [this] { this->publishFaultStatus(); }))
{
  declare_parameter<std::vector<std::string>>(kParameterControllersDefaultActive, std::vector<std::string>{});
  declare_parameter<std::vector<std::string>>(kParameterControllersDefaultNotActive, std::vector<std::string>{});

  // Retrieve list of controllers which should be active by default
  active_controller_names = get_parameter(kParameterControllersDefaultActive).as_string_array();

  // Set of all controllers = default-active controllers + default-inactive controllers
  all_controller_names = get_parameter(kParameterControllersDefaultNotActive).as_string_array();
  all_controller_names.insert(all_controller_names.end(), active_controller_names.cbegin(),
                              active_controller_names.cend());
}

void ProtectiveStopManager::recoverFromProtectiveStop(const std_srvs::srv::Trigger::Request::SharedPtr /* request */,
                                                      std_srvs::srv::Trigger::Response::SharedPtr response)
{
  // Unlock the protective stop.
  if (indicateUnavailableService(unlock_pstop_client_, response))
  {
    return;
  }

  // Older version of UR dashboard client does not accept reset protective stop requests unless 5 seconds has passed.
  // Hence, retry sending the p-stop request every second until it clears.
  for (std::size_t i = 0; i < kMaxResetPStopAttempts; i++)
  {
    RCLCPP_INFO_STREAM(kLogger, "Sending Unlock protective stop request");
    auto unlock_pstop_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto unlock_pstop_response_future = unlock_pstop_client_->async_send_request(unlock_pstop_request);

    if (unlock_pstop_response_future.wait_for(kServiceCallTimeout) == std::future_status::timeout)
    {
      const auto msg = std::string("Timed out waiting for response from `") + kUnlockProtectiveStopService +
                       "`service. You may need to restart the robot drivers manually.";
      RCLCPP_ERROR_STREAM(kLogger, msg);
      response->success = false;
      response->message = msg;
      break;
    }

    if (unlock_pstop_response_future.get()->success)
    {
      RCLCPP_INFO_STREAM(kLogger, "Unlock Pstop request returned success");
      response->success = true;
      break;
    }

    response->success = false;
    response->message = "Unlock Protective Stop request returned failure";

    RCLCPP_INFO_STREAM(kLogger, "Unlock Pstop request returned failure. Retrying again..");
    sleep(1);  // Try unlocking p-stop again after 1 sec
  }

  // Check if success was set to false in the for loop above and return early if applicable.
  if (!response->success)
  {
    RCLCPP_INFO_STREAM(kLogger, "Tried to unlock p-stop multiple times but was unsuccessful");
    return;
  }

  // Stop the program running on the UR before re-sending it.
  RCLCPP_INFO_STREAM(kLogger, "Stopping previous UR control program...");
  if (indicateUnavailableService(stop_program_client_, response))
  {
    return;
  }
  auto stop_program_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto stop_program_response_future = stop_program_client_->async_send_request(stop_program_request);
  if (stop_program_response_future.wait_for(kServiceCallTimeout) == std::future_status::timeout)
  {
    const auto msg = std::string("Timed out waiting for response from `") + kStopProgramService +
                     "` service. You may need to restart the robot drivers manually.";
    RCLCPP_ERROR_STREAM(kLogger, msg);
    response->success = false;
    response->message = msg;
    return;
  }

  // Deactivate currently-active controllers.
  // This is needed to prevent any controllers from commanding the robot hardware to a stale position when the UR
  // control program is re-launched.
  RCLCPP_INFO_STREAM(kLogger, "Deactivating all controllers...");
  if (indicateUnavailableService(switch_controller_client_, response))
  {
    return;
  }
  auto deactivate_all_controllers_request = std::make_shared<SwitchController::Request>();
  deactivate_all_controllers_request->deactivate_controllers = all_controller_names;
  // Use BEST_EFFORT strictness because some controllers may already be inactive, and attempting to stop them while
  // using STRICT strictness would introduce an unnecessary error.
  deactivate_all_controllers_request->strictness = SwitchController::Request::BEST_EFFORT;
  auto deactivate_all_controllers_future =
      switch_controller_client_->async_send_request(deactivate_all_controllers_request);
  if (deactivate_all_controllers_future.wait_for(kServiceCallTimeout) == std::future_status::timeout)
  {
    const auto msg = std::string("Timed out waiting for response from `") +
                     switch_controller_client_->get_service_name() +
                     "` service. You may need to restart the robot drivers manually.";
    RCLCPP_ERROR_STREAM(kLogger, msg);
    response->success = false;
    response->message = msg;
    return;
  }
  if (!deactivate_all_controllers_future.get()->ok)
  {
    const auto msg = "Failed to deactivate robot controllers.";
    RCLCPP_ERROR_STREAM(kLogger, msg);
    response->success = false;
    response->message = msg;
    return;
  }

  // Reactivate controllers set in the site config as active by default.
  // This resets the state of the robot controllers to match the state it was in at startup.
  RCLCPP_INFO_STREAM(kLogger, "Reactivating default controllers...");
  auto activate_default_controllers_request = std::make_shared<SwitchController::Request>();
  activate_default_controllers_request->activate_controllers = active_controller_names;
  // BEST_EFFORT is good enough to put the system back into an operational state without throwing unnecessary errors.
  activate_default_controllers_request->strictness = SwitchController::Request::BEST_EFFORT;
  auto activate_default_controllers_future =
      switch_controller_client_->async_send_request(activate_default_controllers_request);
  if (activate_default_controllers_future.wait_for(kServiceCallTimeout) == std::future_status::timeout)
  {
    const auto msg = std::string("Timed out waiting for response from `") +
                     switch_controller_client_->get_service_name() +
                     "` service. You may need to restart the robot drivers manually.";
    RCLCPP_ERROR_STREAM(kLogger, msg);
    response->success = false;
    response->message = msg;
    return;
  }
  if (!activate_default_controllers_future.get()->ok)
  {
    const auto msg = "Failed to activate default robot controllers.";
    RCLCPP_ERROR_STREAM(kLogger, msg);
    response->success = false;
    response->message = msg;
    return;
  }

  // Resend the robot program.
  RCLCPP_INFO_STREAM(kLogger, "Sending program to robot...");
  if (indicateUnavailableService(resend_program_client_, response))
  {
    return;
  }
  auto resend_program_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto resend_program_response_future = resend_program_client_->async_send_request(resend_program_request);
  if (resend_program_response_future.wait_for(kServiceCallTimeout) == std::future_status::timeout)
  {
    const auto msg = std::string("Timed out waiting for response from `") + kResendRobotProgramService +
                     "` service. You may need to restart the robot drivers manually.";
    RCLCPP_ERROR_STREAM(kLogger, msg);
    response->success = false;
    response->message = msg;
    return;
  }

  if (!resend_program_response_future.get()->success)
  {
    const auto msg = "Failed to resend robot program.";
    RCLCPP_ERROR_STREAM(kLogger, msg);
    response->success = false;
    response->message = msg;
    return;
  }

  // Adding a sleep here so that the UR control program has some time to come up.
  sleep(1);

  // Make sure that the program is running.
  auto program_running_opt = isProgramRunning(response);
  if (!program_running_opt.has_value())
  {
    return;
  }

  auto program_running = program_running_opt.value();

  // Indicate success/failure.
  response->success = program_running;
  if (!program_running)
  {
    const auto msg = std::string("Program was sent to the robot, but failed to start.");
    RCLCPP_ERROR_STREAM(kLogger, msg);
    response->message = msg;
  }
}

bool ProtectiveStopManager::indicateUnavailableService(rclcpp::ClientBase::SharedPtr client,
                                                       std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (client->service_is_ready())
  {
    return false;
  }

  const auto msg = std::string("`") + client->get_service_name() +
                   "` service is not available. You may need to restart the robot drivers manually.";
  RCLCPP_ERROR_STREAM(kLogger, msg);
  if (response)
  {
    response->success = false;
    response->message = msg;
  }
  return true;
}

void ProtectiveStopManager::publishFaultStatus()
{
  if (indicateUnavailableService(get_safety_mode_client_))
  {
    return;
  }

  auto safety_mode_request = std::make_shared<ur_dashboard_msgs::srv::GetSafetyMode::Request>();
  auto safety_mode_response_future = get_safety_mode_client_->async_send_request(safety_mode_request);
  if (safety_mode_response_future.wait_for(kServiceCallTimeout) == std::future_status::timeout)
  {
    RCLCPP_ERROR_STREAM(kLogger, "Timed out waiting for response from `"
                                     << kSafetyModeService
                                     << "` service. You may need to restart the robot drivers manually.");
    return;
  }

  auto safety_mode_response = safety_mode_response_future.get();
  if (!safety_mode_response->success)
  {
    RCLCPP_ERROR_STREAM(kLogger, "`" << kSafetyModeService << "` did not return successfully.");
    return;
  }

  auto msg = moveit_studio_agent_msgs::msg::FaultStatus();
  switch (safety_mode_response->safety_mode.mode)
  {
    case ur_dashboard_msgs::msg::SafetyMode::NORMAL:
      msg.status = moveit_studio_agent_msgs::msg::FaultStatus::NORMAL;
      break;
    case ur_dashboard_msgs::msg::SafetyMode::SYSTEM_EMERGENCY_STOP:
    case ur_dashboard_msgs::msg::SafetyMode::ROBOT_EMERGENCY_STOP:
      msg.status = moveit_studio_agent_msgs::msg::FaultStatus::NONRECOVERABLE_FAULT;
      break;
    case ur_dashboard_msgs::msg::SafetyMode::PROTECTIVE_STOP:
      msg.status = moveit_studio_agent_msgs::msg::FaultStatus::RECOVERABLE_FAULT;
      break;
    // Treat anything we don't recognize as a recoverable fault, because we might as well let the user try to recover it.
    default:
      msg.status = moveit_studio_agent_msgs::msg::FaultStatus::RECOVERABLE_FAULT;
      break;
  }

  // If the program is not running, indicate a recoverable fault, even if the robot is not in a protective stop state.
  if (!isProgramRunning().value_or(true) &&
      msg.status != moveit_studio_agent_msgs::msg::FaultStatus::NONRECOVERABLE_FAULT)
  {
    msg.status = moveit_studio_agent_msgs::msg::FaultStatus::RECOVERABLE_FAULT;
  }

  fault_status_publisher_->publish(msg);
}

std::optional<bool> ProtectiveStopManager::isProgramRunning(std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (indicateUnavailableService(is_program_running_client_, response))
  {
    return {};
  }

  auto is_program_running_request = std::make_shared<ur_dashboard_msgs::srv::IsProgramRunning::Request>();
  auto is_program_running_response_future = is_program_running_client_->async_send_request(is_program_running_request);
  if (is_program_running_response_future.wait_for(kServiceCallTimeout) == std::future_status::timeout)
  {
    const auto msg = std::string("Timed out waiting for response from `") + kIsProgramRunningService +
                     "` service. You may need to restart the robot drivers manually.";
    RCLCPP_ERROR_STREAM(kLogger, msg);
    if (response)
    {
      response->success = false;
      response->message = msg;
    }
    return false;
  }

  return is_program_running_response_future.get()->program_running;
}
}  // namespace moveit_studio::ur_pstop_manager

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<moveit_studio::ur_pstop_manager::ProtectiveStopManager>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  exec.remove_node(node);

  rclcpp::shutdown();
}
