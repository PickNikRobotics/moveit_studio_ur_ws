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

// Implementing this feature as a node that provides a service that calls other ROS services is not ideal.  Please see
// the README for a discussion of alternatives.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <moveit_studio_agent_msgs/msg/fault_status.hpp>
#include <ur_dashboard_msgs/srv/get_safety_mode.hpp>
#include <ur_dashboard_msgs/srv/is_program_running.hpp>

namespace moveit_studio::ur_pstop_manager
{
class ProtectiveStopManager : public rclcpp::Node
{
public:
  ProtectiveStopManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  using SwitchController = controller_manager_msgs::srv::SwitchController;

  /**
   * @brief This is the callback for the "recover_from_protective_stop" service.  It unlocks the protective stop, stops
   * the program currently running on the arm, and re-sends the control program.
   *
   * @param request An empty request.
   * @param response Indicates whether the protective stop was successfully released.
   */
  void recoverFromProtectiveStop(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                 std_srvs::srv::Trigger::Response::SharedPtr response);

  /**
   * @brief Helper function to check whether a service is unavailable.  If the service is unavailable, this function
   * also sets displays the appropriate error message and sets the Response.
   *
   * @param client The client for the service we are checking.
   * @param response The response object, which will indicate success or failure.
   * @return true The service is unavailable.
   * @return false The service is available.
   */
  bool indicateUnavailableService(rclcpp::ClientBase::SharedPtr client,
                                  std_srvs::srv::Trigger::Response::SharedPtr response = nullptr);

  /**
   * @brief Callback function that publishes the current fault status of the robot.
   *
   */
  void publishFaultStatus();

  /**
   * @brief Determines whether the UR control program is currently running.
   *
   * @return true The program is running.
   * @return false The program is not running.
   * @return std::nullopt An error occurred when attempting to call the service.
   */
  std::optional<bool> isProgramRunning(std_srvs::srv::Trigger::Response::SharedPtr response = nullptr);

  std::vector<std::string> all_controller_names;
  std::vector<std::string> active_controller_names;

  rclcpp::CallbackGroup::SharedPtr reentrant_callback_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recovery_service_;
  rclcpp::Client<ur_dashboard_msgs::srv::IsProgramRunning>::SharedPtr is_program_running_client_;
  rclcpp::Client<SwitchController>::SharedPtr switch_controller_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resend_program_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_program_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr unlock_pstop_client_;
  rclcpp::Client<ur_dashboard_msgs::srv::GetSafetyMode>::SharedPtr get_safety_mode_client_;
  rclcpp::Publisher<moveit_studio_agent_msgs::msg::FaultStatus>::SharedPtr fault_status_publisher_;
  rclcpp::TimerBase::SharedPtr fault_status_timer_;
};
}  // namespace moveit_studio::ur_pstop_manager
