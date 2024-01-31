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
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <ur_dashboard_msgs/srv/get_safety_mode.hpp>
#include <ur_dashboard_msgs/srv/is_program_running.hpp>

namespace moveit_studio::ur_pstop_manager
{
/**
 * \brief This class is intended for use in simulation, and mocks some of the services that the DashboardClientROS class
 * provides for real hardware.
 */
class MockURDashboardClient : public rclcpp::Node
{
public:
  explicit MockURDashboardClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  /**
   * @brief Returns the current safety mode for the robot.  Always returns NORMAL, since our simulated robot does not
   * enter any faulted states.
   *
   * @param request The request object.
   * @param response The response, containing the robot's safety mode (always NORMAL).
   */
  void handleSafetyModeQuery(const ur_dashboard_msgs::srv::GetSafetyMode::Request::SharedPtr request,
                             ur_dashboard_msgs::srv::GetSafetyMode::Response::SharedPtr response);

  /**
   * @brief Indicates whether the UR control script is currently running.  Always returns true.
   *
   * @param request The request object.
   * @param response The response, indicating whether the control program is currently running (always returns true).
   */
  void handleRunningQuery(const ur_dashboard_msgs::srv::IsProgramRunning::Request::SharedPtr request,
                          ur_dashboard_msgs::srv::IsProgramRunning::Response::SharedPtr response);

  /**
   * @brief Resets the robot from protective stop.  Always returns true.
   *
   * @param request The request object.
   * @param response The response, indicating whether the protective stop reset was successful (always returns true).
   */
  void handleUnlockPStopQuery(const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
                              std_srvs::srv::Trigger::Response::SharedPtr response) const;

  /**
   * @brief Stops the UR control script.  Always returns true.
   *
   * @param request The request object.
   * @param response The response, indicating whether the UR control script was shutdown successfully (always returns true).
   */
  void handleStopProgramQuery(const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
                              std_srvs::srv::Trigger::Response::SharedPtr response) const;

  rclcpp::Service<ur_dashboard_msgs::srv::GetSafetyMode>::SharedPtr safety_mode_service_;
  rclcpp::Service<ur_dashboard_msgs::srv::IsProgramRunning>::SharedPtr program_running_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr unlock_pstop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resend_program_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_program_service_;
};
}  // namespace moveit_studio::ur_pstop_manager
