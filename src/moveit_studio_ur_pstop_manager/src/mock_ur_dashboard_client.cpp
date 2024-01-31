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

#include <moveit_studio_ur_pstop_manager/mock_ur_dashboard_client.hpp>

namespace moveit_studio::ur_pstop_manager
{
constexpr auto kNodeName = "dashboard_client";
constexpr auto kSafetyModeService = "~/get_safety_mode";
constexpr auto kProgramRunningService = "~/program_running";

constexpr auto kUnlockProtectiveStopService = "/dashboard_client/unlock_protective_stop";
constexpr auto kStopProgramService = "/dashboard_client/stop";

MockURDashboardClient::MockURDashboardClient(const rclcpp::NodeOptions& options)
  : rclcpp::Node(kNodeName, options)
  , safety_mode_service_(create_service<ur_dashboard_msgs::srv::GetSafetyMode>(
        kSafetyModeService,
        [this](const ur_dashboard_msgs::srv::GetSafetyMode::Request::SharedPtr request,
               ur_dashboard_msgs::srv::GetSafetyMode::Response::SharedPtr response) {
          handleSafetyModeQuery(request, response);
        }))
  , program_running_service_(create_service<ur_dashboard_msgs::srv::IsProgramRunning>(
        kProgramRunningService,
        [this](const ur_dashboard_msgs::srv::IsProgramRunning::Request::SharedPtr request,
               ur_dashboard_msgs::srv::IsProgramRunning::Response::SharedPtr response) {
          handleRunningQuery(request, response);
        }))
  , unlock_pstop_service_(create_service<std_srvs::srv::Trigger>(
        kUnlockProtectiveStopService,
        [this](const std_srvs::srv::Trigger::Request::SharedPtr request,
               std_srvs::srv::Trigger::Response::SharedPtr response) { handleUnlockPStopQuery(request, response); }))
  , stop_program_service_(create_service<std_srvs::srv::Trigger>(
        kStopProgramService,
        [this](const std_srvs::srv::Trigger::Request::SharedPtr request,
               std_srvs::srv::Trigger::Response::SharedPtr response) { handleStopProgramQuery(request, response); }))
{
}

void MockURDashboardClient::handleSafetyModeQuery(
    const ur_dashboard_msgs::srv::GetSafetyMode::Request::SharedPtr /*request*/,
    ur_dashboard_msgs::srv::GetSafetyMode::Response::SharedPtr response)
{
  response->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL;
  response->success = true;
}

void MockURDashboardClient::handleRunningQuery(
    const ur_dashboard_msgs::srv::IsProgramRunning::Request::SharedPtr /*request*/,
    ur_dashboard_msgs::srv::IsProgramRunning::Response::SharedPtr response)
{
  response->program_running = true;
  response->success = true;
}

void MockURDashboardClient::handleUnlockPStopQuery(const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
                                                   std_srvs::srv::Trigger::Response::SharedPtr response) const
{
  response->success = true;
}

void MockURDashboardClient::handleStopProgramQuery(const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
                                                   std_srvs::srv::Trigger::Response::SharedPtr response) const
{
  response->success = true;
}
}  // namespace moveit_studio::ur_pstop_manager
