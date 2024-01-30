// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

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
