// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <moveit_studio_ur_pstop_manager/mock_ur_dashboard_client.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<moveit_studio::ur_pstop_manager::MockURDashboardClient>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  exec.remove_node(node);

  rclcpp::shutdown();
}
