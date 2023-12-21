// Copyright 2023 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include <moveit_studio_agent/test/do_objective_test_fixture.hpp>
#include <moveit_studio_agent_msgs/srv/get_planning_groups.hpp>

namespace moveit_studio::agent::testing
{

TEST_F(ObjectiveFixture, CallGetPlanningGroupsService)
{
  rclcpp::Client<moveit_studio_agent_msgs::srv::GetPlanningGroups>::SharedPtr client =
      node_->create_client<moveit_studio_agent_msgs::srv::GetPlanningGroups>("get_planning_groups");

  // Send request to /get_planning_groups service.
  auto request = std::make_shared<moveit_studio_agent_msgs::srv::GetPlanningGroups::Request>();
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  auto result = client->async_send_request(request);

  // Wait and check response.
  std::future_status status = result.wait_for(std::chrono::seconds(1));
  ASSERT_EQ(status, std::future_status::ready);
  ASSERT_TRUE(result.valid());
  ASSERT_GT(result.get()->planning_groups.size(), 0);
}

}  // namespace moveit_studio::agent::testing

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
