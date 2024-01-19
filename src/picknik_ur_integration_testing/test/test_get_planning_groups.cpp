// Copyright 2023 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include <moveit_pro_agent/test/do_objective_test_fixture.hpp>
#include <moveit_pro_agent_msgs/srv/get_planning_groups.hpp>

namespace moveit_pro::agent::testing
{

TEST_F(ObjectiveFixture, CallGetPlanningGroupsService)
{
  rclcpp::Client<moveit_pro_agent_msgs::srv::GetPlanningGroups>::SharedPtr client =
      node_->create_client<moveit_pro_agent_msgs::srv::GetPlanningGroups>("get_planning_groups");

  // Send request to /get_planning_groups service.
  auto request = std::make_shared<moveit_pro_agent_msgs::srv::GetPlanningGroups::Request>();
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  auto result = client->async_send_request(request);

  // Wait and check response.
  std::future_status status = result.wait_for(std::chrono::seconds(1));
  ASSERT_EQ(status, std::future_status::ready);
  ASSERT_TRUE(result.valid());
  ASSERT_GT(result.get()->planning_groups.size(), 0);
}

}  // namespace moveit_pro::agent::testing

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
