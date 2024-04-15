// Copyright 2024 PickNik Inc.
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

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <moveit_studio_agent_msgs/srv/set_transform.hpp>
#include <moveit_studio_agent_msgs/srv/store_joint_state.hpp>
#include <moveit_studio_internal_msgs/srv/request_points_from_user.hpp>

#include <moveit_studio_agent/test/do_objective_test_fixture.hpp>

namespace moveit_studio::agent::testing
{
using PointStamped = geometry_msgs::msg::PointStamped;


using RequestPointsFromUser = moveit_studio_internal_msgs::srv::RequestPointsFromUser;

TEST_F(ObjectiveFixture, TestPickObject)
{
  ASSERT_TRUE(setupDoObjectiveSequenceClient());

  const auto server = node_->create_service<RequestPointsFromUser>("/request_clicked_points", [](const std::shared_ptr<RequestPointsFromUser::Request>& req,
             const std::shared_ptr<RequestPointsFromUser::Response>& res) {
              res->status.success = true;
              const auto& name = req->point_names.front();
              // Set a grasp pose based on a user-selected grasp originally picked in simulation
              res->points.push_back([&name] {
    PointStamped msg;
    msg.header.frame_id = name;
    msg.point.x = 200;
    msg.point.y = 200;
    return msg;
  }());
  });

  auto do_objective_goal = std::make_unique<DoObjectiveSequence::Goal>();
  do_objective_goal->objective_name = "Pick Object";

  EXPECT_TRUE(sendDoObjectiveSequenceGoal(std::move(do_objective_goal), 30.0));
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
