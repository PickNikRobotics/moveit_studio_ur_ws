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

#include <moveit_pro_agent/test/do_objective_test_fixture.hpp>

#include <chrono>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <moveit_pro_agent_msgs/srv/set_transform.hpp>
#include <moveit_pro_sdk_msgs/msg/behavior_parameter.hpp>
#include <moveit_pro_agent_msgs/srv/store_joint_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
using BehaviorParameter = moveit_pro_sdk_msgs::msg::BehaviorParameter;
using BehaviorParameterDescription = moveit_pro_sdk_msgs::msg::BehaviorParameterDescription;

constexpr auto kServiceWaitTime = std::chrono::seconds{ 1 };
}  // namespace

namespace moveit_pro::agent::testing
{
using Point = geometry_msgs::msg::Point;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using StoreJointState = moveit_pro_agent_msgs::srv::StoreJointState;
using Quaternion = geometry_msgs::msg::Quaternion;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Vector3 = geometry_msgs::msg::Vector3;

TEST_F(ObjectiveFixture, Trivial)
{
  // This just tests if the fixture can do its setup and teardown completely even if the test finishes very quickly.
  ASSERT_TRUE(true);
}

TEST_F(ObjectiveFixture, InterpolateToPinchJointState)
{
  ASSERT_TRUE(setupDoObjectiveSequenceClient());

  // Store a joint state parameter
  rclcpp::Client<StoreJointState>::SharedPtr store_client = node_->create_client<StoreJointState>("store_joint_state");
  ASSERT_TRUE(store_client->wait_for_service(kServiceWaitTime));

  auto store_request = std::make_shared<StoreJointState::Request>();
  store_request->joint_state.name = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };
  store_request->joint_state.position = { -0.66, -2.10, -1.27, 0.60, 2.87, 0.36 };

  auto store_result_future = store_client->async_send_request(store_request);
  ASSERT_EQ(store_result_future.wait_for(kServiceWaitTime), std::future_status::ready)
      << "Timed out waiting for store joint state service response.";
  ASSERT_TRUE(store_result_future.get()->success);

  // Run the Objective
  auto do_objective_goal = std::make_unique<DoObjectiveSequence::Goal>();
  do_objective_goal->objective_name = "Interpolate to Joint State";

  // Expect to fail due to pinch links blocking entering a pinch stop zone
  EXPECT_FALSE(sendDoObjectiveSequenceGoal(std::move(do_objective_goal), 30.0));
}

TEST_F(ObjectiveFixture, InterpolateToNearPinchJointState)
{
  ASSERT_TRUE(setupDoObjectiveSequenceClient());

  // Store a joint state parameter
  rclcpp::Client<StoreJointState>::SharedPtr store_client = node_->create_client<StoreJointState>("store_joint_state");
  ASSERT_TRUE(store_client->wait_for_service(kServiceWaitTime));

  auto store_request = std::make_shared<StoreJointState::Request>();
  store_request->joint_state.name = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };
  store_request->joint_state.position = { -0.66, -2.10, -1.27, 0.60, 2.70, 0.36 };

  auto store_result_future = store_client->async_send_request(store_request);
  ASSERT_EQ(store_result_future.wait_for(kServiceWaitTime), std::future_status::ready)
      << "Timed out waiting for store joint state service response.";
  ASSERT_TRUE(store_result_future.get()->success);

  // Run the Objective
  auto do_objective_goal = std::make_unique<DoObjectiveSequence::Goal>();
  do_objective_goal->objective_name = "Interpolate to Joint State";

  // Expect the pinch link geometries to be unobtrusive enough that planning to just outside the
  // pinch zone still works.
  EXPECT_TRUE(sendDoObjectiveSequenceGoal(std::move(do_objective_goal), 30.0));
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
