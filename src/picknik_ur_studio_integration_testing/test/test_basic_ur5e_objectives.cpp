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

#include <moveit_studio_agent/test/do_objective_test_fixture.hpp>

#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <moveit_studio_agent_msgs/srv/set_transform.hpp>
#include <moveit_studio_agent_msgs/srv/store_joint_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
using BehaviorParameter = moveit_studio_sdk_msgs::msg::BehaviorParameter;
using BehaviorParameterDescription = moveit_studio_sdk_msgs::msg::BehaviorParameterDescription;

constexpr auto kServiceWaitTime = std::chrono::seconds{ 1 };
}  // namespace

namespace moveit_pro::agent::testing
{
using Point = geometry_msgs::msg::Point;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using StoreJointState = moveit_studio_agent_msgs::srv::StoreJointState;
using Quaternion = geometry_msgs::msg::Quaternion;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Vector3 = geometry_msgs::msg::Vector3;

TEST_F(ObjectiveFixture, Trivial)
{
  // This just tests if the fixture can do its setup and teardown completely even if the test finishes very quickly.
  ASSERT_TRUE(true);
}

TEST_F(ObjectiveFixture, TestCloseGripper)
{
  ASSERT_TRUE(setupDoObjectiveSequenceClient());

  auto do_objective_goal = std::make_unique<DoObjectiveSequence::Goal>();
  do_objective_goal->objective_name = "Close Gripper";

  EXPECT_TRUE(sendDoObjectiveSequenceGoal(std::move(do_objective_goal)));
}

TEST_F(ObjectiveFixture, TestOpenGripper)
{
  ASSERT_TRUE(setupDoObjectiveSequenceClient());

  auto do_objective_goal = std::make_unique<DoObjectiveSequence::Goal>();
  do_objective_goal->objective_name = "Open Gripper";

  EXPECT_TRUE(sendDoObjectiveSequenceGoal(std::move(do_objective_goal)));
}

TEST_F(ObjectiveFixture, TestPickObject)
{
  ASSERT_TRUE(setupDoObjectiveSequenceClient());

  auto do_objective_goal = std::make_unique<DoObjectiveSequence::Goal>();
  do_objective_goal->objective_name = "Pick Object";

  // Set a grasp pose based on a user-selected grasp originally picked in simulation
  const auto grasp_pose = [] {
    PoseStamped msg;
    msg.header.frame_id = "world";
    msg.pose.position = geometry_msgs::build<Point>().x(0.58976525).y(-0.1367085).z(0.27501845);
    msg.pose.orientation = geometry_msgs::build<Quaternion>().x(0.7063694).y(0.44723025).z(0.29349735).w(0.46355873);
    return msg;
  }();

  // Update the grasp pose parameter used when planning the objective
  BehaviorParameter pose_name_parameter;
  pose_name_parameter.behavior_namespaces.emplace_back("pick_object");
  pose_name_parameter.description.name = "grasp_pose";
  pose_name_parameter.description.type = BehaviorParameterDescription::TYPE_POSE;
  pose_name_parameter.pose_value = grasp_pose;

  do_objective_goal->parameter_overrides.insert(do_objective_goal->parameter_overrides.end(), { pose_name_parameter });

  EXPECT_TRUE(sendDoObjectiveSequenceGoal(std::move(do_objective_goal), 30.0));
}

TEST_F(ObjectiveFixture, DISABLED_TestInspectSurface)
{
  using SetTransform = moveit_studio_agent_msgs::srv::SetTransform;

  ASSERT_TRUE(setupDoObjectiveSequenceClient());

  const auto set_transform_client = node_->create_client<SetTransform>("/set_transform");

  const auto transform = [] {
    TransformStamped msg;
    msg.header.frame_id = "world";
    msg.child_frame_id = "selected_frame";
    msg.transform.translation = geometry_msgs::build<Vector3>().x(0.48976525).y(-0.1367085).z(0.27501845);
    msg.transform.rotation = geometry_msgs::build<Quaternion>().x(0.7063694).y(0.44723025).z(0.29349735).w(0.46355873);
    return msg;
  }();
  const auto set_transform_req =
      std::make_shared<SetTransform::Request>(moveit_studio_agent_msgs::build<SetTransform::Request>()
                                                  .action(SetTransform::Request::SET_OR_UPDATE)
                                                  .tform(transform));

  auto set_transform_future = set_transform_client->async_send_request(set_transform_req);
  ASSERT_EQ(set_transform_future.wait_for(std::chrono::seconds(10)), std::future_status::ready);
  auto set_transform_result = set_transform_future.get();
  ASSERT_TRUE(set_transform_result->success);

  auto do_objective_goal = std::make_unique<DoObjectiveSequence::Goal>();
  do_objective_goal->objective_name = "Move to Target";

  EXPECT_TRUE(sendDoObjectiveSequenceGoal(std::move(do_objective_goal), 30.0));
}

TEST_F(ObjectiveFixture, TestMoveToJointState)
{
  ASSERT_TRUE(setupDoObjectiveSequenceClient());

  // Store a joint state parameter
  rclcpp::Client<StoreJointState>::SharedPtr store_client = node_->create_client<StoreJointState>("store_joint_state");
  ASSERT_TRUE(store_client->wait_for_service(kServiceWaitTime));

  auto store_request = std::make_shared<StoreJointState::Request>();
  store_request->joint_state.name = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };
  store_request->joint_state.position = { 0, -0.78, -0.51, -1.57, 1.65, 0 };

  auto store_result_future = store_client->async_send_request(store_request);
  ASSERT_EQ(store_result_future.wait_for(kServiceWaitTime), std::future_status::ready)
      << "Timed out waiting for store joint state service response.";
  ASSERT_TRUE(store_result_future.get()->success);

  // Run the Objective
  auto do_objective_goal = std::make_unique<DoObjectiveSequence::Goal>();
  do_objective_goal->objective_name = "Move to Joint State";

  EXPECT_TRUE(sendDoObjectiveSequenceGoal(std::move(do_objective_goal), 30.0));
}

TEST_F(ObjectiveFixture, TestInterpolateToJointState)
{
  ASSERT_TRUE(setupDoObjectiveSequenceClient());

  // Store a joint state parameter
  rclcpp::Client<StoreJointState>::SharedPtr store_client = node_->create_client<StoreJointState>("store_joint_state");
  ASSERT_TRUE(store_client->wait_for_service(kServiceWaitTime));

  auto store_request = std::make_shared<StoreJointState::Request>();
  store_request->joint_state.name = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };
  store_request->joint_state.position = { 0, -0.78, -0.51, -1.57, 1.65, 0 };

  auto store_result_future = store_client->async_send_request(store_request);
  ASSERT_EQ(store_result_future.wait_for(kServiceWaitTime), std::future_status::ready)
      << "Timed out waiting for store joint state service response.";
  ASSERT_TRUE(store_result_future.get()->success);

  // Run the Objective
  auto do_objective_goal = std::make_unique<DoObjectiveSequence::Goal>();
  do_objective_goal->objective_name = "Interpolate to Joint State";

  EXPECT_TRUE(sendDoObjectiveSequenceGoal(std::move(do_objective_goal), 30.0));
}

// TODO: Enable once admittance control works in the current simulation environment, or we can override behaviors for sim configs.
TEST_F(ObjectiveFixture, DISABLED_TestOpenDoorAffordance)
{
  ASSERT_TRUE(setupDoObjectiveSequenceClient());

  auto do_objective_goal = std::make_unique<DoObjectiveSequence::Goal>();
  do_objective_goal->objective_name = "Open Cabinet Door";

  // Update the grasp pose parameter used when planning the objective
  BehaviorParameter pose_name_parameter;
  pose_name_parameter.behavior_namespaces.emplace_back("process_door_selection");
  pose_name_parameter.description.name = "grasp_pose";
  pose_name_parameter.description.type = BehaviorParameterDescription::TYPE_POSE;
  pose_name_parameter.pose_value = [] {
    PoseStamped msg;
    msg.header.frame_id = "world";
    msg.pose.position = geometry_msgs::build<Point>().x(0.529).y(0.460).z(0.858);
    msg.pose.orientation = geometry_msgs::build<Quaternion>().x(0.5).y(0.5).z(0.5).w(0.5);
    return msg;
  }();

  BehaviorParameter hinge_axis_pose_start_parameter;
  hinge_axis_pose_start_parameter.behavior_namespaces.emplace_back("process_door_selection");
  hinge_axis_pose_start_parameter.description.name = "hinge_axis_pose_start";
  hinge_axis_pose_start_parameter.description.type = BehaviorParameterDescription::TYPE_POSE;
  hinge_axis_pose_start_parameter.pose_value = [] {
    PoseStamped msg;
    msg.header.frame_id = "world";
    msg.pose.position = geometry_msgs::build<Point>().x(0.55).y(0.350).z(0.765);
    msg.pose.orientation = geometry_msgs::build<Quaternion>().x(0.5).y(0.5).z(0.5).w(0.5);
    return msg;
  }();

  BehaviorParameter hinge_axis_pose_end_parameter;
  hinge_axis_pose_end_parameter.behavior_namespaces.emplace_back("process_door_selection");
  hinge_axis_pose_end_parameter.description.name = "hinge_axis_pose_end";
  hinge_axis_pose_end_parameter.description.type = BehaviorParameterDescription::TYPE_POSE;
  hinge_axis_pose_end_parameter.pose_value = [] {
    PoseStamped msg;
    msg.header.frame_id = "world";
    msg.pose.position = geometry_msgs::build<Point>().x(0.55).y(0.350).z(0.767);
    msg.pose.orientation = geometry_msgs::build<Quaternion>().x(0.5).y(0.5).z(0.5).w(0.5);
    return msg;
  }();

  do_objective_goal->parameter_overrides = { pose_name_parameter, hinge_axis_pose_start_parameter,
                                             hinge_axis_pose_end_parameter };

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
