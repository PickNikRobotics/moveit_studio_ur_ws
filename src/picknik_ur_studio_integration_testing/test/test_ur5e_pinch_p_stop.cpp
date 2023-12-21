// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <moveit_studio_agent/test/do_objective_test_fixture.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <moveit_studio_agent_msgs/srv/set_transform.hpp>
#include <moveit_studio_sdk_msgs/msg/behavior_parameter.hpp>

namespace
{
using BehaviorParameter = moveit_studio_sdk_msgs::msg::BehaviorParameter;
using BehaviorParameterDescription = moveit_studio_sdk_msgs::msg::BehaviorParameterDescription;
}  // namespace

namespace moveit_studio::agent::testing
{
using Point = geometry_msgs::msg::Point;
using PoseStamped = geometry_msgs::msg::PoseStamped;
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

  auto do_objective_goal = std::make_unique<DoObjectiveSequence::Goal>();
  do_objective_goal->objective_name = "Interpolate to Joint State";

  // Update the joint_state parameter used when planning the objective
  BehaviorParameter pose_name_parameter;
  pose_name_parameter.behavior_namespaces = { "interpolate_to_joint_state" };
  pose_name_parameter.description.name = "target_joint_state";
  pose_name_parameter.description.type = BehaviorParameterDescription::TYPE_JOINT_STATE;
  pose_name_parameter.joint_state_value.name = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                                 "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };
  pose_name_parameter.joint_state_value.position = { -0.66, -2.10, -1.27, 0.60, 2.87, 0.36 };

  do_objective_goal->parameter_overrides = { pose_name_parameter };

  // Expect to fail due to pinch links blocking entering a pinch stop zone
  EXPECT_FALSE(sendDoObjectiveSequenceGoal(std::move(do_objective_goal), 30.0));
}

TEST_F(ObjectiveFixture, InterpolateToNearPinchJointState)
{
  ASSERT_TRUE(setupDoObjectiveSequenceClient());

  auto do_objective_goal = std::make_unique<DoObjectiveSequence::Goal>();
  do_objective_goal->objective_name = "Interpolate to Joint State";

  // Update the joint_state parameter used when planning the objective
  BehaviorParameter pose_name_parameter;
  pose_name_parameter.behavior_namespaces = { "interpolate_to_joint_state" };
  pose_name_parameter.description.name = "target_joint_state";
  pose_name_parameter.description.type = BehaviorParameterDescription::TYPE_JOINT_STATE;
  pose_name_parameter.joint_state_value.name = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                                 "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };
  // Now we put wrist_2_joint around 10 degrees more relaxed from the pinch point.
  pose_name_parameter.joint_state_value.position = { -0.66, -2.10, -1.27, 0.60, 2.70, 0.36 };

  do_objective_goal->parameter_overrides = { pose_name_parameter };

  // Expect the pinch link geometries to be unobtrusive enough that planning to just outside the
  // pinch zone still works.
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
