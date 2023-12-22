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

TEST_F(ObjectiveFixture, TestMoveToNamedState)
{
  ASSERT_TRUE(setupDoObjectiveSequenceClient());

  auto do_objective_goal = std::make_unique<DoObjectiveSequence::Goal>();
  do_objective_goal->objective_name = "Move to Known Pose";

  BehaviorParameter pose_name_parameter;
  pose_name_parameter.behavior_namespaces.emplace_back("move_to_known_pose");
  pose_name_parameter.description.name = "target_state_name";
  pose_name_parameter.description.type = BehaviorParameterDescription::TYPE_STRING;
  pose_name_parameter.string_value = "Home";

  do_objective_goal->parameter_overrides.push_back(pose_name_parameter);

  EXPECT_TRUE(sendDoObjectiveSequenceGoal(std::move(do_objective_goal)));
}

TEST_F(ObjectiveFixture, TestMoveToWaypoint)
{
  ASSERT_TRUE(setupDoObjectiveSequenceClient());

  auto do_objective_goal = std::make_unique<DoObjectiveSequence::Goal>();
  do_objective_goal->objective_name = "Move to Waypoint";

  BehaviorParameter waypoint_name_parameter;
  waypoint_name_parameter.behavior_namespaces.emplace_back("move_to_waypoint");
  waypoint_name_parameter.description.name = "waypoint_name";
  waypoint_name_parameter.description.type = BehaviorParameterDescription::TYPE_STRING;
  waypoint_name_parameter.string_value = "Forward Down";

  do_objective_goal->parameter_overrides.push_back(waypoint_name_parameter);

  EXPECT_TRUE(sendDoObjectiveSequenceGoal(std::move(do_objective_goal)));
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

TEST_F(ObjectiveFixture, TestInterpolateToJointState)
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
  pose_name_parameter.joint_state_value.position = { 0, -0.78, -0.51, -1.57, 1.65, 0 };

  do_objective_goal->parameter_overrides = { pose_name_parameter };

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

  BehaviorParameter hing_axis_pose_start_parameter;
  hing_axis_pose_start_parameter.behavior_namespaces.emplace_back("process_door_selection");
  hing_axis_pose_start_parameter.description.name = "hinge_axis_pose_start";
  hing_axis_pose_start_parameter.description.type = BehaviorParameterDescription::TYPE_POSE;
  hing_axis_pose_start_parameter.pose_value = [] {
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

  do_objective_goal->parameter_overrides = { pose_name_parameter, hing_axis_pose_start_parameter,
                                             hinge_axis_pose_end_parameter };

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
