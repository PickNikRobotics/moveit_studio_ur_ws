#include <translate_pose/translate_pose.hpp>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_studio_behavior_interface/test_behavior.hpp>
#include <moveit_studio_common/test_utils/ros_test.hpp>

namespace translate_pose
{

BEGIN_BEHAVIOR_PORT_SETTER_MAP(myExpectedInputPorts)
DEFINE_BEHAVIOR_PORT_SETTER("pose_stamped", geometry_msgs::msg::PoseStamped())
DEFINE_BEHAVIOR_PORT_SETTER("x", 0.0)
DEFINE_BEHAVIOR_PORT_SETTER("y", 0.0)
DEFINE_BEHAVIOR_PORT_SETTER("z", 0.0)
END_BEHAVIOR_PORT_SETTER_MAP()

BEGIN_BEHAVIOR_PORT_SETTER_MAP(myExpectedOutputPorts)
DEFINE_BEHAVIOR_PORT_SETTER("new_pose", geometry_msgs::msg::PoseStamped())
END_BEHAVIOR_PORT_SETTER_MAP()

class TranslatePoseTest : public moveit_studio::test_utils::RosTest, 
                           public ::moveit_studio::test_utils::WithBehavior<translate_pose::TranslatePose>
{
  void SetUp() override
  {
    initBehavior(/*name=*/"TranslatePose", myExpectedInputPorts, myExpectedOutputPorts);
  }
};

TEST_F(TranslatePoseTest, TranslatePose) {
  // Set input ports.
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.pose.position.x = 1.0;
  pose_stamped.pose.position.y = 2.0;
  pose_stamped.pose.position.z = 3.0;

  blackboard().set("pose_stamped", pose_stamped);
  blackboard().set("x", 1.0);
  blackboard().set("y", 2.0);
  blackboard().set("z", 3.0);

  EXPECT_CALL(mockLogger(), publishWarnMessage(testing::_, testing::HasSubstr("Called tick"))).Times(1);
  EXPECT_CALL(mockLogger(), publishWarnMessage(testing::_, testing::HasSubstr("Should translate"))).Times(1);

  // Run behavior.
  EXPECT_EQ(behavior().tick(), BT::NodeStatus::SUCCESS);

  // Check output ports.
  const geometry_msgs::msg::PoseStamped new_pose = blackboard().get<geometry_msgs::msg::PoseStamped>("new_pose");
  EXPECT_EQ(new_pose.pose.position.x, 2.0);
  EXPECT_EQ(new_pose.pose.position.y, 4.0);
  EXPECT_EQ(new_pose.pose.position.z, 6.0);
}

}  // namespace translate_pose
