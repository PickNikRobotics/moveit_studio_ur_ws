#!/usr/bin/env python3

# Copyright 2023 Picknik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Picknik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


# TODO:
# Use the Action Server's CancelGoal service: https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/srv/CancelGoal.srv
#     which will let us cancel a goal with just a Goal ID 
# Action Server publishes "GoalInfo" messages with the list of IDs for current active Goals.

import argparse
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.msg import MoveItErrorCodes
from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal


class CancelGoalClient(Node):
    """
    ROS 2 node that acts as an Action Client for MoveIt Studio's Objective Server.
    Calls CancelGoal action with an optional Goal ID.
    """

    def __init__(self):
        super().__init__("CancelGoalNode")
        self._action_client = ActionClient(self, CancelGoal, "cancel_goal")

    def cancel_goal(self, goal_id=0, timestamp=0):
        """
        Cancels an Objective Server's DoObjectiveSequence Goal via the node's Action Client.

        Returns:
            future: a rclpy.task.Future that completes when the goal is canceled.
        """
        self.get_logger().info("Attempting to cancel goal.")
        cancel_msg = CancelGoal.Request()
        cancel_msg.goal_info = GoalInfo()
        cancel_msg.goal_info.goal_id = goal_id
        cancel_msg.goal_info.stamp = timestamp
        print("Cancel Msg:", cancel_msg)
        future = self._action_client.call_async(cancel_msg)
        #future.add_done_callback(self.cancel_goal_callback)
        # Cancel the timer that this was a part of.
        return future

    def cancel_goal_callback(self, future):
        cancel_response = future.result()
        if cancel_response.goals_canceling:
            self.get_logger().info("Goal successfully canceled.")
        else:
            self.get_logger().info("Goal failed to cancel.")

        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    #parser.add_argument("objective_name", type=str, help="Name of Objective to cancel.")
    args = parser.parse_args()

    rclpy.init()

    client = CancelGoalClient()

    future = client.cancel_goal()

    #rclpy.spin(client)
    rclpy.spin_until_future_complete(client, future)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
