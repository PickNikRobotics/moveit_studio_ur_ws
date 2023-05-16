#!/usr/bin/env python3

# BSD 3-Clause License
# Copyright (c) 2023, PickNik Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import argparse
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.msg import MoveItErrorCodes
from moveit_studio_agent_msgs.action import DoObjectiveSequence
from moveit_studio_behavior_msgs.msg import (
    BehaviorParameter,
    BehaviorParameterDescription,
)


class DoObjectiveSequenceClient(Node):
    """
    ROS 2 node that acts as an Action Client for MoveIt Studio's Objective Server.
    Configured for the "Move to Joint State" waypoint, and adds a parameter override for waypoint names.
    """

    def __init__(self):
        super().__init__("DoObjectiveSequence")
        self._action_client = ActionClient(self, DoObjectiveSequence, "do_objective")

    def send_goal(self, waypoint_name, cancel):
        """
        Sends a DoObjectiveSequence Goal to the Objective Server via the node's Action Client.

        Args:
            waypoint_name: the (string) name of a waypoint to move to.

        Returns:
            goal_future: a rclpy.task.Future to a rclpy.action.client.ClientGoalHandle.
        """
        goal_msg = DoObjectiveSequence.Goal()
        goal_msg.objective_name = "Move to Joint State"
        self.cancel = cancel

        behavior_parameter = BehaviorParameter()
        behavior_parameter.behavior_namespaces.append("move_to_joint_state")
        behavior_parameter.description.name = "waypoint_name"
        behavior_parameter.description.type = BehaviorParameterDescription.TYPE_STRING
        behavior_parameter.string_value = waypoint_name
        goal_msg.parameter_overrides = [behavior_parameter]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return self._send_goal_future

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")

            rclpy.shutdown()
            return

        self._goal_handle = goal_handle
        self.get_logger().info("Goal accepted...")

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
        # Cancel the goal after a set amount of time (in seconds)
        if self.cancel:
            self._timer = self.create_timer(2.0, self.cancel_goal)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("Objective succeeded!")
        elif hasattr(result.error_code, "error_message"):
            self.get_logger().info(
                f"Objective failed: {result.error_code.error_message}"
            )
        else:
            self.get_logger().info(
                f"Objective failed. MoveItErrorCode Value: {result.error_code.val}"
            )

        rclpy.shutdown()

    def cancel_goal(self):
        """
        Cancels an Objective Server's DoObjectiveSequence Goal via the node's Action Client.

        Returns:
            future: a rclpy.task.Future that completes when the goal is canceled.
        """
        self.get_logger().info("Attempting to cancel goal.")
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_goal_callback)
        # Cancel the timer that this was a part of.
        self._timer.cancel()
        return future

    def cancel_goal_callback(self, future):
        cancel_response = future.result()
        if cancel_response.goals_canceling:
            self.get_logger().info("Goal successfully canceled.")
        else:
            self.get_logger().info("Goal failed to cancel.")

        rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("waypoint_name", type=str, help="Name of waypoint to move to.")
    parser.add_argument(
        "--cancel",
        action="store_true",
        help="Optional boolean for if the objective should be automatically cancelled after a set amount of time.",
    )
    args = parser.parse_args()

    rclpy.init()

    client = DoObjectiveSequenceClient()

    client.send_goal(args.waypoint_name, args.cancel)

    rclpy.spin(client)


if __name__ == "__main__":
    main()
