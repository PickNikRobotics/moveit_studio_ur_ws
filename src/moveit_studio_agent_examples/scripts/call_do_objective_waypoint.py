#!/usr/bin/env python3

# Copyright (c) 2023 PickNik Inc.
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.

# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.

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


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import sys

from moveit_studio_agent_msgs.action import DoObjectiveSequence
from moveit_studio_behavior_msgs.msg import (
    BehaviorParameter,
    BehaviorParameterDescription,
)


class DoObjectiveSequenceClient(Node):
    """
    ROS 2 node that acts as an Action Client for the MoveIt Studio Agent's Objective Server
    """

    def __init__(self):
        super().__init__("DoObjectiveSequence")
        self._action_client = ActionClient(self, DoObjectiveSequence, "do_objective")

    def send_goal(self, waypoint_name="Behind"):
        """
        Sends a DoObjectiveSequence Goal for "Move to Joint State" to the Objective Server via the node's Action Client.

        Args:
            waypoint_name: the (string) name of a waypoint to move to.

        Returns:
            goal_future: a rclpy.task.Future to a rclpy.action.client.ClientGoalHandle.
        """
        goal_msg = DoObjectiveSequence.Goal()
        goal_msg.objective_name = "Move to Joint State"

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
            return

        self.get_logger().info("Goal accepted...")

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info(f"Objective succeeded!")
        elif hasattr(result.error_code, "error_message"):
            self.get_logger().info(
                f"Objective failed: {result.error_code.error_message}"
            )
        else:
            self.get_logger().info(
                f"Objective failed. MoveItErrorCode Value: {result.error_code.val}"
            )

        rclpy.shutdown()


def main(args=None):
    if len(sys.argv) < 2:
        print(
            "usage: ros2 run moveit_studio_agent_examples call_do_objective_waypoint.py 'Waypoint Name'"
        )
    else:
        rclpy.init(args=args)

        client = DoObjectiveSequenceClient()

        waypoint_name = sys.argv[1]
        client.send_goal(waypoint_name)

        rclpy.spin(client)


if __name__ == "__main__":
    main()
