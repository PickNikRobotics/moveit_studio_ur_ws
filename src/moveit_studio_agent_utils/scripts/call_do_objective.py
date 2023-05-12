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


class DoObjectiveSequenceClient(Node):
    """
    ROS 2 node that acts as an Action Client for the MoveIt Studio Agent's Objective Server
    """

    def __init__(self):
        super().__init__("DoObjectiveSequence")
        self._action_client = ActionClient(self, DoObjectiveSequence, "do_objective")

    def send_goal(self, objective_name):
        """
        Sends a DoObjectiveSequence Goal to the Objective Server via the node's Action Client.

        Args:
            objective_name: the (string) name of an objective to run.

        Returns:
            result: a BT::NodeStatus result. Can be Success or Failure.
        """
        goal_msg = DoObjectiveSequence.Goal()
        goal_msg.objective_name = objective_name
        self._action_client.wait_for_server()
        result = self._action_client.send_goal_async(goal_msg)
        return result


def main(args=None):
    if len(sys.argv) < 2:
        print(
            "usage: ros2 run moveit_studio_agent_utils call_do_objective.py 'Objective Name'"
        )
    else:
        rclpy.init(args=args)

        client = DoObjectiveSequenceClient()

        future = client.send_goal(sys.argv[1])

        rclpy.spin_until_future_complete(client, future)


if __name__ == "__main__":
    main()
