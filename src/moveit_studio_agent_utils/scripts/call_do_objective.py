#!/usr/bin/env python3

# Copyright 2022 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import sys

from moveit_studio_agent_msgs.action import DoObjectiveSequence


class DoObjectiveSequenceClient(Node):
    def __init__(self):
        super().__init__("DoObjectiveSequence")
        self._action_client = ActionClient(self, DoObjectiveSequence, "do_objective")

    def send_goal(self, objective_name):
        goal_msg = DoObjectiveSequence.Goal()
        goal_msg.objective_name = objective_name
        self._action_client.wait_for_server()
        result = self._action_client.send_goal_async(goal_msg)
        return result


def main(args=None):
    if len(sys.argv) < 2:
        print(
            "usage: ros2 run moveit_studio_behavior call_do_objective.py 'Objective Name'"
        )
    else:
        rclpy.init(args=args)

        client = DoObjectiveSequenceClient()

        future = client.send_goal(sys.argv[1])

        rclpy.spin_until_future_complete(client, future)


if __name__ == "__main__":
    main()
