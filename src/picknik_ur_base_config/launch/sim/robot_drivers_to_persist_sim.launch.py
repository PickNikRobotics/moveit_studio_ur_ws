# Copyright 2023 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Mock the UR Dashboard Client
    mock_dashboard_client = Node(
        package="moveit_studio_ur_pstop_manager",
        executable="mock_ur_dashboard_client_node",
        name="dashboard_client",
        output="screen",
    )

    return LaunchDescription([mock_dashboard_client])
