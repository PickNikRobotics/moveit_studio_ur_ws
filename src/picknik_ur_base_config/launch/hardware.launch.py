# Copyright 2023 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_studio_utils_py.system_config import SystemConfigParser

from moveit_studio_utils_py.generate_camera_frames import (
    generate_camera_frames,
)


system_config_parser = SystemConfigParser()
cameras_config = system_config_parser.get_cameras_config()


def generate_launch_description():
    nodes_to_start = []
    included_launch_files = []

    frame_pair_params = [
        {
            "world_frame": "world",
            "camera_frames": generate_camera_frames(cameras_config),
        }
    ]

    nodes_to_start.append(
        Node(
            package="moveit_studio_agent",
            executable="camera_transforms_node",
            name="camera_transforms_node",
            output="screen",
            parameters=frame_pair_params,
        )
    )

    return LaunchDescription(nodes_to_start + included_launch_files)
