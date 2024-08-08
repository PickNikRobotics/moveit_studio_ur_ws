import sys

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction

# Add the realsense2_camera/launch directory to the user's path so we can import the realsense2_camera rs_launch module.
realsense2_camera_path = get_package_share_directory("realsense2_camera")
sys.path.append(f"{realsense2_camera_path}/launch")

import rs_launch


rs_launch.configurable_parameters.extend(
    [
        {
            "name": "color_qos",
            "default": "SENSOR_DATA",
            "description": "Color stream QoS settings",
        },
        {
            "name": "depth_qos",
            "default": "SENSOR_DATA",
            "description": "Depth stream QoS settings",
        },
    ]
)


def generate_launch_description():
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(rs_launch.configurable_parameters)
        + [OpaqueFunction(function=rs_launch.launch_setup)]
    )
