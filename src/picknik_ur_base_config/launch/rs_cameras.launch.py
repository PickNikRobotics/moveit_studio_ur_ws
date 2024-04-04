# Copyright 2021 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

from moveit_studio_utils_py.launch_common import get_launch_file, get_ros_path
from moveit_studio_utils_py.system_config import SystemConfigParser


def check_realsense_serial_numbers(camera_config):
    """
    Performs checks on the Realsense camera serial numbers to warn if an error relating to the serial number might occur
    :param camera_config: The camera configuration
    """
    for camera in camera_config:
        camera = camera[next(iter(camera))]
        # There is an issue with the YAML parsing where if the serial number starts with 0, it is read in as an octal number
        # instead of a string. This also only seems to happen if the string contains 8 or 9. If the conditions are met, print out a warning
        if serial_no := camera.get("serial_no"):
            if serial_no[0] == "0" and ("8" in serial_no or "9" in serial_no):
                print(
                    "Warning: The RealSense camera serial number starts with a 0 and contains an 8 or 9, which may cause an issue with the YAML parser for the RealSense launch file. "
                    + "If the following error is seen: "
                    + "An exception has been thrown: /opt/underlay_ws/src/realsense-ros/realsense2_camera/src/realsense_node_factory.cpp:340:parameter "
                    + "'serial_no' has invalid type: Wrong parameter type, parameter {serial_no} is of type {string}, setting it to {double} is not allowed. "
                    + "There is no current workaround other than using a RealSense camera whose serial number does not start with a 0 and contain an 8 or 9."
                )
        else:
            print(
                f"Error: Configuration for camera '{camera.get('camera_name')}' is missing a 'serial_no' entry"
            )


def get_cameras_config(system_config):
    """
    Extracts the camera config from a Pro system config
    :param system: Full MoveIt Pro system config
    """
    if system_config.get("hardware") is None:
        return []
    else:
        check_realsense_serial_numbers(system_config["hardware"].get("cameras", []))
        return system_config["hardware"].get("cameras", [])


def generate_launch_description():
    """
    This is a generic launch file for cameras. At the moment we support:
    realsense D415 cameras only
    This is the yaml config example that goes with these cameras:
      cameras:
        - wrist_mounted:
            camera_name: "wrist_mounted"
            type: "realsense"
            use: True
            serial_no: "'0'"
            device_type: "D415"
            framerate: 6
            image_width: 640
            image_height: 480
            enable_pointcloud: "true"
        - scene_camera:
            camera_name: "scene_camera"
            type: "realsense"
            use: True
            serial_no: "'0'"
            device_type: "D415"
            framerate: 6
            image_width: 640
            image_height: 480
            enable_pointcloud: "true"
    """
    system_config_parser = SystemConfigParser()
    cameras_config = get_cameras_config(system_config_parser.get_system_config())

    nodes_to_launch = []
    included_launch_files = []

    if not cameras_config:
        print("No camera configuration found. Not launching any camera nodes.")

    for camera in cameras_config:
        # The camera is a single dict with data embedded inside the name
        camera = camera[next(iter(camera))]

        # Launch camera type specific nodes
        if not isinstance(camera["serial_no"], str):
            print(
                "Error: Camera serial number is required to be a string in override.yaml\n"
                + "Please add quotes around the serial number"
            )
            exit()
        print("Adding Realsense: " + camera["camera_name"])
        # The Realsense Node wants the a string instead of bool for "enable_pointcloud"
        enable_pointcloud = "false"
        if camera["enable_pointcloud"]:
            enable_pointcloud = "true"

        # The Realsense Node wants the a string instead of bool for "decimation_filter.enable"
        # The value of "enable_decimation_filter" is a bool
        enable_decimation_filter = (
            "true" if camera.get("enable_decimation_filter") else "false"
        )

        # Load optional JSON config file for RealSense camera
        # If the file cannot be loaded, an empty string will be set for the filepath, which will cause the driver to load its default config values
        json_file_path = camera.get("json_file_path")
        json_path = ""

        if json_file_path:
            package_name = json_file_path.get("package")
            relative_path = json_file_path.get("path")
            if not package_name or not relative_path:
                print(
                    "Warning: Could not find json_file_path.package and json_file_path.path parameters from camera config file. Loading default camera settings."
                )
            else:
                json_path = get_ros_path(package_name, relative_path)
                print("Found camera setting JSON at : ", json_path)

        included_launch_files.append(
            IncludeLaunchDescription(
                get_launch_file("realsense2_camera", "launch/rs_launch.py"),
                launch_arguments={
                    "serial_no": "'" + str(camera["serial_no"]) + "'",
                    "pointcloud.enable": enable_pointcloud,
                    "device_type": camera["device_type"],
                    "camera_name": camera["camera_name"],
                    "base_frame_id": camera["camera_name"] + "_link",
                    # All 3 params (fps/width/height) must be set
                    "depth_module.profile": f"{camera['image_width']}x{camera['image_height']}x{camera['framerate']}",
                    "rgb_camera.profile": f"{camera['image_width']}x{camera['image_height']}x{camera['framerate']}",
                    "pointcloud.ordered_pc": "true",
                    "json_file_path": json_path,
                    "decimation_filter.enable": enable_decimation_filter,
                }.items(),
            )
        )

    return LaunchDescription(nodes_to_launch + included_launch_files)
