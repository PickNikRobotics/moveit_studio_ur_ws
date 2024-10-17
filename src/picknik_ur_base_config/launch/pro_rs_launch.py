# Copyright 2024 PickNik Inc.
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
#    * Neither the name of the PickNik Inc. nor the names of its
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

import sys

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction

# Add the realsense2_camera/launch directory to the user's path so we can import the realsense2_camera rs_launch module.
realsense2_camera_path = get_package_share_directory("realsense2_camera")
sys.path.append(f"{realsense2_camera_path}/launch")

import rs_launch


def generate_launch_description():
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
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(rs_launch.configurable_parameters)
        + [
            OpaqueFunction(
                function=rs_launch.launch_setup,
                kwargs={
                    "params": rs_launch.set_configurable_parameters(
                        rs_launch.configurable_parameters
                    )
                },
            )
        ]
    )
