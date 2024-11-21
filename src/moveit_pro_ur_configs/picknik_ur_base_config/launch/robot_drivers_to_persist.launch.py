# Copyright 2023 PickNik Inc.
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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import AnyLaunchDescriptionSource

from moveit_studio_utils_py.launch_common import empty_gen
from moveit_studio_utils_py.system_config import (
    SystemConfigParser,
)


def generate_launch_description():
    system_config_parser = SystemConfigParser()
    hardware_config = system_config_parser.get_hardware_config()
    controller_config = system_config_parser.get_ros2_control_config()

    declare_robot_ip = DeclareLaunchArgument(
        "robot_ip", description="IP address of the robot"
    )
    robot_ip = LaunchConfiguration("robot_ip")

    dashboard_client_node = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client",
        output="both",
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip}],
    )

    protective_stop_manager_node = Node(
        package="moveit_studio_ur_pstop_manager",
        executable="protective_stop_manager_node",
        name="protective_stop_manager_node",
        output="both",
        parameters=[
            {
                "controllers_default_active": controller_config.get(
                    "controllers_active_at_startup", empty_gen()
                ),
                "controllers_default_not_active": controller_config.get(
                    "controllers_inactive_at_startup", empty_gen()
                ),
            }
        ],
    )

    tool_comms_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([ThisLaunchFileDir(), "/ur_tool_comms.launch.xml"]),
        launch_arguments={
            "robot_ip": robot_ip,
            "tool_tcp_port": "54321",
            "tool_device_name": "/tmp/ttyUR",
        }.items(),
    )

    nodes_to_launch = [
        dashboard_client_node,
        protective_stop_manager_node,
        tool_comms_launch,
    ]

    return LaunchDescription(nodes_to_launch)
