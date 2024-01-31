#!/usr/bin/env python3

# Copyright 2023 PickNik Inc.

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

# This launch file corresponds to the tests in `test_reset_ur_pstop.cpp`

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch
import launch_testing
import os
import sys
import unittest

sys.path.append(os.path.dirname(__file__))


def generate_test_description():

    protective_stop_manager_node = Node(
        package="moveit_studio_ur_pstop_manager",
        executable="protective_stop_manager_node",
        name="protective_stop_manager_node",
        output="both",
    )

    # Mock the UR Dashboard Client
    mock_dashboard_client = Node(
        package="moveit_studio_ur_pstop_manager",
        executable="mock_ur_dashboard_client_node",
        name="dashboard_client",
        output="both",
    )

    reset_ur_pstop_gtest = Node(
        executable=PathJoinSubstitution(
            [LaunchConfiguration("test_binary_dir"), "test_reset_ur_pstop"]
        ),
        output="both",
    )

    # Launch controller manager and io_and_status_controller
    robot_driver_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ur_robot_driver"),
                "launch/ur_control.launch.py",
            ),
        ),
        launch_arguments={
            "ur_type": "ur5",
            "robot_ip": "yyy.yyy.yyy.yyy",
            "use_fake_hardware": "true",
            "launch_rviz": "false",
        }.items(),
    )

    return launch.LaunchDescription(
        [
            mock_dashboard_client,
            protective_stop_manager_node,
            robot_driver_launch_description,
            DeclareLaunchArgument(
                name="test_binary_dir",
                description="Binary directory of package "
                "containing test executables",
            ),
            TimerAction(period=1.0, actions=[reset_ur_pstop_gtest]),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        "reset_ur_pstop_gtest": reset_ur_pstop_gtest,
    }


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, reset_ur_pstop_gtest):
        self.proc_info.assertWaitForShutdown(reset_ur_pstop_gtest, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes (successful codes)
    def test_gtest_pass(self, proc_info, reset_ur_pstop_gtest):
        launch_testing.asserts.assertExitCodes(proc_info, process=reset_ur_pstop_gtest)
