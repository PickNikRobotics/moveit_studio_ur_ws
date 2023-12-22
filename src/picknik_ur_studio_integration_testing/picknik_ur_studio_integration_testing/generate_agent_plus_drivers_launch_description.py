#!/usr/bin/env python3

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


from datetime import datetime
import os
from pathlib import Path
from typing import Dict

from ament_index_python import get_package_share_directory
import launch_ros
import launch_testing
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare

from moveit_studio_utils_py.system_config import (
    SystemConfigParser,
)


def generate_agent_plus_drivers_launch_description(
    *args,
    gtest_name: SomeSubstitutionsType,
    env_vars: Dict,
):
    """
    Creates a launch description for a comprehensive integration test fixture that runs the full set of Agent nodes
    :param args: Additional arguments passed to the launch description generator function
    :param gtest_name: Name of the GTest executable to run in this integration test
    :param env_vars: A set of environment variables to apply to the environment when running the test scenario
    :return: A LaunchDescription for the integration test scenario
    """
    # Update environment variables from provided values
    os.environ.update(env_vars)

    # Generate the robot system config before running all tests.
    system_config_parser = SystemConfigParser()

    # Get path to test objective directory defined within this repo
    test_objective_path = Path(
        get_package_share_directory("moveit_studio_integration_testing"),
        "test",
        "objectives",
    )

    # Get timestamp at which the LaunchDescription was parsed (used when naming test output files)
    timestamp_string = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

    # Get the value of the STUDIO_TEST_ROSBAG_OUTPUT_DIR environment variable.
    # If this variable is not set, return an invalid path that will not resolve.
    rosbag_output_path = os.environ.get(
        "STUDIO_TEST_ROSBAG_OUTPUT_DIR", default="/intentionally/invalid/path"
    )

    # Store "true" is the parent directory of the provided directory path is a valid and extant directory.
    # Store "false" if it is not valid and extant.
    is_rosbag_output_path_valid = (
        "true"
        if os.path.isdir(os.path.abspath(os.path.join(rosbag_output_path, os.pardir)))
        else "false"
    )

    # Create a launch substitution condition to trigger recording rosbag files during the test.
    condition_save_rosbag = IfCondition(
        TextSubstitution(text=is_rosbag_output_path_valid)
    )

    # Get path to test objective directory defined within this repo
    test_objective_path = Path(
        get_package_share_directory("moveit_studio_integration_testing"),
        "test",
        "objectives",
    )

    # The agent looks for objective XML files in the user folder
    # <HOME>/.config/moveit_studio/objectives
    # This folder is created by the REST API which is executed before the agent.
    # While testing the agent alone, this folder does not exist so we need to
    # point the agent to an alternate set of locations defined by test_objective_path.
    # Additionally, we must convert this list of locations to a comma separated list
    # because standard lists cannot be used as launch arguments.
    objective_library_directories = ",".join(
        system_config_parser.get_objective_library_paths() + [str(test_objective_path)]
    )

    # Get launch description for Studio Agent nodes
    agent_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("moveit_studio_agent"),
                    "launch",
                    "studio_agent.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "is_test": "true",
            "objective_library_directories": objective_library_directories,
        }.items(),
    )

    # Get launch description for Studio robot driver nodes
    robot_driver_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("moveit_studio_agent"),
                    "launch",
                    "robot_drivers.launch.py",
                ]
            )
        )
    )

    # Add test executable
    objective_client_gtest = launch_ros.actions.Node(
        executable=PathJoinSubstitution(
            [LaunchConfiguration("test_binary_dir"), gtest_name]
        ),
        output="both",
    )

    # Create the directory in which rosbag output will be saved.
    make_rosbag_directory = ExecuteProcess(
        cmd=[
            "mkdir",
            "-p",
            TextSubstitution(text=rosbag_output_path),
        ],
        output="both",
        condition=condition_save_rosbag,
    )

    # Record a rosbag file containing MoveIt planning scene updates.
    rosbag_record = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--output",
            PathJoinSubstitution(
                [
                    TextSubstitution(text=rosbag_output_path),
                    gtest_name + "_" + timestamp_string,
                ]
            ),
            "/monitored_planning_scene",
        ],
        output="both",
        condition=condition_save_rosbag,
    )

    gtest_action = objective_client_gtest

    return LaunchDescription(
        [
            # Increase timeout for SIGINT when shutting down the launch processes
            # to give the Gazebo server more time to cleanly exit
            DeclareLaunchArgument(
                "sigterm_timeout",
                default_value="15",
            ),
            DeclareLaunchArgument(
                name="test_binary_dir",
                description="Binary directory of package "
                "containing test executables",
            ),
            agent_launch_description,
            robot_driver_launch_description,
            make_rosbag_directory,
            TimerAction(period=9.0, actions=[rosbag_record]),
            TimerAction(
                period=10.0,
                actions=[gtest_action],
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        "objective_client_gtest": objective_client_gtest,
    }
