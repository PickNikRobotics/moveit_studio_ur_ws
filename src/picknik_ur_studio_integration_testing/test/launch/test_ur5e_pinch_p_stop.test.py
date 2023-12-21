#!/usr/bin/env python3

# Copyright 2023 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


import os
import sys
import unittest

import launch_testing

from picknik_ur_studio_integration_testing.generate_agent_plus_drivers_launch_description import (
    generate_agent_plus_drivers_launch_description,
)


def generate_test_description():
    return generate_agent_plus_drivers_launch_description(
        gtest_name="test_ur5e_pinch_p_stop",
        env_vars={
            # Load site config for simulated UR-5e.
            # Note: we want a configuration without the tool_changer_link for this one as it will throw off the collision check.
            "STUDIO_CONFIG_PACKAGE": "picknik_001_ur5e_config",
            "MOCK_HARDWARE": "true",
        },
    )


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, objective_client_gtest):
        self.proc_info.assertWaitForShutdown(objective_client_gtest, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes (successful codes)
    def test_gtest_pass(self, proc_info, objective_client_gtest):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=objective_client_gtest
        )
