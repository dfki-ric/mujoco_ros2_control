#!/bin/env python3
"""Launch smoke test for the franka example (headless when DISABLE_OPENGL=1).

See example_smoke.py for what is exercised and when the test self-skips.
"""

import os
import sys
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
import rclpy.node

# example_smoke.py lives next to this test; make sure it is importable regardless
# of how the launch_test runner sets up sys.path.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from example_smoke import availability, make_test_description  # noqa: E402

ROBOT = "franka"
_AVAILABLE, _SKIP_REASON = availability(ROBOT)


@pytest.mark.launch_test
def generate_test_description():
    if not _AVAILABLE:
        # Nothing to launch; the single test below is skipped.
        return launch.LaunchDescription([launch_testing.actions.ReadyToTest()])
    return make_test_description(ROBOT)


class TestFrankaExample(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.node.Node("test_franka_example_node")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @unittest.skipUnless(_AVAILABLE, _SKIP_REASON)
    def test_node_comes_up(self):
        # The mujoco node only registers once it has loaded the merged MJCF; if
        # MuJoCo rejected the model (or the xacro failed) it FATALs and never appears.
        deadline = time.time() + 30.0
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if "mujoco_ros2_control" in self.node.get_node_names():
                return
        self.fail(
            "mujoco_ros2_control node did not come up; "
            "MuJoCo failed to load the franka example model"
        )
