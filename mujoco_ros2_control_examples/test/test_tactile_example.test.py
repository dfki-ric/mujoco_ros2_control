#!/bin/env python3
"""Covers the tactile example, and with it the out-of-tree sensor plugin path.

TouchGridSensor lives in this package rather than in mujoco_ros2_control, so
this exercises the whole chain a third-party plugin depends on: the plugin
manifest exported from a package that only *uses* mujoco_ros2_control, pluginlib
resolving the class through the ament index, and the plugin linking the same
libmujoco the simulator does.

What is asserted, beyond "a topic exists":

  * the array has the shape the MJCF declared, and carries the layout labels a
    consumer needs to index it as an image,
  * some taxel reports real contact force. Without that the test would pass on a
    plugin publishing a correctly shaped block of zeros, which is exactly what a
    mis-oriented touch_grid site produces.
"""

import os
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
import rclpy.node
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray

# Matches the <config> block in urdf/tactile/tactile_pad.urdf.xacro.
TOUCH_CHANNELS = 3
TOUCH_WIDTH = 7
TOUCH_HEIGHT = 7
TOUCH_TOPIC = "/pad_touch/touch_grid"

# The pad weighs 0.1 kg, so it presses into the floor with ~0.98 N. How that
# splits across taxels depends on the contact points, so only require that some
# taxel is clearly off zero.
MIN_CONTACT_FORCE = 0.1


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory("mujoco_ros2_control_examples"),
        "launch", "tactile.launch.py",
    )
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={"headless": "true"}.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("tactile_example_test_node")

    def wait_for_message(self, topic, msg_type, timeout=45.0, predicate=None):
        """Spin until a message arrives that satisfies `predicate` (if given)."""
        received = []

        def callback(msg):
            if predicate is None or predicate(msg):
                received.append(msg)

        sub = self.create_subscription(
            msg_type, topic, callback, qos_profile_sensor_data)
        try:
            deadline = self.get_clock().now().nanoseconds + timeout * 1e9
            while not received and self.get_clock().now().nanoseconds < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self.destroy_subscription(sub)
        return received[0] if received else None


class TestTactileExample(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = TestNode()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_1_touch_grid_is_published(self):
        """The out-of-tree plugin loaded and is publishing."""
        msg = self.node.wait_for_message(TOUCH_TOPIC, Float64MultiArray)
        self.assertIsNotNone(
            msg,
            f"no message on {TOUCH_TOPIC}; the TouchGridSensor plugin exported by "
            "this package may not have been resolved by pluginlib",
        )
        self.assertEqual(
            len(msg.data), TOUCH_CHANNELS * TOUCH_WIDTH * TOUCH_HEIGHT,
            "tactile array has the wrong number of taxels",
        )

    def test_2_touch_grid_layout_is_labelled(self):
        """Consumers need the layout to index the flat array as an image."""
        msg = self.node.wait_for_message(TOUCH_TOPIC, Float64MultiArray)
        self.assertIsNotNone(msg, f"no message on {TOUCH_TOPIC}")

        self.assertEqual(
            [d.label for d in msg.layout.dim], ["channel", "height", "width"])
        self.assertEqual(
            [d.size for d in msg.layout.dim],
            [TOUCH_CHANNELS, TOUCH_HEIGHT, TOUCH_WIDTH])
        # Channel-major: [k*width*height + j*width + i]
        self.assertEqual(
            [d.stride for d in msg.layout.dim],
            [TOUCH_CHANNELS * TOUCH_WIDTH * TOUCH_HEIGHT,
             TOUCH_WIDTH * TOUCH_HEIGHT,
             TOUCH_WIDTH])

    def test_3_touch_grid_reports_contact(self):
        """The pad rests on the floor, so some taxel must register its weight."""
        msg = self.node.wait_for_message(
            TOUCH_TOPIC, Float64MultiArray,
            predicate=lambda m: any(abs(v) > MIN_CONTACT_FORCE for v in m.data),
        )
        self.assertIsNotNone(
            msg,
            f"no taxel exceeded {MIN_CONTACT_FORCE} N; the pad may not be in "
            "contact, or the touch_grid site may be facing the wrong way "
            "(it images along -z)",
        )
