#!/bin/env python3
"""Launch smoke test for the ur_multi example (no viewer; GL sensors off with DISABLE_OPENGL=1).

Three UR arms in one MuJoCo world, one controller manager. Beyond the shared
"does the node come up" check (see example_smoke.py), this asserts what is
specific to running several arms together: that each arm contributed its own
prefixed joints to the merged model, and that the controller manager really did
instantiate one hardware component per arm rather than collapsing them.
"""

import os
import sys
import time
import unittest
import xml.etree.ElementTree as ET

import pytest
import rclpy
import rclpy.node

# example_smoke.py lives next to this test; make sure it is importable regardless
# of how the launch_test runner sets up sys.path.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from example_smoke import (  # noqa: E402
    availability,
    make_skipped_description,
    make_test_description,
)

ROBOT = "ur_multi"

# Each arm is prefixed with its own UR type, and the upper-arm length of that
# type is taken from ur_description/config/<type>/default_kinematics.yaml. That
# length is the cleanest fingerprint of a model in the generated MJCF: link and
# joint names are identical across UR types, only the geometry differs - so it
# catches an arm being built as the wrong type behind a correct-looking name.
UPPER_ARM_LENGTH = {"ur3e": 0.24355, "ur5e": 0.425, "ur10e": 0.6127}
PREFIXES = tuple(f"{t}_" for t in UPPER_ARM_LENGTH)
_AVAILABLE, _SKIP_REASON = availability(ROBOT)

MODEL_PATH = f"/tmp/mujoco_example_{ROBOT}"
MODEL_FILE = os.path.join(MODEL_PATH, "main.xml")


@pytest.mark.launch_test
def generate_test_description():
    if not _AVAILABLE:
        return make_skipped_description()
    return make_test_description(ROBOT)


class TestUrMultiExample(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.node.Node("test_ur_multi_example_node")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _mjcf_trees(self):
        """The merged model plus every file it includes."""
        root = ET.parse(MODEL_FILE).getroot()
        trees = [root]
        for inc in root.iter("include"):
            path = os.path.join(MODEL_PATH, inc.get("file", ""))
            if os.path.exists(path):
                trees.append(ET.parse(path).getroot())
        return trees

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
            "MuJoCo failed to load the ur_multi example model"
        )

    @unittest.skipUnless(_AVAILABLE, _SKIP_REASON)
    def test_every_arm_reached_the_model(self):
        """All three arms must survive into the MJCF, not just the first."""
        self.assertTrue(os.path.exists(MODEL_FILE), f"no model at {MODEL_FILE}")
        trees = self._mjcf_trees()
        joints = {j.get("name") for t in trees for j in t.iter("joint")}
        actuators = {a.get("name") for t in trees
                     for group in t.iter("actuator") for a in group}
        for prefix in PREFIXES:
            self.assertIn(f"{prefix}shoulder_pan_joint", joints)
            self.assertIn(f"{prefix}wrist_3_joint", joints)
            self.assertIn(f"pos_{prefix}elbow_joint", actuators)

    @unittest.skipUnless(_AVAILABLE, _SKIP_REASON)
    def test_arms_stand_apart(self):
        """A row, not three arms stacked on the same spot."""
        trees = self._mjcf_trees()
        bases = {}
        for tree in trees:
            for body in tree.iter("body"):
                name = body.get("name") or ""
                for prefix in PREFIXES:
                    if name == f"{prefix}base_link" and body.get("pos"):
                        bases[prefix] = [float(v) for v in body.get("pos").split()]
        self.assertEqual(set(bases), set(PREFIXES), f"missing base bodies: {bases}")
        ys = sorted(pos[1] for pos in bases.values())
        self.assertEqual(len(set(ys)), 3, f"arms share a y position: {bases}")
        # Evenly spaced, and far enough apart that neighbours cannot overlap.
        self.assertAlmostEqual(ys[1] - ys[0], ys[2] - ys[1], places=3)
        self.assertGreater(ys[1] - ys[0], 0.5, f"arms too close: {ys}")

    @unittest.skipUnless(_AVAILABLE, _SKIP_REASON)
    def test_each_prefix_carries_the_arm_it_names(self):
        """ur10e_ must be a ur10e, not a ur5e wearing the name."""
        trees = self._mjcf_trees()
        # Each arm's forearm body sits at its own upper-arm length from the
        # elbow, so that offset says which UR type was actually instantiated.
        found = {}
        for tree in trees:
            for body in tree.iter("body"):
                name = body.get("name") or ""
                for ur_type in UPPER_ARM_LENGTH:
                    if name == f"{ur_type}_forearm_link" and body.get("pos"):
                        found[ur_type] = abs(float(body.get("pos").split()[0]))
        self.assertEqual(set(found), set(UPPER_ARM_LENGTH),
                         f"missing forearm bodies: {sorted(found)}")
        for ur_type, expected in UPPER_ARM_LENGTH.items():
            self.assertAlmostEqual(
                found[ur_type], expected, places=3,
                msg=f"{ur_type}_ has an upper arm of {found[ur_type]:.5f} m, but a "
                    f"{ur_type} is {expected} m - it was built as the wrong type")

    @unittest.skipUnless(_AVAILABLE, _SKIP_REASON)
    def test_one_hardware_component_per_arm(self):
        """Each arm is its own ros2_control component, so it can be driven alone."""
        from controller_manager_msgs.srv import ListHardwareComponents

        client = self.node.create_client(
            ListHardwareComponents, "/controller_manager/list_hardware_components")
        self.assertTrue(client.wait_for_service(timeout_sec=30.0),
                        "controller_manager never offered list_hardware_components")
        future = client.call_async(ListHardwareComponents.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=15.0)
        self.assertIsNotNone(future.result(), "list_hardware_components did not answer")

        names = {c.name for c in future.result().component}
        for prefix in PREFIXES:
            self.assertIn(f"{prefix}UrMujocoSystem", names,
                          f"no hardware component for {prefix}; got {sorted(names)}")
