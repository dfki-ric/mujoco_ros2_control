#!/bin/env python3
"""Covers decompose_industreal_peg_trays: launches the real franka.launch.py
with it set, and checks CoaCD actually ran, not just that the node came up.

Includes franka.launch.py itself (rather than example_smoke.py's lower-level
launch graph, which never loads the IndustReal task board at all) so this
exercises the production decompose_industreal_peg_trays code path exactly as a
user would trigger it, with no duplicated launch logic to drift out of sync.

Without this flag, the pick/insert tray meshes collide as their convex hull,
which fills in the peg-shaped hole a tray is supposed to have - see "Task
table assets" in README.md. The folder-of-parts check below is the only thing
that would actually fail if CoaCD stopped running (or started silently
failing): the simulator can come up perfectly well with an undecomposed,
hole-free tray, since MuJoCo does not care that its convex hull is wrong, only
that it is a valid mesh.
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
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# example_smoke.py lives next to this test; make sure it is importable regardless
# of how the launch_test runner sets up sys.path.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from example_smoke import availability, make_skipped_description, opengl_enabled  # noqa: E402

# One pick and one insert tray, different shape/size, is enough to prove the
# flag decomposes trays in general without asserting all twelve.
CHECK_TRAYS = [
    "industreal_tray_pick_round_peg_8mm",
    "industreal_tray_insert_rectangular_peg_16mm",
]


def _coacd_available():
    try:
        import coacd  # noqa: F401
        import trimesh  # noqa: F401
    except ModuleNotFoundError as exc:
        return False, f"{exc.name} is not installed (see README.md, 'Task table assets')"
    return True, ""


_FRANKA_AVAILABLE, _FRANKA_SKIP_REASON = availability("franka")
_COACD_AVAILABLE, _COACD_SKIP_REASON = _coacd_available()
_AVAILABLE = _FRANKA_AVAILABLE and _COACD_AVAILABLE
_SKIP_REASON = _FRANKA_SKIP_REASON or _COACD_SKIP_REASON


@pytest.mark.launch_test
def generate_test_description():
    if not _AVAILABLE:
        return make_skipped_description()
    launch_file = os.path.join(
        get_package_share_directory("mujoco_ros2_control_examples"),
        "launch", "franka.launch.py",
    )
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={
                "headless": "true" if not opengl_enabled() else "false",
                "rviz": "false",
                "decompose_industreal_peg_trays": "true",
            }.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestFrankaIndustrealDecomposeExample(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.node.Node("test_franka_industreal_decompose_example_node")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @unittest.skipUnless(_AVAILABLE, _SKIP_REASON)
    def test_node_comes_up(self):
        # Generous timeout: decompose_industreal_peg_trays runs CoaCD over all
        # twelve tray meshes synchronously, inside franka.launch.py's
        # OpaqueFunction, before any node in the included description starts -
        # so all of that time shows up here as "waiting for the node", not
        # inside xacro2mjcf or MuJoCo's own load.
        deadline = time.time() + 1100.0
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if "mujoco_ros2_control" in self.node.get_node_names():
                return
        self.fail(
            "mujoco_ros2_control node did not come up; MuJoCo failed to load "
            "the franka+IndustReal-board model, or CoaCD decomposition failed"
        )

    @unittest.skipUnless(_AVAILABLE, _SKIP_REASON)
    def test_trays_were_decomposed(self):
        # Confirms CoaCD actually ran rather than just checking the node came
        # up: an undecomposed tray still loads fine in MuJoCo (as its convex
        # hull), so test_node_comes_up alone would pass even if this flag were
        # silently doing nothing.
        pegs_dir = os.path.join(
            get_package_share_directory("mujoco_ros2_control_examples"),
            "meshes", "industreal", "pegs",
        )
        for tray in CHECK_TRAYS:
            folder = os.path.join(pegs_dir, tray)
            self.assertTrue(
                os.path.isdir(folder),
                f"{folder} was not created; decompose_industreal_peg_trays did "
                "not decompose this tray",
            )
            parts = [f for f in os.listdir(folder) if f.endswith(".obj")]
            self.assertGreater(
                len(parts), 1,
                f"{folder} holds {len(parts)} convex piece(s); CoaCD normally "
                "splits a tray-with-a-hole into several",
            )
