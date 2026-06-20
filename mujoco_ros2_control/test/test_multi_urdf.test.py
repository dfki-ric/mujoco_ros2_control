#!/bin/env python3
"""Multi-URDF input test for xacro2mjcf.

Feeds two separate robot_descriptions (each a floating body at a distinct pose
plus a MuJoCo sensor) to the xacro2mjcf converter and checks the generated MJCF:

  * both bodies are present,
  * both per-file sensors are present,
  * there is exactly ONE <keyframe> whose qpos covers BOTH free joints (the
    "single initial keyframe for all included files" behaviour) and contains
    each body's distinct initial pose, and
  * MuJoCo actually accepts the merged model: the mujoco_ros2_control node is
    started on it and must come up (a malformed MJCF would FATAL on load).
"""

import os
import time
import unittest
import xml.etree.ElementTree as ET

import launch
import launch.actions
import launch_testing.actions
import pytest
import rclpy
import rclpy.node
from ament_index_python import get_package_share_directory
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


MODEL_PATH = "/tmp/mujoco_multi_urdf"
MODEL_FILE = os.path.join(MODEL_PATH, "main.xml")


def _floating_robot(name, xyz):
    """A minimal URDF: a floating body at xyz (parented to world) plus a framepos
    sensor. The root link must be named "world" (the converter maps it to the MJCF
    worldbody), and the free body must be a direct child of world so MuJoCo accepts
    its free joint. "world" is never emitted as a named body, so two such URDFs
    merge without a body-name clash."""
    return f"""<?xml version="1.0"?>
<robot name="{name}">
  <link name="world"/>
  <link name="{name}_body">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="{name}_float" type="floating">
    <parent link="world"/>
    <child link="{name}_body"/>
    <origin xyz="{xyz}" rpy="0 0 0"/>
  </joint>
  <mujoco>
    <sensor>
      <framepos name="{name}_pos" objtype="xbody" objname="{name}_body"
                reftype="body" refname="world"/>
    </sensor>
  </mujoco>
</robot>"""


@pytest.mark.launch_test
def generate_test_description():
    scene_file = os.path.join(
        get_package_share_directory("mujoco_ros2_control"), "mjcf", "scene.xml")
    xacro2mjcf = Node(
        package="mujoco_ros2_control",
        executable="xacro2mjcf.py",
        parameters=[
            {"robot_descriptions": [
                _floating_robot("alpha", "1.0 0.0 2.0"),
                _floating_robot("beta", "-1.0 0.0 3.0"),
            ]},
            {"input_files": [scene_file]},
            {"output_file": MODEL_FILE},
            {"mujoco_files_path": MODEL_PATH},
        ],
    )

    # Once the model is generated, start mujoco on it to confirm MuJoCo loads the
    # merged model (a malformed MJCF makes this node FATAL and exit).
    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            {"robot_model_path": MODEL_FILE},
            {"simulation_frequency": 100.0},
            {"realtime_factor": 1.0},
            {"show_gui": False},
        ],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
    )

    return launch.LaunchDescription([
        xacro2mjcf,
        RegisterEventHandler(
            OnProcessExit(
                target_action=xacro2mjcf,
                on_exit=[
                    mujoco,
                    # Give mujoco a moment to load the model and start spinning
                    # before the tests query the node graph.
                    TimerAction(period=4.0, actions=[launch_testing.actions.ReadyToTest()]),
                ],
            )
        ),
    ])


class TestMultiUrdf(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.node.Node("test_multi_urdf_node")

        assert os.path.exists(MODEL_FILE), f"converter did not produce {MODEL_FILE}"
        # main.xml holds the merged <keyframe> and <include>s the per-robot files;
        # the bodies and sensors live in those included files.
        cls.root = ET.parse(MODEL_FILE).getroot()
        cls.trees = [cls.root]
        for inc in cls.root.iter("include"):
            path = os.path.join(MODEL_PATH, inc.get("file", ""))
            if os.path.exists(path):
                cls.trees.append(ET.parse(path).getroot())

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _names(self, tag):
        return {e.get("name") for t in self.trees for e in t.iter(tag)}

    def test_both_bodies_present(self):
        names = self._names("body")
        self.assertIn("alpha_body", names)
        self.assertIn("beta_body", names)

    def test_both_sensors_present(self):
        names = self._names("framepos")
        self.assertIn("alpha_pos", names)
        self.assertIn("beta_pos", names)

    def test_single_merged_keyframe(self):
        keyframes = list(self.root.iter("keyframe"))
        self.assertEqual(len(keyframes), 1, "expected exactly one <keyframe> for all files")
        keys = list(keyframes[0].iter("key"))
        self.assertEqual(len(keys), 1, "expected exactly one <key> in the keyframe")

        qpos = [float(v) for v in keys[0].get("qpos").split()]
        # Two free joints -> 7 qpos each (xyz + wxyz quat).
        self.assertEqual(len(qpos), 14, f"expected 14 qpos values, got {len(qpos)}: {qpos}")

        # Both bodies' distinct initial heights must appear in the single keyframe;
        # if the per-file keyframes were not merged, one of them would be missing.
        self.assertTrue(any(abs(v - 2.0) < 1e-6 for v in qpos), f"alpha z=2.0 missing: {qpos}")
        self.assertTrue(any(abs(v - 3.0) < 1e-6 for v in qpos), f"beta z=3.0 missing: {qpos}")

    def test_model_loads_in_mujoco(self):
        # The mujoco node only registers in the graph once it has loaded the merged
        # MJCF; if MuJoCo rejected the model the node FATALs and never appears.
        deadline = time.time() + 20.0
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if "mujoco_ros2_control" in self.node.get_node_names():
                return
        self.fail("mujoco_ros2_control node did not come up; MuJoCo failed to load the merged model")
