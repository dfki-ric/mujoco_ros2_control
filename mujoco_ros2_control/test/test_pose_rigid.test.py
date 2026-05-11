#!/bin/env python3
"""Ground-truth test for pose_broadcaster + framepos/framequat sensors.

Two probe bodies are parented to "world" with a known, constant pose:
  probe_fixed  - fixed joint, 30 deg pitch, at (1.0, 2.0, 3.0)
  probe_float  - floating joint (free body, gravity off), 45 deg yaw,
                 at (-1.0, -2.0, 5.0)

The test verifies that the PoseStamped message and the broadcast TF for each
probe match the URDF-declared pose to within a tight tolerance.
"""

import math
import os
import time
import unittest

import launch
import launch.actions
import launch_testing.actions
import pytest
import rclpy
import rclpy.node
import xacro
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from launch import LaunchContext
from launch.actions import LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
import tf2_ros


# Ground truth (must match test_pose_rigid.urdf.xacro)
EXPECTED = {
    "probe_fixed": {
        "xyz": (1.0, 2.0, 3.0),
        "rpy": (0.0, math.radians(30.0), 0.0),
        "child_frame": "probe_fixed_meas",
        "topic": "/probe_fixed_pose_broadcaster/pose",
    },
    "probe_float": {
        "xyz": (-1.0, -2.0, 5.0),
        "rpy": (0.0, 0.0, math.radians(45.0)),
        "child_frame": "probe_float_meas",
        "topic": "/probe_float_pose_broadcaster/pose",
    },
    "probe_fixed_site": {
        "xyz": (1.0, 2.0, 3.0),
        "rpy": (0.0, math.radians(30.0), 0.0),
        "child_frame": "probe_fixed_meas_site",
        "topic": "/probe_fixed_pose_site_broadcaster/pose",
    },
    "probe_float_site": {
        "xyz": (-1.0, -2.0, 5.0),
        "rpy": (0.0, 0.0, math.radians(45.0)),
        "child_frame": "probe_float_meas_site",
        "topic": "/probe_float_pose_site_broadcaster/pose",
    },
}

POS_TOL = 1e-3
QUAT_TOL = 1e-3


def quat_from_rpy(roll, pitch, yaw):
    """Return (x, y, z, w) for an intrinsic ZYX (yaw-pitch-roll) rotation."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


def create_nodes(context: LaunchContext):
    mujoco_model_path = "/tmp/mujoco_test_pose_rigid"
    mujoco_model_file = os.path.join(mujoco_model_path, "main.xml")

    pkg_share = get_package_share_directory("mujoco_ros2_control")
    xacro_file = os.path.join(pkg_share, "test_data", "test_pose_rigid.urdf.xacro")
    controllers_file = os.path.join(
        pkg_share, "test_data", "test_pose_rigid_controllers.yaml"
    )
    scene_file = os.path.join(pkg_share, "mjcf", "scene.xml")

    robot_description = {
        "robot_description": xacro.process_file(xacro_file).toprettyxml(indent="  ")
    }

    xacro2mjcf = Node(
        package="mujoco_ros2_control",
        executable="xacro2mjcf.py",
        parameters=[
            {"robot_descriptions": [robot_description["robot_description"]]},
            {"input_files": [scene_file]},
            {"output_file": mujoco_model_file},
            {"mujoco_files_path": mujoco_model_path},
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            robot_description,
            controllers_file,
            {"simulation_frequency": 100.0},
            {"realtime_factor": 1.0},
            {"robot_model_path": mujoco_model_file},
            {"show_gui": False},
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    start_mujoco = RegisterEventHandler(
        OnProcessExit(
            target_action=xacro2mjcf,
            on_exit=[LogInfo(msg="MJCF created, starting mujoco..."), mujoco],
        )
    )

    spawn_fixed = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "probe_fixed_pose_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_file,
        ],
    )
    spawn_float = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "probe_float_pose_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_file,
        ],
    )


    spawn_site_fixed = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "probe_fixed_pose_site_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_file,
        ],
    )

    spawn_site_float = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "probe_float_pose_site_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_file,
        ],
    )

    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="Spawning pose broadcasters..."),
                spawn_fixed,
                spawn_float,
                spawn_site_fixed,
                spawn_site_float,
            ],
        )
    )

    return [robot_state_publisher, xacro2mjcf, start_mujoco, load_controllers]


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription(
        [
            OpaqueFunction(function=create_nodes),
            launch.actions.TimerAction(
                period=8.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]
    )


class _Listener(rclpy.node.Node):
    def __init__(self):
        super().__init__("test_pose_rigid_listener")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def wait_for_pose(self, topic, timeout=15.0):
        msgs = []
        sub = self.create_subscription(
            PoseStamped, topic, lambda m: msgs.append(m), 10
        )
        deadline = time.time() + timeout
        while time.time() < deadline and len(msgs) < 3:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        return msgs[-1] if msgs else None

    def wait_for_tf(self, parent, child, timeout=15.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.tf_buffer.can_transform(parent, child, rclpy.time.Time()):
                return self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time()
                )
        return None


class TestPoseRigid(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = _Listener()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _check_pose(self, name):
        spec = EXPECTED[name]
        msg = self.node.wait_for_pose(spec["topic"])
        self.assertIsNotNone(msg, f"no PoseStamped on {spec['topic']}")
        self.assertEqual(msg.header.frame_id, "world")

        x, y, z = spec["xyz"]
        qx, qy, qz, qw = quat_from_rpy(*spec["rpy"])
        p = msg.pose

        self.assertAlmostEqual(p.position.x, x, delta=POS_TOL, msg=f"{name} pos.x")
        self.assertAlmostEqual(p.position.y, y, delta=POS_TOL, msg=f"{name} pos.y")
        self.assertAlmostEqual(p.position.z, z, delta=POS_TOL, msg=f"{name} pos.z")

        # Quaternion sign is ambiguous: q and -q represent the same rotation.
        if p.orientation.w * qw < 0:
            qx, qy, qz, qw = -qx, -qy, -qz, -qw
        self.assertAlmostEqual(p.orientation.x, qx, delta=QUAT_TOL, msg=f"{name} q.x")
        self.assertAlmostEqual(p.orientation.y, qy, delta=QUAT_TOL, msg=f"{name} q.y")
        self.assertAlmostEqual(p.orientation.z, qz, delta=QUAT_TOL, msg=f"{name} q.z")
        self.assertAlmostEqual(p.orientation.w, qw, delta=QUAT_TOL, msg=f"{name} q.w")

    def _check_tf(self, name):
        spec = EXPECTED[name]
        tf = self.node.wait_for_tf("world", spec["child_frame"])
        self.assertIsNotNone(tf, f"no TF world->{spec['child_frame']}")

        x, y, z = spec["xyz"]
        qx, qy, qz, qw = quat_from_rpy(*spec["rpy"])
        t = tf.transform.translation
        q = tf.transform.rotation

        self.assertAlmostEqual(t.x, x, delta=POS_TOL, msg=f"{name} tf.x")
        self.assertAlmostEqual(t.y, y, delta=POS_TOL, msg=f"{name} tf.y")
        self.assertAlmostEqual(t.z, z, delta=POS_TOL, msg=f"{name} tf.z")
        if q.w * qw < 0:
            qx, qy, qz, qw = -qx, -qy, -qz, -qw
        self.assertAlmostEqual(q.x, qx, delta=QUAT_TOL, msg=f"{name} tf.qx")
        self.assertAlmostEqual(q.y, qy, delta=QUAT_TOL, msg=f"{name} tf.qy")
        self.assertAlmostEqual(q.z, qz, delta=QUAT_TOL, msg=f"{name} tf.qz")
        self.assertAlmostEqual(q.w, qw, delta=QUAT_TOL, msg=f"{name} tf.qw")

    def test_fixed_pose(self):
        self._check_pose("probe_fixed")

    def test_fixed_tf(self):
        self._check_tf("probe_fixed")

    def test_float_pose(self):
        self._check_pose("probe_float")

    def test_float_tf(self):
        self._check_tf("probe_float")

    def test_fixed_site_pose(self):
        self._check_pose("probe_fixed_site")

    def test_fixed_site_tf(self):
        self._check_tf("probe_fixed_site")

    def test_float_site_pose(self):
        self._check_pose("probe_float_site")

    def test_float_site_tf(self):
        self._check_tf("probe_float_site")