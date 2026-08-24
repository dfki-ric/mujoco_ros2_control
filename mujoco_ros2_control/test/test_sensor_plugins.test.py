#!/bin/env python3
"""End-to-end test for the pluginlib sensor path (MujocoSensorInterface).

A 0.1 kg pad rests on the ground plane carrying four plugin-loaded sensors:

  pad_imu     ImuSensor          -> imu_sensor_broadcaster
  pad_wrench  ForceTorqueSensor  -> force_torque_sensor_broadcaster
  pad_pose    PoseSensor         -> pose_broadcaster

The wrench sensor sits on a separate welded "hanger" body rather than on the
pad; see the model for why a body that settles into contact reports zero force.

The tactile plugin lives in mujoco_ros2_control_examples and is covered by
test_tactile_example.test.py there.

What this pins down, beyond "it starts":

  * plugins named by a <param name="plugin"> are actually instantiated, rather
    than the <sensor> silently falling through to the deprecated built-in
    classifier,
  * the three interface-exporting plugins stay wire-compatible with the stock
    broadcasters, reporting values that match the model's ground truth.
"""

import os
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
import rclpy.node
import xacro
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import PoseStamped, WrenchStamped
from launch import LaunchContext
from launch.actions import LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from sensor_msgs.msg import Imu

# The hanger weighs 0.2 kg and is welded to the world, so the joint carries its
# full weight. That is an exact ground truth, unlike the tactile split.
HANGER_WEIGHT_N = 0.2 * 9.81
FORCE_TOL_N = 0.05


def create_nodes(context: LaunchContext):
    mujoco_model_path = "/tmp/mujoco_test_sensor_plugins"
    mujoco_model_file = os.path.join(mujoco_model_path, "main.xml")

    pkg_share = get_package_share_directory("mujoco_ros2_control")
    xacro_file = os.path.join(pkg_share, "test_data", "test_sensor_plugins.urdf.xacro")
    controllers_file = os.path.join(
        pkg_share, "test_data", "test_sensor_plugins_controllers.yaml"
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
            {"simulation_frequency": 200.0},
            {"real_time_factor": 1.0},
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

    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                name,
                "--controller-manager",
                "/controller_manager",
                "--param-file",
                controllers_file,
            ],
        )
        for name in (
            "pad_imu_broadcaster",
            "pad_wrench_broadcaster",
            "pad_pose_broadcaster",
        )
    ]

    return [xacro2mjcf, robot_state_publisher, start_mujoco, *spawners]


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription(
        [OpaqueFunction(function=create_nodes), launch_testing.actions.ReadyToTest()]
    )


class TestNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("sensor_plugins_test_node")

    def wait_for_message(self, topic, msg_type, timeout=30.0, qos=10, predicate=None):
        """Spin until a message arrives that satisfies `predicate` (if given)."""
        received = []

        def callback(msg):
            if predicate is None or predicate(msg):
                received.append(msg)

        sub = self.create_subscription(msg_type, topic, callback, qos)
        try:
            deadline = self.get_clock().now().nanoseconds + timeout * 1e9
            while not received and self.get_clock().now().nanoseconds < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self.destroy_subscription(sub)
        return received[0] if received else None


class TestSensorPlugins(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = TestNode()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_1_imu_plugin_feeds_broadcaster(self):
        """ImuSensor exports interfaces imu_sensor_broadcaster understands."""
        msg = self.node.wait_for_message("/pad_imu_broadcaster/imu", Imu, timeout=45.0)
        self.assertIsNotNone(msg, "no message on /pad_imu_broadcaster/imu")
        # A resting body reads ~+9.81 m/s^2 along z (proper acceleration).
        self.assertGreater(
            msg.linear_acceleration.z, 5.0,
            "accelerometer does not see gravity; the IMU plugin may be reading the "
            "wrong sensor address",
        )

    def test_2_force_torque_plugin_feeds_broadcaster(self):
        """ForceTorqueSensor reports the hanger's weight through its weld."""
        msg = self.node.wait_for_message(
            "/pad_wrench_broadcaster/wrench", WrenchStamped, timeout=45.0
        )
        self.assertIsNotNone(msg, "no message on /pad_wrench_broadcaster/wrench")

        force = msg.wrench.force
        magnitude = (force.x ** 2 + force.y ** 2 + force.z ** 2) ** 0.5
        self.assertAlmostEqual(
            magnitude, HANGER_WEIGHT_N, delta=FORCE_TOL_N,
            msg=f"wrench sensor reports {magnitude:.3f} N, expected the hanger's "
                f"{HANGER_WEIGHT_N:.3f} N of weight; the plugin may be reading the "
                "wrong sensor address",
        )

    def test_3_pose_plugin_feeds_broadcaster(self):
        """PoseSensor exports interfaces pose_broadcaster understands."""
        msg = self.node.wait_for_message(
            "/pad_pose_broadcaster/pose", PoseStamped, timeout=45.0
        )
        self.assertIsNotNone(msg, "no message on /pad_pose_broadcaster/pose")
        # The pad settles on the floor, so its centre sits near the box half-height.
        self.assertGreater(msg.pose.position.z, 0.0, "pad reported below the floor")
        self.assertLess(msg.pose.position.z, 0.5, "pad reported implausibly high")
        # A quaternion the broadcaster passed through must be normalised.
        q = msg.pose.orientation
        norm = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) ** 0.5
        self.assertAlmostEqual(norm, 1.0, places=3, msg="orientation is not a unit quaternion")
