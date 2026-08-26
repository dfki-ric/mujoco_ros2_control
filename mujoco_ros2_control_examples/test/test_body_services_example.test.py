#!/usr/bin/env python3
"""Body services example: reading and teleporting bodies from outside.

Covers the BodyServices plugin, which lives in this package rather than in
mujoco_ros2_control. As with the tactile example, that exercises the whole
out-of-tree plugin chain -- the manifest exported from a package that only *uses*
mujoco_ros2_control, pluginlib resolving the class through the ament index, the
plugin linking the same libmujoco as the simulator -- and additionally the case
of a plugin that brings its own generated service types.

Run in synchronous mode, which is the mode these services exist for and the one
that pins down their contract: nothing advances the simulation except a step
request, so a pose written and read straight back must come back unchanged. That
only holds because the write happens in the service callback rather than being
queued for the next step.

What is asserted:

  * both declarations come up, and the renamed pair works rather than merely
    existing -- the parameters are what let several models in one simulation each
    have their own services instead of fighting over the default names,
  * a teleport through one instance is visible through the other: one simulation,
    two front doors,
  * reset restores the body state (this assertion used to live in
    mujoco_ros2_control's test_synchronous_rl, which can no longer reach the
    plugin),
  * the two rejection paths: a welded body cannot be teleported, and an unknown
    body name is an error rather than a crash,
  * a teleport shows up on the ordinary ros2_control path too, on the pose
    broadcaster's topic -- which is also what catches the controllers not
    receiving their parameters.
"""

import os
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
import rclpy.node
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from geometry_msgs.msg import PoseStamped
from mujoco_ros2_control.srv import StepSimulation
from mujoco_ros2_control_examples.srv import GetBodyState, SetBodyPose
from std_srvs.srv import Trigger

# Matches urdf/body_services/probe_bodies.urdf.xacro.
FREE_BODY = "probe_float"
WELDED_BODY = "probe_fixed"


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory("mujoco_ros2_control_examples"),
        "launch", "body_services.launch.py",
    )
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={"headless": "true", "synchronous": "true"}.items(),
        ),
        TimerAction(period=5.0, actions=[launch_testing.actions.ReadyToTest()]),
    ])


class TestBodyServicesExample(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("body_services_example_test_node")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def call(cls, service_type, name, request, timeout=30.0):
        client = cls.node.create_client(service_type, name)
        assert client.wait_for_service(timeout_sec=timeout), f"missing service {name}"
        future = client.call_async(request)
        rclpy.spin_until_future_complete(cls.node, future, timeout_sec=timeout)
        assert future.done(), f"service timed out: {name}"
        response = future.result()
        cls.node.destroy_client(client)
        return response

    def get_state(self, service="/mujoco_get_body_state", body=FREE_BODY):
        request = GetBodyState.Request()
        request.body_name = body
        return self.call(GetBodyState, service, request)

    def set_pose(self, x, y, z, service="/mujoco_set_body_pose", body=FREE_BODY):
        request = SetBodyPose.Request()
        request.body_name = body
        request.x, request.y, request.z = x, y, z
        request.qw, request.qx, request.qy, request.qz = 1.0, 0.0, 0.0, 0.0
        return self.call(SetBodyPose, service, request)

    def assert_position(self, response, x, y, z):
        self.assertAlmostEqual(response.pose.position.x, x, places=8)
        self.assertAlmostEqual(response.pose.position.y, y, places=8)
        self.assertAlmostEqual(response.pose.position.z, z, places=8)

    def test_1_both_declarations_come_up(self):
        """One plugin declared twice gives two nodes, hence two service pairs."""
        deadline = time.time() + 30.0
        while time.time() < deadline:
            names = self.node.get_node_names()
            if "body_services" in names and "body_services_alt" in names:
                break
            rclpy.spin_once(self.node, timeout_sec=0.1)
        names = self.node.get_node_names()
        self.assertIn(
            "body_services", names,
            "the BodyServices plugin did not come up; pluginlib may not have "
            "resolved mujoco_ros2_control_examples/BodyServices",
        )
        self.assertIn(
            "body_services_alt", names, "the renamed instance did not come up")

    def test_2_renamed_instance_reads_the_same_simulation(self):
        """The renamed pair has to work, not merely exist."""
        alt = self.get_state("/alt/get_body_state")
        self.assertTrue(alt.success, alt.message)

        default = self.get_state()
        self.assertTrue(default.success, default.message)
        self.assert_position(
            alt,
            default.pose.position.x,
            default.pose.position.y,
            default.pose.position.z,
        )

    def test_3_teleport_is_visible_through_the_other_instance(self):
        """Write through one front door, read through the other."""
        moved = self.set_pose(-1.5, 0.75, 2.25, service="/alt/set_body_pose")
        self.assertTrue(moved.success, moved.message)

        # No step in between: the write lands in the callback, and mj_forward
        # makes xpos/xquat agree with it immediately. A queued write would not be
        # visible here, because nothing advances a synchronous simulation on its
        # own.
        self.assert_position(self.get_state(), -1.5, 0.75, 2.25)

        self.call(Trigger, "/mujoco_reset", Trigger.Request())

    def test_4_reset_restores_the_body_state(self):
        initial = self.get_state()
        self.assertTrue(initial.success, initial.message)

        moved = self.set_pose(0.5, -0.25, 1.75)
        self.assertTrue(moved.success, moved.message)
        self.assert_position(self.get_state(), 0.5, -0.25, 1.75)

        reset = self.call(Trigger, "/mujoco_reset", Trigger.Request())
        self.assertTrue(reset.success, reset.message)
        self.assert_position(
            self.get_state(),
            initial.pose.position.x,
            initial.pose.position.y,
            initial.pose.position.z,
        )

        # Reset rewinds simulation time too, so the next step lands on one tick
        # of the 100 Hz simulation.
        request = StepSimulation.Request()
        request.steps = 1
        stepped = self.call(StepSimulation, "/mujoco_step_simulation", request)
        self.assertTrue(stepped.success, stepped.message)
        self.assertAlmostEqual(
            stepped.simulation_time.sec + stepped.simulation_time.nanosec * 1e-9,
            0.01,
            places=8,
        )

    def test_5_teleport_rejects_a_body_without_a_free_joint(self):
        """probe_fixed is welded, so there are no degrees of freedom to write."""
        response = self.set_pose(0.0, 0.0, 0.0, body=WELDED_BODY)
        self.assertFalse(response.success)
        self.assertIn("free joint", response.message)

    def test_6_body_state_rejects_an_unknown_body(self):
        response = self.get_state(body="no_such_body")
        self.assertFalse(response.success)
        self.assertIn("Unknown body", response.message)

    def test_7_teleport_is_visible_on_the_pose_broadcaster(self):
        """The services and the ros2_control sensors see the same simulation.

        Also the only assertion that fails if the spawners come up without their
        --param-file: an unconfigured PoseBroadcaster does not start at all.
        """
        topic = "/probe_float_pose_broadcaster/pose"

        deadline = time.time() + 30.0
        while time.time() < deadline and self.node.count_publishers(topic) < 1:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertGreaterEqual(
            self.node.count_publishers(topic), 1,
            f"nothing publishes {topic}; the pose broadcaster did not start",
        )

        received = []
        subscription = self.node.create_subscription(
            PoseStamped, topic, lambda msg: received.append(msg), 10)
        self.addCleanup(self.node.destroy_subscription, subscription)

        moved = self.set_pose(2.0, -0.5, 1.25)
        self.assertTrue(moved.success, moved.message)

        # Synchronous mode: the controller only runs when the simulation steps,
        # so the broadcaster publishes nothing until one is asked for.
        request = StepSimulation.Request()
        request.steps = 1
        self.call(StepSimulation, "/mujoco_step_simulation", request)

        deadline = time.time() + 15.0
        match = None
        while time.time() < deadline and match is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            match = next(
                (m for m in received
                 if abs(m.pose.position.x - 2.0) < 1e-6
                 and abs(m.pose.position.y + 0.5) < 1e-6
                 and abs(m.pose.position.z - 1.25) < 1e-6),
                None,
            )
        self.assertIsNotNone(
            match,
            f"no pose on {topic} matched the teleport; got "
            f"{[(m.pose.position.x, m.pose.position.y, m.pose.position.z) for m in received]}",
        )

        self.call(Trigger, "/mujoco_reset", Trigger.Request())
