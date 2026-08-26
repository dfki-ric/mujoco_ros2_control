#!/usr/bin/env python3
"""Deterministic stepping and reset regression test.

The body read/teleport services used to be exercised here too. They are a
plugin in mujoco_ros2_control_examples now, so their coverage -- including
"reset restores the body state" -- lives in that package's
test_body_services_example.test.py, which can depend on it.
"""

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
from launch import LaunchContext
from launch.actions import LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from mujoco_ros2_control.srv import StepSimulation
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


def opengl_enabled():
    """GL toggle. Set DISABLE_OPENGL=1 to skip the camera; CI runs with GL on."""
    return os.environ.get("DISABLE_OPENGL", "0") != "1"


def create_nodes(context: LaunchContext):
    del context
    package = get_package_share_directory("mujoco_ros2_control")
    model_directory = "/tmp/mujoco_test_synchronous_rl"
    model_file = os.path.join(model_directory, "main.xml")
    description = xacro.process_file(
        os.path.join(package, "test_data", "test_pose_rigid.urdf.xacro"),
        mappings={"with_camera": "true" if opengl_enabled() else "false"},
    ).toprettyxml(indent="  ")
    robot_description = {"robot_description": description}
    converter = Node(
        package="mujoco_ros2_control",
        executable="xacro2mjcf.py",
        parameters=[{
            "robot_descriptions": [description],
            "input_files": [os.path.join(package, "mjcf", "scene.xml")],
            "output_file": model_file,
            "mujoco_files_path": model_directory,
        }],
    )
    simulator = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            robot_description,
            os.path.join(package, "test_data", "test_pose_rigid_controllers.yaml"),
            os.path.join(package, "test_data", "test_pose_rigid_camera_params.yaml"),
            {
                "simulation_frequency": 100.0,
                "synchronous_mode": True,
                "robot_model_path": model_file,
                "show_gui": False,
            },
        ],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
    )
    start = RegisterEventHandler(OnProcessExit(
        target_action=converter,
        on_exit=[LogInfo(msg="Starting synchronous MuJoCo test"), simulator],
    ))
    return [converter, start]


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        OpaqueFunction(function=create_nodes),
        launch.actions.TimerAction(
            period=5.0, actions=[launch_testing.actions.ReadyToTest()]
        ),
    ])


class TestSynchronousRl(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_synchronous_rl_client")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def call(cls, service_type, name, request, timeout=10.0):
        client = cls.node.create_client(service_type, name)
        assert client.wait_for_service(timeout_sec=timeout), f"missing service {name}"
        future = client.call_async(request)
        rclpy.spin_until_future_complete(cls.node, future, timeout_sec=timeout)
        assert future.done(), f"service timed out: {name}"
        response = future.result()
        cls.node.destroy_client(client)
        return response

    def test_clock_heartbeat_is_available_without_advancing_physics(self):
        clocks = []
        subscription = self.node.create_subscription(
            Clock, "/clock", lambda message: clocks.append(message.clock), 10
        )
        deadline = time.monotonic() + 2.0
        while len(clocks) < 2 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertGreaterEqual(len(clocks), 2, "missing paused /clock heartbeat")
        timestamps = [clock.sec * 1_000_000_000 + clock.nanosec for clock in clocks]
        self.assertGreater(
            timestamps[0], 0, "paused /clock did not initialize ROS time"
        )
        self.assertTrue(
            all(timestamp == timestamps[0] for timestamp in timestamps),
            "paused /clock heartbeat advanced simulation time",
        )
        self.node.destroy_subscription(subscription)

    def test_exact_step_timing_and_reset(self):
        """A batch adds exactly steps / frequency, and reset rewinds the clock."""
        baseline_request = StepSimulation.Request()
        baseline_request.steps = 1
        baseline = self.call(
            StepSimulation, "/mujoco_step_simulation", baseline_request
        )
        baseline_time = (
            baseline.simulation_time.sec + baseline.simulation_time.nanosec * 1e-9
        )

        # Waiting in wall time must not advance a synchronous simulation, and
        # the following batch must add exactly steps / frequency.
        time.sleep(0.2)
        request = StepSimulation.Request()
        request.steps = 25
        stepped = self.call(StepSimulation, "/mujoco_step_simulation", request)
        self.assertTrue(stepped.success, stepped.message)
        first_time = stepped.simulation_time.sec + stepped.simulation_time.nanosec * 1e-9
        self.assertAlmostEqual(first_time - baseline_time, 0.25, places=8)

        reset = self.call(Trigger, "/mujoco_reset", Trigger.Request())
        self.assertTrue(reset.success, reset.message)

        one_step = StepSimulation.Request()
        one_step.steps = 1
        after_reset = self.call(StepSimulation, "/mujoco_step_simulation", one_step)
        reset_time = after_reset.simulation_time.sec + after_reset.simulation_time.nanosec * 1e-9
        self.assertAlmostEqual(reset_time, 0.01, places=8)

    @unittest.skipUnless(opengl_enabled(), "OpenGL disabled (DISABLE_OPENGL=1)")
    def test_step_waits_for_declared_camera_frame(self):
        """A declared camera is part of the post-step barrier.

        In synchronous mode nothing advances simulation time except a step
        request, and the camera paces itself on simulation time, so its own
        periodic loop never produces anything on its own here -- at 2 Hz it would
        need 0.5 s of simulated time, far more than this test spends. Every frame
        therefore comes from the post-step barrier, and one stamped with exactly
        the returned simulation time must have been published before the response
        arrived. Without the barrier there is no frame at all, and a client reads
        a state it has already stepped past.
        """
        topic = "/sync_camera/color/image_raw"

        # Distinguish "the camera never loaded" from "the barrier did not wait":
        # the publisher has to exist before the assertion below means anything.
        deadline = time.time() + 30.0
        while time.time() < deadline and self.node.count_publishers(topic) < 1:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertGreaterEqual(
            self.node.count_publishers(topic), 1,
            "the declared camera never created its publisher")

        received = []
        subscription = self.node.create_subscription(
            Image, topic, lambda msg: received.append(msg), 10)
        self.addCleanup(self.node.destroy_subscription, subscription)

        request = StepSimulation.Request()
        request.steps = 10
        received.clear()
        stepped = self.call(StepSimulation, "/mujoco_step_simulation", request)
        self.assertTrue(stepped.success, stepped.message)

        target = (stepped.simulation_time.sec
                  + stepped.simulation_time.nanosec * 1e-9)

        # The barrier publishes before responding, so the frame is already in
        # flight; spin only to drain it out of the subscription queue.
        deadline = time.time() + 5.0
        stamps = []
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                      for m in received]
            if any(abs(stamp - target) < 1e-6 for stamp in stamps):
                break

        self.assertTrue(
            any(abs(stamp - target) < 1e-6 for stamp in stamps),
            f"no camera frame stamped at the post-step simulation time {target}; "
            f"got {stamps}. StepSimulation is not waiting for cameras declared as "
            f"<mujoco_ros2_plugin> -- only for prefix-discovered ones.")
