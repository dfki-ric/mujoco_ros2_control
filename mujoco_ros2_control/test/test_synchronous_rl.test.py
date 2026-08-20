#!/usr/bin/env python3
"""Deterministic stepping, body-state, pose, and reset regression test."""

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
from mujoco_ros2_control.srv import GetBodyState, SetBodyPose, StepSimulation
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger


def create_nodes(context: LaunchContext):
    del context
    package = get_package_share_directory("mujoco_ros2_control")
    model_directory = "/tmp/mujoco_test_synchronous_rl"
    model_file = os.path.join(model_directory, "main.xml")
    description = xacro.process_file(
        os.path.join(package, "test_data", "test_pose_rigid.urdf.xacro")
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

    def test_exact_step_pose_and_reset(self):
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

        get_request = GetBodyState.Request()
        get_request.body_name = "probe_float"
        initial = self.call(GetBodyState, "/mujoco_get_body_state", get_request)
        self.assertTrue(initial.success, initial.message)

        set_request = SetBodyPose.Request()
        set_request.body_name = "probe_float"
        set_request.x, set_request.y, set_request.z = 0.5, -0.25, 1.75
        set_request.qw, set_request.qx, set_request.qy, set_request.qz = 1.0, 0.0, 0.0, 0.0
        moved = self.call(SetBodyPose, "/mujoco_set_body_pose", set_request)
        self.assertTrue(moved.success, moved.message)
        observed = self.call(GetBodyState, "/mujoco_get_body_state", get_request)
        self.assertAlmostEqual(observed.pose.position.x, 0.5, places=8)
        self.assertAlmostEqual(observed.pose.position.y, -0.25, places=8)
        self.assertAlmostEqual(observed.pose.position.z, 1.75, places=8)

        reset = self.call(Trigger, "/mujoco_reset", Trigger.Request())
        self.assertTrue(reset.success, reset.message)
        restored = self.call(GetBodyState, "/mujoco_get_body_state", get_request)
        self.assertAlmostEqual(restored.pose.position.x, initial.pose.position.x, places=8)
        self.assertAlmostEqual(restored.pose.position.y, initial.pose.position.y, places=8)
        self.assertAlmostEqual(restored.pose.position.z, initial.pose.position.z, places=8)

        one_step = StepSimulation.Request()
        one_step.steps = 1
        after_reset = self.call(StepSimulation, "/mujoco_step_simulation", one_step)
        reset_time = after_reset.simulation_time.sec + after_reset.simulation_time.nanosec * 1e-9
        self.assertAlmostEqual(reset_time, 0.01, places=8)
