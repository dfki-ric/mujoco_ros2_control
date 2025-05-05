#!/bin/env python3

from typing import List
import os
import time
import unittest
from launch import LaunchContext
import launch.actions
from launch_ros.actions import Node
import launch_testing.actions
from launch.substitutions import PathJoinSubstitution,Command,FindExecutable
from launch_ros.substitutions import FindPackageShare
import pytest
import rclpy
import rclpy.node
from ament_index_python import get_package_share_directory

from launch.actions import (
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction
)

from launch.event_handlers import (
    OnProcessStart,
    OnProcessExit
    )
from sensor_msgs.msg import Imu
from geometry_msgs.msg import WrenchStamped, PoseStamped

import xacro


def create_nodes(context: LaunchContext):
    namespace = ""
    mujoco_model_path = "/tmp/mujoco"
    mujoco_model_file = os.path.join(mujoco_model_path, "main.xml")

    # Set file paths
    double_pendulum_xacro_filepath = os.path.join(
        get_package_share_directory("mujoco_ros2_control"),
        "test_data",
        "double_pendulum.urdf.xacro",
    )

    # Process the xacro file and create the robot description
    robot_description = {
        'robot_description': xacro.process_file(
            double_pendulum_xacro_filepath,
            mappings={
                "command_mode": "effort"
            }
        ).toprettyxml(indent="  ")
    }

    additional_files = []
    # Mujoco Scene
    additional_files.append(os.path.join(get_package_share_directory("mujoco_ros2_control"), "mjcf", "scene.xml"))

    # Define the xacro2mjcf node
    xacro2mjcf = Node(
        package="mujoco_ros2_control",
        executable="xacro2mjcf.py",
        parameters=[
            {"robot_descriptions": [robot_description["robot_description"]]},
            {"input_files": additional_files},
            {"output_file": mujoco_model_file},
            {"mujoco_files_path": mujoco_model_path},
        ],
    )

    # Define the robot state publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        parameters=[robot_description],
    )

    # Path to the ros2 control parameters file
    ros2_control_params_file = os.path.join(
        get_package_share_directory("mujoco_ros2_control"),
        "test_data",
        "double_pendulum_controllers.yaml",
    )

    # Define the mujoco node
    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        namespace=namespace,
        #prefix=['gnome-terminal -- gdb -ex run --args'],
        parameters=[
            robot_description,
            ros2_control_params_file,
            {"simulation_frequency": 500.0},
            {"realtime_factor": 1.0},
            {"robot_model_path": mujoco_model_file},
            {"show_gui": True},
        ],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ]
    )

    # Register an event handler for when xacro2mjcf completes
    start_mujoco = RegisterEventHandler(
        OnProcessExit(
            target_action=xacro2mjcf,
            on_exit=[
                LogInfo(msg="Created mujoco xml, starting mujoco node..."),
                mujoco
            ],
        )
    )

    # Define the load_joint_state_broadcaster node
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            ["/", "controller_manager"],
        ],
    )

    joint_effort_controller = Node(package="controller_manager", executable="spawner", arguments=["joint_effort_controller", "--controller-manager", ["/", "controller_manager"]], namespace="/")

    # Register an event handler to start controllers once mujoco is up
    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="Starting joint state broadcaster..."),
                load_joint_state_broadcaster,
                joint_effort_controller
            ],
        )
    )

    # Return the nodes and handlers
    return [
        robot_state_publisher,
        xacro2mjcf,
        start_mujoco,
        load_controllers
    ]

@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        OpaqueFunction(function=create_nodes),
        launch.actions.TimerAction(
            period=5.0,
            actions=[
        launch_testing.actions.ReadyToTest()
            ]),
     ])

class TestNode(rclpy.node.Node):
    def __init__(self, name='mujoco_ros2_control_test_node'):
        super().__init__(name)

    def wait_for_node(self, node_name, timeout=10.0):
        start = time.time()
        while time.time() - start < timeout:
            if node_name in self.get_node_names():
                return True
        return False

    def wait_for_topic(self, topic_name, message_types, timeout=10.0):
        start = time.time()
        while time.time() - start < timeout:
            if (topic_name, message_types) in self.get_topic_names_and_types():
                return True
        return False

    def wait_for_message(self, topic_name, message_type, target_frame=None, timeout=10.0):
        msgs_rx = []
        sub = self.create_subscription(
            message_type, topic_name,
            lambda msg: msgs_rx.append(msg), 1)
        
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self)
            if msgs_rx:
                break

        print(msgs_rx)
        if msgs_rx:
            if target_frame is None or target_frame == msgs_rx[0].header.frame_id:
                return True
        return False

class TestBringup(unittest.TestCase):
    def test_node_start(self):
        rclpy.init()
        try:
            node = TestNode()
            # Test startup and ros2 control
            assert node.wait_for_node('mujoco_ros2_control'), 'mujoco_ros2_control Node not found !'
            assert node.wait_for_node('controller_manager'), 'controller_manager Node not found !'
            assert node.wait_for_node('joint_state_broadcaster'), 'joint_state_broadcaster Node not found !'
            assert node.wait_for_node('robot_state_publisher'), 'robot_state_publisher Node not found !'
            assert node.wait_for_node('joint_effort_controller'), 'effort controller Node not found !'
            # Test Sensors
            assert node.wait_for_node('link3_pose_sensor'), 'pose Node not found !'
            assert node.wait_for_node('link3_imu_sensor'), 'imu Node not found !'
            assert node.wait_for_node('link3_wrench_sensor'), 'wrench Node not found !'
            # Test Sensor Topics
            assert node.wait_for_topic('/link3_pose_sensor/pose', ['geometry_msgs/msg/PoseStamped']), 'pose topic not found !'
            assert node.wait_for_topic('/link3_imu_sensor/imu', ['sensor_msgs/msg/Imu']), 'imu topic not found !'
            assert node.wait_for_topic('/link3_wrench_sensor/wrench', ['geometry_msgs/msg/WrenchStamped']), 'wrench topic not found !'
            # Test Sensor Header
            assert node.wait_for_message('/link3_pose_sensor/pose', PoseStamped, 'world'), 'pose message not found !'
            assert node.wait_for_message('/link3_imu_sensor/imu', Imu, 'link3'), 'imu message not found !'
            assert node.wait_for_message('/link3_wrench_sensor/wrench', WrenchStamped, 'link3'), 'wrench message not found !'
        finally:
            rclpy.shutdown()