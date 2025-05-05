#!/bin/env python3

import os
import time
import unittest
from launch import LaunchContext, LaunchDescription
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

def generate_launch_description():

    return LaunchDescription([
        OpaqueFunction(function=create_nodes)  # Use OpaqueFunction for node creation
    ])