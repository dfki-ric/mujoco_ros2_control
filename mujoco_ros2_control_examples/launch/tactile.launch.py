"""Tactile example: a pad on the ground plane with a MuJoCo touch_grid sensor.

Demonstrates a MujocoSensorInterface plugin that lives outside
mujoco_ros2_control. The grid is published rather than exported as state
interfaces, because its nchannel * width * height output does not fit
ros2_control's scalar interface model.

    ros2 launch mujoco_ros2_control_examples tactile.launch.py
    ros2 topic echo /pad_touch/touch_grid
"""

import os

import xacro
from ament_index_python import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawner(name, params_file=None):
    arguments = [name, "--controller-manager", "/controller_manager"]
    if params_file:
        arguments += ["--param-file", params_file]
    return Node(package="controller_manager", executable="spawner", arguments=arguments)


def create_nodes(context: LaunchContext):
    examples_share = get_package_share_directory("mujoco_ros2_control_examples")
    mujoco_share = get_package_share_directory("mujoco_ros2_control")
    model_dir = "/tmp/mujoco_tactile"
    model_file = os.path.join(model_dir, "main.xml")

    def value(name):
        return context.perform_substitution(LaunchConfiguration(name))

    headless_bool = value("headless").lower() == "true"

    robot_xml = xacro.process_file(
        os.path.join(examples_share, "urdf", "tactile", "tactile_pad.urdf.xacro")
    ).toprettyxml(indent="  ")
    robot_description = {"robot_description": robot_xml}

    converter = Node(
        package="mujoco_ros2_control",
        executable="xacro2mjcf.py",
        parameters=[{
            "robot_descriptions": [robot_xml],
            "input_files": [os.path.join(mujoco_share, "mjcf", "scene.xml")],
            "output_file": model_file,
            "mujoco_files_path": model_dir,
        }],
    )

    controllers = os.path.join(
        examples_share, "config", "tactile", "tactile_controllers.yaml")

    simulator = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            robot_description,
            controllers,
            {"simulation_frequency": 200.0, "real_time_factor": 1.0},
            {"robot_model_path": model_file, "show_gui": not headless_bool},
        ],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
    )

    publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    start_simulator = RegisterEventHandler(OnProcessExit(
        target_action=converter,
        on_exit=[LogInfo(msg="Created MuJoCo model; starting simulator"), simulator],
    ))
    start_controllers = RegisterEventHandler(OnProcessStart(
        target_action=simulator,
        on_start=[_spawner("joint_state_broadcaster")],
    ))
    return [publisher, converter, start_simulator, start_controllers]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="Run without the MuJoCo viewer window."),
        OpaqueFunction(function=create_nodes),
    ])
