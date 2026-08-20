import os
import re

import xacro
from ament_index_python import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
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
    model_dir = "/tmp/mujoco"
    model_file = os.path.join(model_dir, "main.xml")

    def value(name):
        return context.perform_substitution(LaunchConfiguration(name))

    arm_id = value("arm_id")
    ee_id = value("ee_id")
    headless = value("headless")
    headless_bool = headless.lower() == "true"
    load_task_table = value("load_task_table").lower() == "true"

    franka_xacro = os.path.join(examples_share, "urdf", "franka", "franka.urdf.xacro")
    robot_xml = xacro.process_file(
        franka_xacro,
        mappings={
            "name": "franka",
            "mujoco": "true",
            "arm_id": arm_id,
            "hand": value("load_gripper"),
            "ee_id": ee_id,
            "headless": headless,
        },
    ).toprettyxml(indent="  ")

    # Non-robot models are passed through unchanged as independent scene files.
    additional_files = [os.path.join(mujoco_share, "mjcf", "scene.xml")]
    if load_task_table:
        task_table_xacro = os.path.join(
            examples_share, "urdf", "task_table", "task_table.urdf.xacro"
        )
        additional_files.append(task_table_xacro)
        # Merge the task table's ros2_control blocks into robot_description so
        # the gear pose sensors are registered with the controller manager.
        task_table_xml = xacro.process_file(task_table_xacro).toprettyxml(indent="  ")
        for block in re.findall(r"(<ros2_control.*?</ros2_control>)", task_table_xml, re.DOTALL):
            robot_xml = robot_xml.replace("</robot>", block + "\n</robot>")
    robot_description = {"robot_description": robot_xml}

    converter = Node(
        package="mujoco_ros2_control",
        executable="xacro2mjcf.py",
        parameters=[{
            "robot_descriptions": [robot_xml],
            "input_files": additional_files,
            "output_file": model_file,
            "mujoco_files_path": model_dir,
        }],
    )

    controllers = os.path.join(examples_share, "config", "franka", "franka_controllers.yaml")
    simulator = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            robot_description,
            controllers,
            {
                "simulation_frequency": 500.0,
                "realtime_factor": 1.0,
                "synchronous_mode": value("synchronous_mode").lower() == "true",
            },
            {"robot_model_path": model_file, "show_gui": not headless_bool},
        ],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
    )

    controller_nodes = [
        _spawner("joint_state_broadcaster"),
        _spawner("ft_sensor_broadcaster", controllers),
        _spawner(f"{arm_id}_joint_trajectory_controller", controllers),
    ]
    if ee_id == "franka_hand":
        controller_nodes.append(
            _spawner(f"{arm_id}_franka_hand_joint_trajectory_controller", controllers)
        )
    if load_task_table:
        controller_nodes += [
            _spawner(f"{gear}_pose_broadcaster", controllers)
            for gear in ("gears_large", "gears_medium", "gears_small")
        ]

    rviz = Node(
        condition=IfCondition(LaunchConfiguration("rviz")),
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", os.path.join(examples_share, "config", "franka", "default.rviz")],
        parameters=[{"use_sim_time": True}],
    )
    publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    start_simulator = RegisterEventHandler(OnProcessExit(
        target_action=converter,
        on_exit=[LogInfo(msg="Created MuJoCo model; starting simulator"), simulator],
    ))
    start_controllers = RegisterEventHandler(OnProcessStart(
        target_action=simulator,
        on_start=[*controller_nodes, rviz],
    ))
    return [publisher, converter, start_simulator, start_controllers]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("arm_id", default_value="fr3"),
        DeclareLaunchArgument("load_gripper", default_value="true"),
        DeclareLaunchArgument("ee_id", default_value="franka_hand"),
        DeclareLaunchArgument("load_task_table", default_value="true"),
        DeclareLaunchArgument("synchronous_mode", default_value="false"),
        OpaqueFunction(function=create_nodes),
    ])
