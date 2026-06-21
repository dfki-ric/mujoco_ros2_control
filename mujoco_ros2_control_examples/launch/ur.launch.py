import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import xacro


def create_nodes(context: LaunchContext):
    namespace = ""
    mujoco_model_path = "/tmp/mujoco"
    mujoco_model_file = os.path.join(mujoco_model_path, "main.xml")

    # Fetch launch configurations
    rviz = LaunchConfiguration("rviz")
    ur_type = context.perform_substitution(LaunchConfiguration("ur_type"))

    # Build the robot description from this package's UR + MuJoCo xacro
    ur_xacro_filepath = os.path.join(
        get_package_share_directory("mujoco_ros2_control_examples"),
        "urdf",
        "ur",
        "ur.urdf.xacro",
    )
    robot_description = {
        "robot_description": xacro.process_file(
            ur_xacro_filepath,
            mappings={
                "name": "ur",
                "ur_type": ur_type,
                "mujoco": "true",
                "mujoco_model_path": mujoco_model_path,
            },
        ).toprettyxml(indent="  ")
    }

    # Additional MuJoCo files: the shared ground/scene
    additional_files = [
        os.path.join(
            get_package_share_directory("mujoco_ros2_control"), "mjcf", "scene.xml"
        )
    ]

    # Convert the xacro/URDF into a MuJoCo MJCF model
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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        parameters=[robot_description],
    )

    ros2_control_params_file = os.path.join(
        get_package_share_directory("mujoco_ros2_control_examples"),
        "config",
        "ur",
        "ur_controllers.yaml",
    )

    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        namespace=namespace,
        parameters=[
            robot_description,
            ros2_control_params_file,
            {"simulation_frequency": 500.0},
            {"realtime_factor": 1.0},
            {"robot_model_path": mujoco_model_file},
            {"show_gui": True},
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    # Start mujoco once the model has been generated
    start_mujoco = RegisterEventHandler(
        OnProcessExit(
            target_action=xacro2mjcf,
            on_exit=[
                LogInfo(msg="Created mujoco xml, starting mujoco node..."),
                mujoco,
            ],
        )
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            ["/", "controller_manager"],
        ],
    )

    ft_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ft_sensor_broadcaster",
            "--controller-manager",
            ["/", "controller_manager"],
            "--param-file",
            ros2_control_params_file,
        ],
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            ["/", "controller_manager"],
            "--param-file",
            ros2_control_params_file,
        ],
        namespace="/",
    )

    rviz_node = Node(
        condition=IfCondition(rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    # Start controllers once mujoco is up
    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="Starting controllers..."),
                load_joint_state_broadcaster,
                ft_sensor_broadcaster,
                joint_trajectory_controller,
                rviz_node,
            ],
        )
    )

    return [
        robot_state_publisher,
        xacro2mjcf,
        start_mujoco,
        load_controllers,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur5e",
                description="UR robot model: ur3, ur5, ur10, ur3e, ur5e, ur7e, "
                "ur10e, ur12e, ur16e, ur20, ur30, ...",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz.",
            ),
            OpaqueFunction(function=create_nodes),
        ]
    )
