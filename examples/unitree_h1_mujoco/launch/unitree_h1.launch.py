import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction,
    ExecuteLocal
)
from launch.event_handlers import OnExecutionComplete, OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import xacro

# Define your namespace and file paths


def create_nodes(context: LaunchContext):
    namespace = ""
    mujoco_model_path = "/tmp/mujoco"
    mujoco_model_file = os.path.join(mujoco_model_path, "main.xml")

    # Fetch launch configurations
    rviz = LaunchConfiguration("rviz")

    # Set file paths
    unitree_h1_xacro_filepath = os.path.join(
        get_package_share_directory("unitree_h1_mujoco"),
        "urdf",
        "unitree_h1.urdf.xacro",
    )

    # Process the xacro file and create the robot description
    robot_description = {
        'robot_description': xacro.process_file(
            unitree_h1_xacro_filepath,
            mappings={
                "name": "unitree_h1",
                "mujoco": "true",
                "mujoco_effort": "false"
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
            {"base_link": "pelvis"},
            {"floating": True},
            {"initial_position": "0 0 1.05"},
            {"initial_orientation": "0 0 0"}
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
        get_package_share_directory("unitree_h1_mujoco"),
        "config",
        "unitree_h1_controllers.yaml",
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
            '--param-file',
            ros2_control_params_file,
        ],
    )

    left_arm_position_controller = Node(package="controller_manager", executable="spawner", arguments=["left_arm_position_controller", "--controller-manager", ["/", "controller_manager"], '--param-file', ros2_control_params_file], namespace="/")
    left_hand_position_controller = Node(package="controller_manager", executable="spawner", arguments=["left_hand_position_controller", "--controller-manager", ["/", "controller_manager"], '--param-file', ros2_control_params_file], namespace="/")
    left_leg_position_controller = Node(package="controller_manager", executable="spawner", arguments=["left_leg_position_controller", "--controller-manager", ["/", "controller_manager"], '--param-file', ros2_control_params_file], namespace="/")
    right_arm_position_controller = Node(package="controller_manager", executable="spawner", arguments=["right_arm_position_controller", "--controller-manager", ["/", "controller_manager"], '--param-file', ros2_control_params_file], namespace="/")
    right_hand_position_controller = Node(package="controller_manager", executable="spawner", arguments=["right_hand_position_controller", "--controller-manager", ["/", "controller_manager"], '--param-file', ros2_control_params_file], namespace="/")
    right_leg_position_controller = Node(package="controller_manager", executable="spawner", arguments=["right_leg_position_controller", "--controller-manager", ["/", "controller_manager"], '--param-file', ros2_control_params_file], namespace="/")
    # load_hand_pose_left_controller = Node(package="controller_manager", executable="spawner", arguments=["hand_pose_left_controller", "--controller-manager", ["/", "controller_manager"]], namespace="/")
    # load_hand_pose_right_controller = Node(package="controller_manager", executable="spawner", arguments=["hand_pose_right_controller", "--controller-manager", ["/", "controller_manager"]], namespace="/")
    # foot_pose_left_controller = Node(package="controller_manager", executable="spawner", arguments=["foot_pose_left_controller", "--controller-manager", ["/", "controller_manager"]], namespace="/")
    # foot_pose_right_controller = Node(package="controller_manager", executable="spawner", arguments=["foot_pose_right_controller", "--controller-manager", ["/", "controller_manager"]], namespace="/")

    rviz_config_file = os.path.join(
        get_package_share_directory("unitree_h1_mujoco"),
        "config",
        "default.rviz",
    )
    
    rviz_node = Node(
        condition=IfCondition(rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            {"use_sim_time": True},
        ],
    )

    # Register an event handler to start controllers once mujoco is up
    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="Starting joint state broadcaster..."),
                load_joint_state_broadcaster,
                left_arm_position_controller,
                left_hand_position_controller,
                left_leg_position_controller,
                right_arm_position_controller,
                right_hand_position_controller,
                right_leg_position_controller,
                # load_wbc_controller,
                # load_hand_pose_left_controller,
                # load_hand_pose_right_controller,
                # foot_pose_left_controller,
                # foot_pose_right_controller,
                rviz_node
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
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start rviz. "
        ),
        OpaqueFunction(function=create_nodes)  # Use OpaqueFunction for node creation
    ])