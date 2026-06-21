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

import re
import xacro

# Define your namespace and file paths


def create_nodes(context: LaunchContext):
    namespace = ""
    mujoco_model_path = "/tmp/mujoco"
    mujoco_model_file = os.path.join(mujoco_model_path, "main.xml")

    # Fetch launch configurations
    rviz = LaunchConfiguration("rviz")
    load_task_table = LaunchConfiguration("load_task_table")
    load_gripper = LaunchConfiguration("load_gripper")
    ee_id = LaunchConfiguration("ee_id")
    arm_id = LaunchConfiguration("arm_id")

    # Perform substitutions to get actual values
    load_task_table_bool = context.perform_substitution(load_task_table).lower() == "true"
    load_gripper_str = context.perform_substitution(load_gripper)
    ee_id_str = context.perform_substitution(ee_id)
    arm_id_str = context.perform_substitution(arm_id)

    # Set file paths
    franka_xacro_filepath = os.path.join(
        get_package_share_directory("mujoco_ros2_control_examples"),
        "urdf",
        "franka",
        "franka.urdf.xacro",
    )

    # Process the xacro file and create the robot description
    robot_description = {
        'robot_description': xacro.process_file(
            franka_xacro_filepath,
            mappings={
                "name": "franka",
                "mujoco": "true",
                "arm_id": arm_id_str,
                "hand": load_gripper_str,
                "ee_id": ee_id_str
            }
        ).toprettyxml(indent="  ")
    }

    additional_files = []
    # Mujoco Scene
    additional_files.append(os.path.join(get_package_share_directory("mujoco_ros2_control"), "mjcf", "scene.xml"))
    if load_task_table_bool:
        # Some gears with position and orientation sensors
        task_table_xacro = os.path.join(get_package_share_directory("mujoco_ros2_control_examples"), "urdf", "task_table", "task_table.urdf.xacro")
        additional_files.append(task_table_xacro)

        # Merge task_table ros2_control blocks into robot_description
        # so the gear pose sensors are registered with the controller manager
        task_table_urdf = xacro.process_file(task_table_xacro).toprettyxml(indent="  ")
        ros2_control_blocks = re.findall(
            r'(<ros2_control.*?</ros2_control>)', task_table_urdf, re.DOTALL)
        franka_urdf = robot_description['robot_description']
        for block in ros2_control_blocks:
            franka_urdf = franka_urdf.replace('</robot>', block + '\n</robot>')
        robot_description['robot_description'] = franka_urdf

    # Define the xacro2mjcf node
    xacro2mjcf = Node(
        package="mujoco_ros2_control",
        executable="xacro2mjcf.py",
        parameters=[
            {"robot_descriptions": [robot_description["robot_description"]]},
            {"input_files": additional_files},
            {"output_file": mujoco_model_file},
            {"mujoco_files_path": mujoco_model_path}
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
        get_package_share_directory("mujoco_ros2_control_examples"),
        "config",
        "franka",
        "franka_controllers.yaml",
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

    ft_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ft_sensor_broadcaster",
            "--controller-manager",
            ["/", "controller_manager"],
            '--param-file',
            ros2_control_params_file,
        ],
    )

    franka_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            arm_id_str + "_joint_trajectory_controller",
            "--controller-manager",
            ["/", "controller_manager"],
            '--param-file',
            ros2_control_params_file,
        ],
        namespace="/",
    )

    gripper_joint_trajectory_controller = Node(
        condition=IfCondition([str(ee_id_str == "franka_hand")]),
        package="controller_manager",
        executable="spawner",
        arguments=[
            arm_id_str + "_franka_hand_joint_trajectory_controller",
            "--controller-manager",
            ["/", "controller_manager"],
            '--param-file',
            ros2_control_params_file,
        ],
        namespace="/",
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("mujoco_ros2_control_examples"),
        "config",
        "franka",
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

    # Gear pose broadcasters (only when task table is loaded)
    gear_broadcasters = []
    if load_task_table_bool:
        for gear_name in ["gears_large", "gears_medium", "gears_small"]:
            gear_broadcasters.append(Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    gear_name + "_pose_broadcaster",
                    "--controller-manager",
                    ["/", "controller_manager"],
                    '--param-file',
                    ros2_control_params_file,
                ],
            ))

    # Register an event handler to start controllers once mujoco is up
    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="Starting joint state broadcaster..."),
                load_joint_state_broadcaster,
                ft_sensor_broadcaster,
                franka_joint_trajectory_controller,
                gripper_joint_trajectory_controller,
                *gear_broadcasters,
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
        DeclareLaunchArgument(
            "load_task_table",
            default_value="true",
            description="Load multiple high resolution collision objects with pose sensors. "
                        "Scene is loaded only with robot otherwise",
        ),
        DeclareLaunchArgument(
            "load_gripper",
            default_value="true",
            description="Use end-effector if true. Default value is franka hand. "
                        "Robot is loaded without end-effector otherwise",
        ),
        DeclareLaunchArgument(
            "ee_id",
            default_value="franka_hand",
            description="ID of the type of end-effector used. Supported values: "
                        "none, franka_hand, cobot_pump",
        ),
        DeclareLaunchArgument(
            "arm_id",
            default_value="fr3",
            description="ID of the type of arm used. Supported values: "
                        "fer, fr3, fp3",
        ),
        OpaqueFunction(function=create_nodes)  # Use OpaqueFunction for node creation
    ])