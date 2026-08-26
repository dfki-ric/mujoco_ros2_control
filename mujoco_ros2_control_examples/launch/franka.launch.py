import os

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


def _pending_peg_tray_decompositions(examples_share):
    """Pick/insert tray meshes that still need CoaCD, in a fixed order.

    A tray that already carries its decomposed folder (from a previous run)
    is left alone -- coacd_node.py (and decompose_mesh() under it) skips it
    too, so this is only for deciding whether to run the node at all."""
    pegs_dir = os.path.join(examples_share, "meshes", "industreal", "pegs")
    pending = []
    for shape in ("round", "rectangular"):
        for size in (8, 12, 16):
            for kind in ("pick", "insert"):
                mesh = os.path.join(pegs_dir, f"industreal_tray_{kind}_{shape}_peg_{size}mm.obj")
                if os.path.isfile(mesh) and not os.path.isdir(os.path.splitext(mesh)[0]):
                    pending.append(mesh)
    return pending


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
            "load_realsense": value("load_realsense"),
            "render_realsense": value("render_realsense"),
            "realsense_xyz": value("realsense_xyz"),
            "realsense_rpy": value("realsense_rpy"),
            "base_xyz": value("robot_base_xyz"),
            "base_rpy": "0 0 0",
        },
    ).toprettyxml(indent="  ")
    robot_description = {"robot_description": robot_xml}

    # Non-robot models are passed through unchanged as independent scene files.
    additional_files = [os.path.join(mujoco_share, "mjcf", "scene.xml")]
    if value("load_imrk_table").lower() == "true":
        additional_files.append(os.path.join(
            examples_share, "urdf", "imrk_table", "imrk_table.urdf.xacro"
        ))
    pending_trays = []
    if value("load_industreal_board").lower() == "true":
        additional_files.append(os.path.join(
            examples_share, "urdf", "industreal", "industreal_task_board.urdf.xacro"
        ))
        if value("decompose_industreal_peg_trays").lower() == "true":
            pending_trays = _pending_peg_tray_decompositions(examples_share)

    converter = Node(
        package="mujoco_ros2_control",
        executable="xacro2mjcf.py",
        parameters=[{
            "robot_descriptions": [robot_xml],
            "input_files": additional_files,
            "xacro_args": [f"task_board_config:={value('task_board_config')}"],
            "output_file": model_file,
            "mujoco_files_path": model_dir,
        }],
    )

    controllers = os.path.join(examples_share, "config", "franka", "franka_controllers.yaml")
    realsense = value("camera_config")
    simulator = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            robot_description,
            controllers,
            realsense,
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

    # coacd_node.py takes the whole pending list in one process (like
    # xacro2mjcf.py takes input_files); converter starts once it exits, same
    # as when there is nothing to decompose and it is the very first action.
    if pending_trays:
        decompose = Node(
            package="mujoco_ros2_control",
            executable="coacd_node.py",
            parameters=[{"meshes": pending_trays}],
            output="screen",
        )
        first_action = decompose
        decompose_handlers = [RegisterEventHandler(
            OnProcessExit(target_action=decompose, on_exit=[converter]))]
    else:
        first_action = converter
        decompose_handlers = []

    return [publisher, first_action, *decompose_handlers, start_simulator, start_controllers]


def generate_launch_description():
    examples_share = get_package_share_directory("mujoco_ros2_control_examples")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("arm_id", default_value="fr3"),
        DeclareLaunchArgument("load_gripper", default_value="true"),
        DeclareLaunchArgument("load_realsense", default_value="true"),
        DeclareLaunchArgument("render_realsense", default_value="true"),
        DeclareLaunchArgument("realsense_xyz", default_value="0.025 0 0.0075"),
        DeclareLaunchArgument("realsense_rpy", default_value="0 0 0"),
        DeclareLaunchArgument("ee_id", default_value="franka_hand"),
        DeclareLaunchArgument("robot_base_xyz", default_value="-0.308 0 0.875"),
        DeclareLaunchArgument("load_imrk_table", default_value="true"),
        DeclareLaunchArgument("load_industreal_board", default_value="true"),
        DeclareLaunchArgument("decompose_industreal_peg_trays", default_value="false"),
        DeclareLaunchArgument("synchronous_mode", default_value="false"),
        DeclareLaunchArgument(
            "camera_config",
            default_value=os.path.join(
                examples_share, "config", "franka", "realsense_d435.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "task_board_config",
            default_value=os.path.join(
                examples_share, "config", "franka", "industreal_task_board.yaml"
            ),
        ),
        OpaqueFunction(function=create_nodes),
    ])
