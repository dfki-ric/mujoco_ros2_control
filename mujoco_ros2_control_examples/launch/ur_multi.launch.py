"""Launch several Universal Robots arms standing in a row in one MuJoCo world.

By default the row is a ur3e, a ur5e and a ur10e - three different arms rather
than three copies of one. `ur_types` lists one UR type per arm in row order, and
its length sets how many arms there are.

One description holds every arm (urdf/ur/ur_multi.urdf.xacro), so there is a
single robot_description, a single MJCF and a single controller manager. Each arm
sits under a tf_prefix named after its own type - ur3e_, ur5e_, ur10e_ - and gets
its own <ros2_control> hardware component, joint trajectory controller and wrist
force/torque broadcaster under that prefix, so every name says which robot it
belongs to. The joint state broadcaster is shared: one instance publishes every
joint in the model.

The prefixes are read back out of the generated description rather than recomputed
here, so the xacro stays their single definition (it is the one that resolves a
repeated type into ur5e_, ur5e_2_, ...).

Because the controller names follow the arm types, config/ur/ur_multi_controllers.yaml
matches the default `ur_types`. Another list needs matching entries there.
"""

import os
import xml.etree.ElementTree as ET

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

# ur_multi.urdf.xacro names each arm's hardware component "<tf_prefix>UrMujocoSystem".
HARDWARE_SUFFIX = "UrMujocoSystem"


def create_nodes(context: LaunchContext):
    namespace = ""
    mujoco_model_path = "/tmp/mujoco"
    mujoco_model_file = os.path.join(mujoco_model_path, "main.xml")

    rviz = LaunchConfiguration("rviz")
    ur_types = context.perform_substitution(LaunchConfiguration("ur_types"))
    spacing = context.perform_substitution(LaunchConfiguration("spacing"))
    types = [t.strip() for t in ur_types.split(",") if t.strip()]

    ur_multi_xacro_filepath = os.path.join(
        get_package_share_directory("mujoco_ros2_control_examples"),
        "urdf",
        "ur",
        "ur_multi.urdf.xacro",
    )
    robot_description = {
        "robot_description": xacro.process_file(
            ur_multi_xacro_filepath,
            mappings={
                "name": "ur_multi",
                "ur_types": ",".join(types),
                "spacing": spacing,
                "mujoco": "true",
                "mujoco_model_path": mujoco_model_path,
            },
        ).toprettyxml(indent="  ")
    }

    # Ask the description which prefixes it actually used, instead of deriving
    # them a second time: ur_multi.urdf.xacro names each hardware component
    # "<tf_prefix>UrMujocoSystem", and it alone decides how a repeated type is
    # disambiguated.
    prefixes = [
        block.get("name")[: -len(HARDWARE_SUFFIX)]
        for block in ET.fromstring(robot_description["robot_description"])
        .findall("ros2_control")
        if block.get("name", "").endswith(HARDWARE_SUFFIX)
    ]

    additional_files = [
        os.path.join(
            get_package_share_directory("mujoco_ros2_control"), "mjcf", "scene.xml"
        )
    ]

    # Every arm is in the one description, so this is a single-element list -
    # xacro2mjcf's multi-description support is for separately authored models.
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
        "ur_multi_controllers.yaml",
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

    start_mujoco = RegisterEventHandler(
        OnProcessExit(
            target_action=xacro2mjcf,
            on_exit=[
                LogInfo(msg="Created mujoco xml, starting mujoco node..."),
                mujoco,
            ],
        )
    )

    def spawner(controller, with_params=True):
        arguments = [
            controller,
            "--controller-manager", ["/", "controller_manager"],
        ]
        if with_params:
            arguments += ["--param-file", ros2_control_params_file]
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=arguments,
        )

    # One shared joint state broadcaster, then a trajectory controller and a
    # force/torque broadcaster per arm, named after that arm.
    controllers = [spawner("joint_state_broadcaster", with_params=False)]
    for prefix in prefixes:
        controllers.append(spawner(f"{prefix}ft_sensor_broadcaster"))
        controllers.append(spawner(f"{prefix}joint_trajectory_controller"))

    rviz_node = Node(
        condition=IfCondition(rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[{"use_sim_time": True}],
    )

    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg=f"Starting controllers for {len(prefixes)} arms "
                            f"({', '.join(p.rstrip('_') for p in prefixes)})..."),
                *controllers,
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
                "ur_types",
                default_value="ur3e,ur5e,ur10e",
                description="One UR type per arm, in row order, comma separated "
                "(ur3, ur5, ur10, ur3e, ur5e, ur7e, ur10e, ur12e, ur16e, ur20, "
                "ur30, ...). The number of entries is the number of arms; more "
                "than three needs extra controller entries in "
                "config/ur/ur_multi_controllers.yaml.",
            ),
            DeclareLaunchArgument(
                "spacing",
                default_value="1.5",
                description="Distance between neighbouring arm bases along y, m.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz.",
            ),
            OpaqueFunction(function=create_nodes),
        ]
    )
