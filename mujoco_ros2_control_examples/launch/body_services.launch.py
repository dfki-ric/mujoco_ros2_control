r"""Body services example: read and teleport bodies in a running simulation.

Brings up two probe bodies and the BodyServices plugin this package ships, twice
-- once on the default service names and once on names of its own.

    ros2 launch mujoco_ros2_control_examples body_services.launch.py

    ros2 service call /mujoco_get_body_state \
        mujoco_ros2_control_examples/srv/GetBodyState "{body_name: probe_float}"

    ros2 service call /mujoco_set_body_pose \
        mujoco_ros2_control_examples/srv/SetBodyPose \
        "{body_name: probe_float, x: 0.5, y: -0.25, z: 1.75, qw: 1.0}"

    # the same simulation, through the second declaration
    ros2 service call /alt/get_body_state \
        mujoco_ros2_control_examples/srv/GetBodyState "{body_name: probe_float}"

probe_fixed is welded to the world, so setting its pose fails -- deliberately;
only a body whose sole joint is free can be teleported.

`synchronous:=true` starts the simulation paused, advancing only on
/mujoco_step_simulation. That is the mode these services exist for: a client sets
a pose and reads it straight back with no step in between, which works because
the write happens in the service callback rather than being queued for the next
step.
"""

import os

import xacro
from ament_index_python import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawner(name, params_file):
    # The per-controller ros__parameters blocks reach the controller only through
    # --param-file; the controller manager itself reads no more than the type out
    # of the file it was given.
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[name, "--controller-manager", "/controller_manager",
                   "--param-file", params_file],
    )


def create_nodes(context: LaunchContext):
    examples_share = get_package_share_directory("mujoco_ros2_control_examples")
    mujoco_share = get_package_share_directory("mujoco_ros2_control")
    model_dir = "/tmp/mujoco_body_services"
    model_file = os.path.join(model_dir, "main.xml")

    def value(name):
        return context.perform_substitution(LaunchConfiguration(name)).lower() == "true"

    headless = value("headless")
    synchronous = value("synchronous")

    robot_xml = xacro.process_file(
        os.path.join(examples_share, "urdf", "body_services", "probe_bodies.urdf.xacro")
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
        examples_share, "config", "body_services", "body_services_controllers.yaml")

    simulator = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            robot_description,
            controllers,
            {"simulation_frequency": 100.0, "real_time_factor": 1.0},
            {"synchronous_mode": synchronous},
            {"robot_model_path": model_file, "show_gui": not headless},
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
        on_start=[_spawner("probe_fixed_pose_broadcaster", controllers),
                  _spawner("probe_float_pose_broadcaster", controllers)],
    ))
    return [publisher, converter, start_simulator, start_controllers]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="Run without the MuJoCo viewer window."),
        DeclareLaunchArgument(
            "synchronous", default_value="false",
            description="Start paused; advance only on /mujoco_step_simulation."),
        OpaqueFunction(function=create_nodes),
    ])
