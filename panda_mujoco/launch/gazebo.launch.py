import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
import launch_ros
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                   OnProcessIO, OnProcessStart, OnShutdown)

import xacro


def generate_launch_description():
    namespace = ''
    command_interface = 'effort'

    robot_description_path = os.path.join(
        get_package_share_directory('panda_mujoco'),
        'urdf',
        'panda.urdf.xacro')

    robot_description_string = xacro.process_file(robot_description_path, mappings={
        'ros2_control_command_interface': command_interface,
        'name': 'panda',
        'origin_xyz': '0 0 0',
        'origin_rpy': '0 0 0',
        'world_name': 'base_link',
        'generate_world_frame': 'false'
    }).toxml()

    robot_description = {'robot_description': robot_description_string}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        parameters=[robot_description]
    )

    ros2_control_params_file = os.path.join(
        get_package_share_directory('panda_mujoco'),
        'config',
        'controllers_joint_trajectory_controller.yaml')
    # 'controllers_effort.yaml')

    # Gazebo Sim
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    spawn_model = Node(package='ros_gz_sim', executable='create',
                       arguments=[
                           '-name', 'panda',
                           '-topic', '/robot_description'],
                       output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_controller'],
        output='screen'
    )

    # RViz
    rviz_config_file = (
            get_package_share_directory("panda_mujoco") + "/rviz/panda.rviz"
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # output="log",
        arguments=["-d", rviz_config_file],
        parameters=[]
    )

    rqt_joint_trajectory_controller = Node(
        package="rqt_joint_trajectory_controller",
        executable="rqt_joint_trajectory_controller",
        namespace=namespace
    )

    return LaunchDescription(
        [
            gazebo,
            spawn_model,
            robot_state_publisher,
            #load_joint_state_controller,
            #load_arm_controller,
            #load_gripper_controller
        ]
    )
