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

    robot_model_path = "/tmp/panda.xml"

    xacro2mjcf_path = os.path.join(
        get_package_share_directory('mujoco_ros2_control'),
        'scripts',
        'xacro2mjcf.py')

    xacro2mjcf = ExecuteProcess(
        cmd=[xacro2mjcf_path, robot_description_path, robot_model_path],
        output='screen'
    )

    robot_description = {'robot_description': xacro.process_file(robot_description_path, mappings={
        'ros2_control_command_interface': command_interface}).toxml()}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        parameters=[robot_description]
    )

    ros2_control_params_file = os.path.join(
        get_package_share_directory('panda_moveit_config'),
        'config',
        'controllers_' + command_interface + '.yaml')

    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        namespace=namespace,
        respawn=True,
        parameters=[
            {"robot_model_path": robot_model_path},
            {"params_file_path": ros2_control_params_file}]
    )

    start_mujoco = RegisterEventHandler(
        OnExecutionComplete(
            target_action=xacro2mjcf,
            on_completion=[
                LogInfo(msg='Created mujoco xml'),
                mujoco
            ]
        )
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )

    gripper_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_trajectory_controller'],
        output='screen'
    )

    register_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg='Turtlesim started, spawning turtle'),
                load_joint_state_controller,
                joint_trajectory_controller,
                gripper_trajectory_controller
            ]
        )
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

    start_rviz = RegisterEventHandler(
        OnExecutionComplete(
            target_action=load_joint_state_controller,
            on_completion=[
                LogInfo(msg='Created mujoco xml'),
                rviz_node,
                #rqt_joint_trajectory_controller
            ]
        )
    )

    return LaunchDescription(
        [
            start_mujoco,
            register_controllers,
            start_rviz,
            xacro2mjcf,
            robot_state_publisher,
            #rqt_joint_trajectory_controller
        ]
    )
