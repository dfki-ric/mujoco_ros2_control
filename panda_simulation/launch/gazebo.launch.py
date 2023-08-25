import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                   OnProcessIO, OnProcessStart, OnShutdown)
import xacro


def generate_launch_description():
    namespace = ''
    command_interface = 'effort'

    robot_description_path = os.path.join(
        get_package_share_directory('panda_simulation'),
        'urdf',
        'panda.urdf.xacro')

    robot_description_string = xacro.process_file(robot_description_path, mappings={
        'ros2_control_command_interface': command_interface,
        'name': 'panda',
        'origin_xyz': '0 0 0.875',
        'origin_rpy': '0 0 0',
        'world_name': 'base_link',
        'generate_world_frame': 'false',
        'ros2_control_plugin': 'ign'
    }).toxml()

    robot_description = {'robot_description': robot_description_string}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        parameters=[robot_description]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),
                          'launch', 'gz_sim.launch.py')]),
        launch_arguments=[('gz_args', [' -r -v 6 empty.sdf --physics-engine ignition-physics-dartsim-plugin']), ('gz_version', ['6']), ])

    USB_Male = xacro.process_file(os.path.join(
        get_package_share_directory('panda_simulation'),
        'urdf',
        'USB_Male.urdf.xacro')).toxml()

    USB_Female = xacro.process_file(os.path.join(
        get_package_share_directory('panda_simulation'),
        'urdf',
        'USB_Female.urdf.xacro')).toxml()

    gz_spawn_entity_2 = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', USB_Male,
                   '-name', 'USB_Male',
                   '-allow_renaming', 'true']
    )

    gz_spawn_entity_3 = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', USB_Female,
                   '-name', 'USB_Female',
                   '-allow_renaming', 'true']
    )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description_string,
                   '-name', 'panda',
                   '-allow_renaming', 'true'],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'cartesian_impedance_controller'],
        output='screen'
    )

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_controller'],
        output='screen'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value="true",
                description='If true, use simulated clock'),
            bridge,
            gazebo,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=ignition_spawn_entity,
                    on_exit=[load_joint_state_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[load_arm_controller, load_gripper_controller]
                )
            ),
            robot_state_publisher,
            ignition_spawn_entity,
            gz_spawn_entity_2,
            gz_spawn_entity_3
        ]
    )
