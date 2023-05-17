import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():

    namespace = ''

    robot_description_path = os.path.join(
        get_package_share_directory('panda_mujoco'),
        'config',
        'panda.urdf.xacro')
    
    robot_model_path = os.path.join(
    	get_package_share_directory('panda_mujoco'),
    	'config',
    	'panda.urdf')
    
    generate_urdf = ExecuteProcess(
        cmd=['xacro', robot_description_path, '>', robot_model_path],
        output='screen'
    )    
    
    
    robot_description = {'robot_description': xacro.process_file(robot_description_path).toxml()}

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
    	'controllers_effort.yaml')
    	
    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        name="mujoco_ros2_control",
        namespace=namespace,
        parameters=[
        {"robot_model_path": robot_model_path},
    	{"params_file_path": ros2_control_params_file}]
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

    return LaunchDescription(
        [
            robot_state_publisher,
            joint_trajectory_controller,
            gripper_trajectory_controller,
            #mujoco
        ]
    )
