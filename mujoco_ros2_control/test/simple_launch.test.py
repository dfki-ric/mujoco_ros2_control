#!/bin/env python3

from typing import List
import os
import time
import unittest
from launch import LaunchContext
import launch.actions
from launch_ros.actions import Node
import launch_testing.actions
from launch.substitutions import PathJoinSubstitution,Command,FindExecutable
from launch_ros.substitutions import FindPackageShare
import pytest
import rclpy
import rclpy.node
from ament_index_python import get_package_share_directory

from launch.actions import (
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction
)

from launch.event_handlers import (
    OnProcessStart,
    OnProcessExit
    )
from sensor_msgs.msg import Imu, Image, CameraInfo, PointCloud2, LaserScan, JointState
from geometry_msgs.msg import WrenchStamped, PoseStamped
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock

import xacro


def opengl_enabled():
    """Headless toggle. Set DISABLE_OPENGL=1 in the env to skip GL sensors."""
    return os.environ.get("DISABLE_OPENGL", "0") != "1"

def create_nodes(context: LaunchContext):
    namespace = ""
    mujoco_model_path = "/tmp/mujoco"
    mujoco_model_file = os.path.join(mujoco_model_path, "main.xml")

    # Set file paths
    double_pendulum_xacro_filepath = os.path.join(
        get_package_share_directory("mujoco_ros2_control"),
        "test_data",
        "double_pendulum.urdf.xacro",
    )

    # Process the xacro file and create the robot description
    robot_description = {
        'robot_description': xacro.process_file(
            double_pendulum_xacro_filepath,
            mappings={
                "command_mode": "effort",
                "with_visual_sensors": "true" if opengl_enabled() else "false",
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
        get_package_share_directory("mujoco_ros2_control"),
        "test_data",
        "double_pendulum_controllers.yaml",
    )

    # Path to the lidar params file
    lidar_params_file = os.path.join(
        get_package_share_directory("mujoco_ros2_control"),
        "test_data",
        "double_pendulum_lidar_params.yaml",
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
            lidar_params_file,
            {"simulation_frequency": 100.0},
            {"realtime_factor": 1.0},
            {"robot_model_path": mujoco_model_file},
            {"show_gui": False},
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

    joint_effort_controller = Node(
        package="controller_manager",
        executable="spawner", 
        arguments=["joint_effort_controller", "--controller-manager", ["/", "controller_manager"], "--param-file", ros2_control_params_file], 
        namespace="/")
    # Define the sensor broadcaster nodes
    link3_imu_sensor = Node(
        package="controller_manager", executable="spawner",
        arguments=["link3_imu_sensor", "--controller-manager", ["/", "controller_manager"], "--param-file", ros2_control_params_file],
        namespace="/")
    link3_wrench_sensor = Node(
        package="controller_manager", executable="spawner",
        arguments=["link3_wrench_sensor", "--controller-manager", ["/", "controller_manager"], "--param-file", ros2_control_params_file],
        namespace="/")
    link3_pose_sensor = Node(
        package="controller_manager", executable="spawner",
        arguments=["link3_pose_sensor", "--controller-manager", ["/", "controller_manager"], "--param-file", ros2_control_params_file],
        namespace="/")

    # Register an event handler to start controllers once mujoco is up
    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="Starting joint state broadcaster..."),
                load_joint_state_broadcaster,
                joint_effort_controller,
                link3_imu_sensor,
                link3_wrench_sensor,
                link3_pose_sensor,
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

@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        OpaqueFunction(function=create_nodes),
        launch.actions.TimerAction(
            period=5.0,
            actions=[
        launch_testing.actions.ReadyToTest()
            ]),
     ])

class TestNode(rclpy.node.Node):
    def __init__(self, name='mujoco_ros2_control_test_node'):
        super().__init__(name)

    def wait_for_node(self, node_name, timeout=10.0):
        start = time.time()
        while time.time() - start < timeout:
            if node_name in self.get_node_names():
                return True
        return False

    def wait_for_topic(self, topic_name, message_types, timeout=10.0):
        start = time.time()
        while time.time() - start < timeout:
            if (topic_name, message_types) in self.get_topic_names_and_types():
                return True
        return False

    def wait_for_message(self, topic_name, message_type, target_frame=None, timeout=10.0):
        msgs_rx = []
        sub = self.create_subscription(
            message_type, topic_name,
            lambda msg: msgs_rx.append(msg), 1)
        
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self)
            if msgs_rx:
                break
        if msgs_rx:
            if target_frame is None or target_frame == msgs_rx[0].header.frame_id:
                return True
        return False

class TestBringup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = TestNode()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_bringup_nodes(self):
        node = self.node
        assert node.wait_for_node('mujoco_ros2_control'), 'mujoco_ros2_control Node not found !'
        assert node.wait_for_node('controller_manager'), 'controller_manager Node not found !'
        assert node.wait_for_node('joint_state_broadcaster'), 'joint_state_broadcaster Node not found !'
        assert node.wait_for_node('robot_state_publisher'), 'robot_state_publisher Node not found !'
        assert node.wait_for_node('joint_effort_controller'), 'effort controller Node not found !'

    def test_default_topics(self):
        node = self.node
        assert node.wait_for_topic('/tf', ['tf2_msgs/msg/TFMessage']), 'TF topic not found !'
        assert node.wait_for_message('/tf', TFMessage, timeout=5.0), 'TF message not found !'

        assert node.wait_for_topic('/tf_static', ['tf2_msgs/msg/TFMessage']), 'TF static topic not found !'
        # assert node.wait_for_message('/tf_static', TFMessage, timeout=15.0), 'TF static message not found !'

        assert node.wait_for_topic('/joint_states', ['sensor_msgs/msg/JointState']), 'joint_states topic not found !'
        assert node.wait_for_message('/joint_states', JointState, timeout=5.0), 'joint_states message not found !'

        assert node.wait_for_topic('/clock', ['rosgraph_msgs/msg/Clock']), 'clock topic not found !'
        assert node.wait_for_message('/clock', Clock, timeout=5.0), 'clock message not found !'

        assert node.wait_for_topic('/robot_description', ['std_msgs/msg/String']), 'robot_description topic not found !'
        # assert node.wait_for_message('/robot_description', String, timeout=15.0), 'robot_description message not found !'


    def test_pose_sensor(self):
        node = self.node
        assert node.wait_for_node('link3_pose_sensor'), 'pose Node not found !'
        assert node.wait_for_topic('/link3_pose_sensor/pose', ['geometry_msgs/msg/PoseStamped']), 'pose topic not found !'
        assert node.wait_for_message('/link3_pose_sensor/pose', PoseStamped, 'world'), 'pose message not found !'

    def test_imu_sensor(self):
        node = self.node
        assert node.wait_for_node('link3_imu_sensor'), 'imu Node not found !'
        assert node.wait_for_topic('/link3_imu_sensor/imu', ['sensor_msgs/msg/Imu']), 'imu topic not found !'
        assert node.wait_for_message('/link3_imu_sensor/imu', Imu, 'link3'), 'imu message not found !'

    def test_wrench_sensor(self):
        node = self.node
        assert node.wait_for_node('link3_wrench_sensor'), 'wrench Node not found !'
        assert node.wait_for_topic('/link3_wrench_sensor/wrench', ['geometry_msgs/msg/WrenchStamped']), 'wrench topic not found !'
        assert node.wait_for_message('/link3_wrench_sensor/wrench', WrenchStamped, 'link3'), 'wrench message not found !'

    # OpenGL tests
    @unittest.skipUnless(opengl_enabled(), "OpenGL disabled (DISABLE_OPENGL=1)")
    def test_camera(self):
        node = self.node
        assert node.wait_for_node('test_camera'), 'camera Node not found !'
        assert node.wait_for_topic('/test_camera/color/image_raw', ['sensor_msgs/msg/Image']), 'color image topic not found !'
        assert node.wait_for_topic('/test_camera/color/camera_info', ['sensor_msgs/msg/CameraInfo']), 'color camera_info topic not found !'
        assert node.wait_for_topic('/test_camera/depth/image_rect_raw', ['sensor_msgs/msg/Image']), 'depth image topic not found !'
        assert node.wait_for_topic('/test_camera/depth/points', ['sensor_msgs/msg/PointCloud2']), 'depth points topic not found !'
        assert node.wait_for_message('/test_camera/color/image_raw', Image, 'test_camera_link', timeout=15.0), 'color image message not found !'
        assert node.wait_for_message('/test_camera/color/camera_info', CameraInfo, 'test_camera_link', timeout=15.0), 'color camera_info message not found !'
        assert node.wait_for_message('/test_camera/depth/image_rect_raw', Image, 'test_camera_link', timeout=15.0), 'depth image message not found !'
        assert node.wait_for_message('/test_camera/depth/points', PointCloud2, 'test_camera_link', timeout=15.0), 'depth points message not found !'

    @unittest.skipUnless(opengl_enabled(), "OpenGL disabled (DISABLE_OPENGL=1)")
    def test_lidar_scan_mode(self):
        node = self.node
        assert node.wait_for_node('lidar_scan'), 'lidar_scan Node not found !'
        assert node.wait_for_topic('/lidar_scan/scan', ['sensor_msgs/msg/LaserScan']), 'lidar scan topic not found !'
        assert node.wait_for_message('/lidar_scan/scan', LaserScan, 'base_link', timeout=15.0), 'lidar scan message not found !'

    @unittest.skipUnless(opengl_enabled(), "OpenGL disabled (DISABLE_OPENGL=1)")
    def test_lidar_cloud_mode(self):
            node = self.node
            assert node.wait_for_node('lidar_cloud'), 'lidar_cloud Node not found !'
            assert node.wait_for_topic('/lidar_cloud/points', ['sensor_msgs/msg/PointCloud2']), 'lidar cloud topic not found !'
            assert node.wait_for_message('/lidar_cloud/points', PointCloud2, 'base_link', timeout=15.0), 'lidar cloud message not found !'