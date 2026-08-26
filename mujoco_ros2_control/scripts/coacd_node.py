#!/usr/bin/env python3
"""ROS node wrapper around run_coacd.decompose_mesh(), for use from a launch
file the way xacro2mjcf.py is used: as a `Node` action taking a list of
inputs as a parameter, rather than one `ExecuteProcess` per mesh.

Usage (see mujoco_ros2_control_examples/launch/franka.launch.py):

    Node(
        package="mujoco_ros2_control",
        executable="coacd_node.py",
        parameters=[{"meshes": [mesh_path, ...]}],
    )

Processes `meshes` in order, one process, and exits once every mesh either
has its decomposed folder already (see decompose_mesh()'s docstring) or has
just been given one. A launch description chains whatever comes next off
this node's exit, exactly like it would off xacro2mjcf.py's.
"""

import rclpy
from rclpy.node import Node

from run_coacd import decompose_mesh


class CoacdNode(Node):
    def __init__(self):
        super().__init__("coacd_node")
        self.declare_parameter("meshes", rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter("threshold", 0.05)
        self.declare_parameter("quiet", False)

        meshes = self.get_parameter("meshes").value
        threshold = self.get_parameter("threshold").value
        quiet = self.get_parameter("quiet").value

        for mesh in meshes:
            self.get_logger().info(f"Decomposing '{mesh}'...")
            decompose_mesh(mesh, threshold=threshold, quiet=quiet)

        self.get_logger().info(f"Decomposed {len(meshes)} mesh(es).")
        self.destroy_node()
        exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = CoacdNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
