#!/usr/bin/env python3
"""ROS node wrapper around prepare_terrain.build_scene(), for use from a
launch file as a `Node` action taking parameters, the way xacro2mjcf.py and
coacd_node.py are used, rather than an `ExecuteProcess` with CLI flags.

Usage (see mujoco_ros2_control_examples/launch/unitree_g1.launch.py):

    Node(
        package="mujoco_ros2_control",
        executable="prepare_terrain_node.py",
        parameters=[{"out_dir": terrain_dir, "difficulty": 0.5, ...}],
    )

Every parameter is optional; unset ones fall back to build_scene()'s own
defaults, from prepare_terrain.py's DEFAULT_* constants and SteppingStonesCfg.
`prepare_terrain.py` itself stays a plain CLI script -- README.md documents
running it by hand, and this node imports build_scene() from it rather than
duplicating the generator.
"""

import rclpy
from rclpy.node import Node

from prepare_terrain import (
    DEFAULT_DIFFICULTY,
    DEFAULT_FIELD_HALF,
    DEFAULT_HFIELD_SCALE,
    DEFAULT_HORIZONTAL_SCALE,
    DEFAULT_SEED,
    SteppingStonesCfg,
    build_scene,
)


class PrepareTerrainNode(Node):
    def __init__(self):
        super().__init__("prepare_terrain_node")
        self.declare_parameter("out_dir", ".")
        self.declare_parameter("scene_name", "terrain_scene.xml")
        self.declare_parameter("difficulty", DEFAULT_DIFFICULTY)
        self.declare_parameter("num_levels", 1)
        self.declare_parameter("seed", DEFAULT_SEED)
        self.declare_parameter("field_half", DEFAULT_FIELD_HALF)
        self.declare_parameter("horizontal_scale", DEFAULT_HORIZONTAL_SCALE)
        self.declare_parameter("collider", "mesh")
        self.declare_parameter("hfield_scale", DEFAULT_HFIELD_SCALE)
        self.declare_parameter("max_height_var", SteppingStonesCfg.max_height_var)
        self.declare_parameter("max_tilt_deg", SteppingStonesCfg.max_tilt_deg)
        self.declare_parameter("max_drop_frac", SteppingStonesCfg.max_drop_frac)

        result = build_scene(
            out_dir=self.get_parameter("out_dir").value,
            scene_name=self.get_parameter("scene_name").value,
            difficulty=self.get_parameter("difficulty").value,
            seed=self.get_parameter("seed").value,
            field_half=self.get_parameter("field_half").value,
            horizontal_scale=self.get_parameter("horizontal_scale").value,
            collider=self.get_parameter("collider").value,
            hfield_scale=self.get_parameter("hfield_scale").value,
            num_levels=self.get_parameter("num_levels").value,
            max_height_var=self.get_parameter("max_height_var").value,
            max_tilt_deg=self.get_parameter("max_tilt_deg").value,
            max_drop_frac=self.get_parameter("max_drop_frac").value,
        )
        self.get_logger().info(
            f"difficulty {self.get_parameter('difficulty').value:.2f} -> "
            f"level {result['difficulty']:.2f}, {result['collider']} collider, "
            f"wrote {len(result['files'])} file(s) to "
            f"{self.get_parameter('out_dir').value}")

        self.destroy_node()
        exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = PrepareTerrainNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
