# ur_mujoco

Universal Robots (UR) arm simulated with **mujoco_ros2_control**.

This example loads a UR arm from [`ur_description`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description),
converts it to a MuJoCo MJCF model with `xacro2mjcf.py`, and drives it through
`ros2_control` using the `mujoco_ros2_control/MujocoSystem` hardware interface.

## Run

```bash
ros2 launch ur_mujoco ur.launch.py
```

Arguments:

| Argument  | Default | Description                                                              |
|-----------|---------|--------------------------------------------------------------------------|
| `ur_type` | `ur5e`  | UR model: `ur3`, `ur5`, `ur10`, `ur3e`, `ur5e`, `ur10e`, `ur16e`, `ur20`, `ur30`, … |
| `rviz`    | `true`  | Start RViz.                                                              |

```bash
ros2 launch ur_mujoco ur.launch.py ur_type:=ur10e rviz:=false
```

## Notes

UR ships `.dae` visual meshes and `.stl` collision meshes. `xacro2mjcf.py` symlinks
the `.stl` meshes into `<mujoco_files_path>/meshes` and substitutes the collision
mesh for the (![MuJoCo-unsupported](https://mujoco.readthedocs.io/en/stable/programming/extension.html#decoders)) `.dae` visual, so the rendered geometry is the
collision mesh.
