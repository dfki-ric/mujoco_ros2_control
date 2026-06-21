# mujoco_ros2_control_examples

Example robots simulated with [`mujoco_ros2_control`](../mujoco_ros2_control). Each
example loads a robot description, converts it to a MuJoCo MJCF model with
`xacro2mjcf.py`, and drives it through `ros2_control` using the
`mujoco_ros2_control/MujocoSystem` hardware interface.

This package merges the former `franka_mujoco`, `ur_mujoco`, `unitree_h1_mujoco`
and `task_table_mujoco` packages. Assets are split per robot:

```
launch/   <robot>.launch.py
urdf/     franka/  ur/  unitree_h1/  task_table/
config/   franka/  ur/  unitree_h1/
meshes/   task_board/         (Franka task table)
          unitree_h1/         (downloaded at build time)
patches/  unitree_h1.urdf.patch
test/     per-robot launch smoke tests
```

## Examples

### Franka

Franka arm (optionally with the task table of high-resolution collision gears and
pose sensors). Requires [`franka_description`](https://github.com/frankarobotics/franka_description).

> **Jazzy:** `franka_description` is not available from apt. Clone its `jazzy`
> branch into your workspace `src/` and build it:
> ```bash
> git clone -b jazzy https://github.com/frankarobotics/franka_description.git src/franka_description
> ```

```bash
ros2 launch mujoco_ros2_control_examples franka.launch.py
```

| Argument           | Default        | Description                                              |
|--------------------|----------------|----------------------------------------------------------|
| `rviz`             | `true`         | Start RViz.                                              |
| `load_task_table`  | `true`         | Load the high-resolution collision objects + pose sensors. |
| `load_gripper`     | `true`         | Attach an end-effector.                                  |
| `ee_id`            | `franka_hand`  | End-effector: `none`, `franka_hand`, `cobot_pump`.       |
| `arm_id`           | `fr3`          | Arm: `fer`, `fr3`, `fp3`.                                |

### Universal Robots (UR)

UR arm loaded from [`ur_description`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description).

```bash
ros2 launch mujoco_ros2_control_examples ur.launch.py ur_type:=ur10e rviz:=false
```

| Argument  | Default | Description                                                      |
|-----------|---------|------------------------------------------------------------------|
| `ur_type` | `ur5e`  | `ur3`, `ur5`, `ur10`, `ur3e`, `ur5e`, `ur10e`, `ur16e`, `ur20`, … |
| `rviz`    | `true`  | Start RViz.                                                      |

UR ships `.dae` visual meshes and `.stl` collision meshes. `xacro2mjcf.py` symlinks
the `.stl` meshes into `<mujoco_files_path>/meshes` and substitutes the collision
mesh for the (MuJoCo-unsupported) `.dae` visual.

### Unitree H1

Unitree H1 humanoid. The upstream URDF and meshes are **not** redistributed here;
they are downloaded from the [`unitree_ros`](https://github.com/unitreerobotics/unitree_ros)
repository and patched at build time (`patches/unitree_h1.urdf.patch`).

```bash
ros2 launch mujoco_ros2_control_examples unitree_h1.launch.py
```

To build the rest of the examples without network access, disable the download:

```bash
colcon build --packages-select mujoco_ros2_control_examples \
  --cmake-args -DDOWNLOAD_UNITREE_H1_ASSETS=OFF
```

## Tests

`test/` holds one launch smoke test per robot. Each converts the robot to MJCF,
starts the `mujoco_ros2_control` node headless, and asserts the node comes up
(i.e. MuJoCo accepted the generated model). A test self-skips when its robot
description package is not installed, or - for Unitree H1 - when the upstream
assets were not downloaded.

OpenGL is optional: export `DISABLE_OPENGL=1` to run fully headless (no GUI), as CI
does. Without it, the MuJoCo GUI is shown.

```bash
colcon test --packages-select mujoco_ros2_control_examples
DISABLE_OPENGL=1 colcon test --packages-select mujoco_ros2_control_examples   # headless
```

## Task table assets

See [`urdf/task_table/`](urdf/task_table) and [`meshes/task_board/`](meshes/task_board).
The collision meshes were generated with [Phobos](https://github.com/dfki-ric/phobos)
and [CoACD](https://github.com/SarahWeiii/CoACD); use `mujoco_ros2_control/scripts/run_coacd.py`
to decompose a mesh into collision-friendly components.
