# mujoco_ros2_control_examples

Example robots simulated with [`mujoco_ros2_control`](../mujoco_ros2_control). Each
example loads a robot description, converts it to a MuJoCo MJCF model with
`xacro2mjcf.py`, and drives it through `ros2_control` using the
`mujoco_ros2_control/MujocoSystem` hardware interface.

This package merges the former `franka_mujoco`, `ur_mujoco`, `unitree_h1_mujoco`
and `task_table_mujoco` packages. Assets are split per robot:

```
launch/   <robot>.launch.py
urdf/     franka/  ur/  unitree_h1/  industreal/
config/   franka/  ur/  unitree_h1/
meshes/   industreal/         (connector-fixture and gear assets; pegs/ downloaded at build time)
          unitree_h1/         (downloaded at build time)
patches/  unitree_h1.urdf.patch
test/     per-robot launch smoke tests
```

## Examples

### Franka

Franka arm (optionally with a modular NVIDIA IndustReal benchmark board).
Requires [`franka_description`](https://github.com/frankarobotics/franka_description).

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
| `load_industreal_board` | `true`      | Load the unified board as an additional scene model.       |
| `task_board_config` | package default | YAML selecting gears, peg assemblies, and connector fixtures. |
| `load_gripper`     | `true`         | Attach an end-effector.                                  |
| `ee_id`            | `franka_hand`  | End-effector: `none`, `franka_hand`, `cobot_pump`.       |
| `arm_id`           | `fr3`          | Arm: `fer`, `fr3`, `fp3`.                                |
| `load_realsense`   | `true`         | Attach the simulated D435 wrist camera and official Franka mount. |
| `realsense_xyz`    | `0.025 0 0.0075` | Centre of the front-side Hand M6 mounting hole.          |
| `realsense_rpy`    | `0 0 0` | M6 parent pose; the assembly Y rotation is internal.      |

The wrist camera publishes:

```text
/d435/color/image_raw
/d435/color/camera_info
/d435/depth/image_rect_raw
/d435/depth/camera_info
/d435/depth/points
```

Its stream settings are in `config/franka/realsense_d435.yaml`. Simulation uses
the built-in MuJoCo RGB-D renderer, so neither `librealsense2` nor
`realsense2_camera` is required. The standard description package is required:

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-realsense2-description
```

Set `load_realsense:=false` to omit the camera. Camera rendering is automatically
disabled by `headless:=true` because it requires an OpenGL context; the mount and
camera remain in the robot description.

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

See [Offline builds](#offline-builds) to build the other examples without
downloading these.

## Offline builds

Two asset groups are fetched at configure time and can each be turned off:

| CMake option                       | Default | Fetches                                        |
|------------------------------------|---------|------------------------------------------------|
| `DOWNLOAD_UNITREE_H1_ASSETS`       | `ON`    | The Unitree H1 URDF (patched) and its meshes.   |
| `DOWNLOAD_INDUSTREAL_PEG_ASSETS`   | `ON`    | The IndustReal peg and tray meshes (~12 MB).    |

```bash
colcon build --packages-select mujoco_ros2_control_examples --cmake-args \
  -DDOWNLOAD_UNITREE_H1_ASSETS=OFF -DDOWNLOAD_INDUSTREAL_PEG_ASSETS=OFF
```

With the peg assets off, the Franka example must be launched without the peg
rows - either `load_industreal_board:=false`, or a `task_board_config` whose
`*_peg_*` components are all disabled. The connector fixtures and gears are
in-tree and always available.

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

See [`urdf/industreal/`](urdf/industreal) and [`meshes/industreal/`](meshes/industreal).
The collision meshes were generated with [Phobos](https://github.com/dfki-ric/phobos)
and [CoACD](https://github.com/SarahWeiii/CoACD); use `mujoco_ros2_control/scripts/run_coacd.py`
to decompose a mesh into collision-friendly components.

The default `config/franka/industreal_task_board.yaml` describes one unified
board: a medium-dark-grey optical breadboard, the 8, 12, and 16 mm round and rectangular
peg rows, all four NEMA fixture positions, and the gear base plus its three
movable gears. Every group and every pose is configured in this single file;
the gears are no longer loaded as a separate task-table model. Exact NVIDIA
connector-tray meshes are used for rendering, with stable primitive collision
shapes because the public meshes are non-watertight. NVIDIA only identifies the
commercial plug/socket part numbers and does not redistribute those meshes;
the pick-side plug bodies are therefore dimensioned approximations, but are
independent free bodies and can be grasped. Pass an alternate file
without changing the package:

```bash
ros2 launch mujoco_ros2_control_examples franka.launch.py \
  task_board_config:=/absolute/path/to/my_task_board.yaml
```

The simulator-ready peg assets come from
[`IsaacGymEnvs`](https://github.com/isaac-sim/IsaacGymEnvs/tree/main/isaacgymenvs/tasks/industreal)
and are **not** redistributed here; they are downloaded at build time, like the
Unitree H1 meshes. The connector fixtures come from
[`IndustRealKit`](https://github.com/NVlabs/industrealkit) and are kept in-tree,
because IndustRealKit serves its meshes only through Git LFS. Their provenance and
license notices are retained in `meshes/industreal/`; IndustRealKit assets are
limited to non-commercial research/evaluation use.
