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
meshes/   industreal/         (gear assets; pegs/ downloaded at build time)
          unitree_h1/         (downloaded at build time)
          franka/             (D435 wrist mount, downloaded at build time)
patches/  unitree_h1.urdf.patch
scripts/  stl_to_obj.py       (build-time mesh conversion)
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
| `task_board_config` | package default | YAML selecting the gears and the peg assemblies.          |
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

#### The camera mount mesh

The mount is Franka's own printable bracket, the one documented in *3D Printable
Camera Mount Guide* R02241. It is **not** redistributed here: the archive Franka
publishes ships no licence file and the guide carries only a copyright line, so
nothing grants permission to copy or modify it. `DOWNLOAD_FRANKA_CAMERA_MOUNT_ASSET`
fetches `https://download.franka.de/camera_mount_guide.zip` at configure time and
installs one file out of it, so the mesh only ever exists in the install tree.

Two details are worth knowing if it ever needs re-checking:

- **The mesh is converted to OBJ on the way in.** MuJoCo's STL decoder rejects
  anything over 200000 faces and the mount is 295768, so `scripts/stl_to_obj.py`
  rewrites the triangle soup rather than decimating it. The conversion is
  lossless - STL carries no texture coordinates and its per-facet normals are
  recomputed by every consumer - and it preserves the exact 48 x 60 x 49.95 mm
  bounds that the visual origin and the box collision were measured against.
- **The download URL carries no revision**, so it cannot be pinned the way the
  IndustReal peg commit is. CMake records the archive's SHA256 and *warns*
  instead of failing when it stops matching: a revised mount would move the
  camera with no other symptom, but a hard failure would break every build the
  day Franka republishes.

See also [Offline builds](#offline-builds).

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

### Unitree G1

Unitree G1 29-DOF humanoid, carrying a depth camera (`/d435/...`) and a LiDAR
(`/lidar_head/...`). As with the H1, the upstream URDF and meshes are **not**
redistributed here; they are downloaded from
[`unitree_ros`](https://github.com/unitreerobotics/unitree_ros) at build time.

```bash
ros2 launch mujoco_ros2_control_examples unitree_g1.launch.py
```

The 29 joints are commanded through five `JointGroupPositionController`s -
`left_leg`, `right_leg`, `waist`, `left_arm` and `right_arm`, each named
`<group>_position_controller`. Each forwards a position setpoint per joint; the
stiff PD that turns it into torque lives in the hardware (`kp`/`kd` per joint in
`urdf/unitree_g1/ros2_control.urdf.xacro`), so the robot holds its start pose
until a command arrives.

```bash
# left_arm joints, in config order: shoulder p/r/y, elbow, wrist r/p/y
ros2 topic pub --once /left_arm_position_controller/commands \
  std_msgs/msg/Float64MultiArray '{data: [0.3, 0.25, 0.0, 1.5, 0.15, 0.0, 0.0]}'
```

#### Stepping-stones terrain

By default this example spawns on a Voronoi stepping-stones field generated by
`mujoco_ros2_control`'s `prepare_terrain.py`: stones of varying height with
tilted tops, separated by gaps and scattered holes, over a 1 m pit. The defaults
below pave a 30 x 30 m field with a few hundred stones. Pass `terrain:=flat` for
a plain ground plane instead.

```bash
ros2 launch mujoco_ros2_control_examples unitree_g1.launch.py
ros2 launch mujoco_ros2_control_examples unitree_g1.launch.py terrain:=flat
ros2 launch mujoco_ros2_control_examples unitree_g1.launch.py terrain_difficulty:=0.3
```

| Argument             | Default  | Meaning                                                                   |
|----------------------|----------|---------------------------------------------------------------------------|
| `terrain`            | `stones` | A `stones` field, or a `flat` ground plane.                               |
| `terrain_difficulty` | `0.5`    | `0` = large, close, level stones; `1` = small, far apart, uneven.         |
| `terrain_seed`       | `42`     | RNG seed; the same seed always yields the same field.                      |
| `terrain_collider`   | `mesh`   | `mesh` = one convex prism per stone, `hfield` = a single heightfield geom. |
| `terrain_field_half` | `15.0`   | Half-extent of the square field, m (so 30 x 30 m).                        |
| `terrain_height_var` | `0.10`   | How far stone tops sit above/below `z=0` at difficulty 1, m.              |
| `terrain_tilt_deg`   | `8.0`    | How far stone tops tilt out of horizontal at difficulty 1, degrees.        |
| `terrain_hole_frac`  | `0.15`   | Fraction of stones dropped at difficulty 1, leaving holes. `0` = none.     |

Difficulty scales every knob together: at `0` the stones are large, close, all
level and unbroken, and at `1` they are small, far apart, offset in height,
tilted, and punctured by holes dropping to the pit. Each effect is also its own
knob, so a hard field can still be made flat (`terrain_height_var:=0
terrain_tilt_deg:=0`) or unbroken (`terrain_hole_frac:=0`).

The spawn platform at the origin stays flat and level at `z=0` whatever the
stones around it do, so the robot spawns at the same height on either terrain.
`mesh` gives crisp stone edges and exact tilted tops, at one collision geom per
stone; `hfield` is a single geom, but MuJoCo interpolates between samples so
every vertical wall becomes a slope one cell wide.

The generator is also a standalone CLI, useful for inspecting a field before
launching:

```bash
ros2 run mujoco_ros2_control prepare_terrain.py --out-dir /tmp/terrain --difficulty 1.0
```

The scene it writes stands in for `mujoco_ros2_control`'s `mjcf/scene.xml` -
pass one or the other to `xacro2mjcf`, never both, since they define the same
skybox, groundplane material and floor geom.

## Offline builds

Four asset groups are fetched at configure time and can each be turned off:

| CMake option                        | Default | Fetches                                       |
|-------------------------------------|---------|-----------------------------------------------|
| `DOWNLOAD_UNITREE_H1_ASSETS`        | `ON`    | The Unitree H1 URDF (patched) and its meshes.  |
| `DOWNLOAD_UNITREE_G1_ASSETS`        | `ON`    | The Unitree G1 URDF (patched) and its meshes.  |
| `DOWNLOAD_INDUSTREAL_PEG_ASSETS`    | `ON`    | The IndustReal peg and tray meshes (~12 MB).   |
| `DOWNLOAD_FRANKA_CAMERA_MOUNT_ASSET`| `ON`    | The Franka printable D435 mount (~6 MB zip).   |

```bash
colcon build --packages-select mujoco_ros2_control_examples --cmake-args \
  -DDOWNLOAD_UNITREE_H1_ASSETS=OFF -DDOWNLOAD_UNITREE_G1_ASSETS=OFF \
  -DDOWNLOAD_INDUSTREAL_PEG_ASSETS=OFF \
  -DDOWNLOAD_FRANKA_CAMERA_MOUNT_ASSET=OFF
```

With the peg assets off, the Franka example must be launched without the peg
rows - either `load_industreal_board:=false`, or a `task_board_config` whose
`*_peg_*` components are all disabled. With the camera mount off it must be
launched with `load_realsense:=false`. The gears are in-tree and always
available.

## Tests

`test/` holds one launch smoke test per robot. Each converts the robot to MJCF,
starts the `mujoco_ros2_control` node headless, and asserts the node comes up
(i.e. MuJoCo accepted the generated model). A test self-skips when its robot
description package is not installed, or - for the Unitree robots - when the
upstream assets were not downloaded.

The Unitree G1 test checks more than that the node starts: it brings the robot up
on its default generated stepping-stones terrain.

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
board: a medium-dark-grey optical breadboard, the 8, 12, and 16 mm round and
rectangular peg rows, and the gear base plus its three movable gears. Every
group and every pose is configured in this single file; the gears are no longer
loaded as a separate task-table model. Pass an alternate file without changing
the package:

```bash
ros2 launch mujoco_ros2_control_examples franka.launch.py \
  task_board_config:=/absolute/path/to/my_task_board.yaml
```

### Asset provenance and licences

Three of the four asset groups this package uses are downloaded at build time
and never redistributed here: the Unitree H1 URDF and meshes, the IndustReal peg
and tray meshes from
[`IsaacGymEnvs`](https://github.com/isaac-sim/IsaacGymEnvs/tree/main/isaacgymenvs/tasks/industreal),
and the Franka camera mount. See [Offline builds](#offline-builds).

The gears are the exception, and the one asset group **not covered by this
package's licence**. They are a derivative work of
[`IndustRealKit`](https://github.com/NVlabs/industrealkit) CAD, they cannot be
re-fetched, and the NVIDIA License they carry limits them and any derivative to
**non-commercial research or evaluation use**. The specific files and the reason
the restriction has to be named are set out in
[`meshes/industreal/README.md`](meshes/industreal/README.md); the licence text
itself is in `meshes/industreal/licenses/`. Build with a `task_board_config`
whose `gears.enabled` is `false` to get a board without that restriction
attached.

The board does not model IndustReal's electrical-connector task. Its plugs and
sockets are commercial parts that NVIDIA identifies by part number but does not
redistribute, so only the printable trays are public and there is no geometry
for the parts that actually mate.

## License

BSD-3-Clause, Copyright (c) 2025 DFKI GmbH, Robotics Innovation Center - the
same terms as the rest of this repository. See [`LICENSE`](LICENSE).

One exception: the gear meshes under `meshes/industreal/gears/` are a derivative
of NVIDIA IndustRealKit CAD and stay under the NVIDIA License, which restricts
them to non-commercial research or evaluation use. They are the only files in
this package not covered by the line above. Everything else the examples pull
from third parties - the Unitree H1 description, the IndustReal peg meshes, the
Franka camera mount - is downloaded at build time and never redistributed here,
so no third-party terms attach to this repository for them. See
[Asset provenance and licences](#asset-provenance-and-licences).
