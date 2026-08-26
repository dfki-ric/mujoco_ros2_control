# mujoco_ros2_control_examples

Example robots simulated with [`mujoco_ros2_control`](../mujoco_ros2_control). Each
example loads a robot description, converts it to a MuJoCo MJCF model with
`xacro2mjcf.py`, and drives it through `ros2_control` using the
`mujoco_ros2_control/MujocoSystem` hardware interface.

This package merges the former `franka_mujoco`, `ur_mujoco`, `unitree_h1_mujoco`
and `task_table_mujoco` packages. Assets are split per robot:

```
launch/   <robot>.launch.py
urdf/     franka/  ur/  unitree_h1/  unitree_g1/  industreal/  tactile/
          body_services/
config/   franka/  ur/  unitree_h1/  unitree_g1/  tactile/
          body_services/
srv/      GetBodyState.srv  SetBodyPose.srv   (the BodyServices interfaces)
meshes/   industreal/         (gear assets; pegs/ downloaded at build time)
          unitree_h1/         (downloaded at build time)
          unitree_g1/         (downloaded at build time)
          franka/             (D435 wrist mount, downloaded at build time)
patches/  unitree_h1.urdf.patch (the G1 URDF is rewritten in place instead)
scripts/  stl_to_obj.py       (build-time mesh conversion)
include/  src/                (the TouchGridSensor and BodyServices example plugins)
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

#### Several arms in a row

`ur_multi.launch.py` stands several UR arms side by side in one MuJoCo world. By
default they are three *different* arms — a ur3e, a ur5e and a ur10e:

```bash
ros2 launch mujoco_ros2_control_examples ur_multi.launch.py
ros2 launch mujoco_ros2_control_examples ur_multi.launch.py ur_types:=ur3,ur10e
ros2 launch mujoco_ros2_control_examples ur_multi.launch.py ur_types:=ur5e,ur5e,ur5e
```

| Argument   | Default            | Description                                                        |
|------------|--------------------|--------------------------------------------------------------------|
| `ur_types` | `ur3e,ur5e,ur10e`  | One UR type per arm, in row order. Its length is the arm count.     |
| `spacing`  | `1.5`              | Distance between neighbouring bases along y, m.                     |
| `rviz`     | `true`             | Start RViz.                                                        |

Each arm is prefixed with its own type — `ur3e_`, `ur5e_`, `ur10e_` — so every
joint, frame, hardware component and controller name says which robot it belongs
to (`ur10e_joint_trajectory_controller`, `ur3e_wrist_3_link`, …). A type repeated
in the list gets an occurrence suffix, so `ur_types:=ur5e,ur5e` yields `ur5e_` and
`ur5e_2_` rather than colliding.

One description holds every arm, so there is a single `robot_description`, a
single MJCF and a single controller manager. Each arm lives under its `tf_prefix`,
which names its links, joints, MuJoCo actuators, force/torque site and its own
`<ros2_control>` block. The controller manager creates one `MujocoSystem`
hardware component per block, so each arm has its own
`<type>_joint_trajectory_controller` and `<type>_ft_sensor_broadcaster` and can be
commanded independently; a single shared `joint_state_broadcaster` publishes every
joint.

`ur_multi.urdf.xacro` is the single definition of the prefixes; the launch file
reads them back off the description it generated rather than deriving them again.

An arm is `urdf/ur/ur_arm.urdf.xacro`, the same macro the single-arm example
instantiates once — so the two examples cannot drift apart. The model-wide MuJoCo
compiler and solver settings live in `urdf/ur/mujoco_options.urdf.xacro`, which a
description includes exactly once however many arms it holds.

Because the controller names follow the arm types, `config/ur/ur_multi_controllers.yaml`
matches the default `ur_types` (`ur3e,ur5e,ur10e`). Launching a different list
needs matching entries there.

Each arm's MuJoCo actuators take their torque limits from that arm's own type
(`ur_max_effort` in `urdf/ur/mujoco_actuators.urdf.xacro`, copied from
`ur_description/config/<ur_type>/joint_limits.yaml`), with the gains tuned on the
ur5e scaled by torque. Before this the ur5e row was hard-coded for every arm,
which silently gave a ur3e 2.8× the shoulder torque it has and capped a ur10e at
45% of its own — so it also affected `ur.launch.py ur_type:=ur10e`.

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

### Tactile pad (touch_grid)

A 6 cm pad on a prismatic joint, resting on the ground plane, with a MuJoCo
`touch_grid` sensor imaging its contact face. Unlike the other examples this one
is about the sensor rather than the robot: it is the worked example of a
`MujocoRos2ControlSensorInterface` plugin living **outside** `mujoco_ros2_control`.

```bash
ros2 launch mujoco_ros2_control_examples tactile.launch.py
ros2 launch mujoco_ros2_control_examples tactile.launch.py headless:=true
ros2 topic echo /pad_touch/touch_grid
```

A `touch_grid` emits `nchannel * width * height` values per step, which does not
fit ros2_control's scalar state interfaces, so
[`TouchGridSensor`](src/touch_grid_sensor.cpp) exports none at all and publishes a
`std_msgs/Float64MultiArray` instead. The array is channel-major and its
`layout.dim` is labelled `channel`, `height`, `width`, so a consumer can index it
as an image: `data[k * width * height + j * width + i]`. Channels are the force
components `[normal, tangent, tangent]` followed by the torque components
`[torsional, rolling, rolling]`, truncated to the model's `nchannel` - the example
declares 3, so it publishes forces only.

The MJCF side needs the engine plugin declared in an `<extension>` block, and the
ros2_control side names the sensor plugin
([`urdf/tactile/tactile_pad.urdf.xacro`](urdf/tactile/tactile_pad.urdf.xacro)):

```xml
<mujoco>
    <extension>
        <plugin plugin="mujoco.sensor.touch_grid"/>
    </extension>

    <reference name="pad">
        <!-- top of the pad, imaging downwards (-z) at the contact face -->
        <site name="pad_touch_site" size="0.005" pos="0 0 0.02"/>
    </reference>

    <sensor>
        <plugin name="pad_touch_grid" plugin="mujoco.sensor.touch_grid"
                objtype="site" objname="pad_touch_site">
            <config key="nchannel" value="3"/>
            <config key="size"     value="7 7"/>
            <config key="fov"      value="45 45"/>
            <config key="gamma"    value="0"/>
        </plugin>
    </sensor>
</mujoco>

<!-- inside <ros2_control>: no state interfaces, it publishes a topic -->
<sensor name="pad_touch">
    <param name="plugin">mujoco_ros2_control_examples/TouchGridSensor</param>
    <param name="site">pad_touch_site</param>
</sensor>
```

Everything else the plugin needs comes from `<param>` entries on the `<sensor>`:

| `<param>` | Default | Meaning |
|-----------|---------|---------|
| `site` | the sensor name | The site the `touch_grid` is attached to. |
| `topic` | `<sensor name>/touch_grid` | Topic to publish on. |
| `frame_id` | the site name | Frame the taxel values are expressed in. Informational only - `Float64MultiArray` carries no header, so this is just logged at startup. |
| `publish` | `true` | Set `false` to step the sensor without producing topic traffic. |

Three things are easy to get wrong when copying this:

- **The site images along its negative z.** It must sit *above* the contact face
  looking down through the body. Placed flush with the face - or flipped with
  `zaxis="0 0 -1"` - it sees nothing and publishes a correctly shaped block of
  zeros.
- **The pad has to be free to move.** A body welded to the world joins the
  world's weld group, and MuJoCo generates no contacts within a weld group, so a
  welded pad reports no touch at all.
- **`mujoco.sensor.touch_grid` is a MuJoCo engine plugin** and has to be loaded
  before the model is compiled. `mujoco_ros2_control` does that by default from
  its own `lib/mujoco_plugin` directory; see
  [MuJoCo Engine Plugins](../mujoco_ros2_control/README.md#mujoco-engine-plugins).

For what writing such a plugin involves - the interface, the CMake and
`package.xml` wiring, and the control-loop rules - see
[Writing a Sensor Plugin](../mujoco_ros2_control/README.md#writing-a-sensor-plugin).
The other examples use the plugins that ship with `mujoco_ros2_control`:
`ImuSensor` and `PoseSensor` on the Unitree robots, `ForceTorqueSensor` on Franka
and UR.

### Body services (read and teleport bodies)

Two probe boxes and no robot at all. Like the tactile pad this example is about
the plugin, not the scene: it is the worked example of reaching into a running
simulation from outside, and of a plugin that brings its **own generated service
types** rather than reusing an existing interface package.

```bash
ros2 launch mujoco_ros2_control_examples body_services.launch.py
ros2 launch mujoco_ros2_control_examples body_services.launch.py headless:=true

ros2 service call /mujoco_get_body_state \
    mujoco_ros2_control_examples/srv/GetBodyState "{body_name: probe_float}"

ros2 service call /mujoco_set_body_pose \
    mujoco_ros2_control_examples/srv/SetBodyPose \
    "{body_name: probe_float, x: 0.5, y: -0.25, z: 1.75, qw: 1.0}"
```

[`BodyServices`](src/body_services.cpp) serves two services: one reads a body's
world pose and twist, the other teleports a body with a single free joint.
Teleporting is a convenience for *driving* a simulation - an external training
loop, a scripted scenario - rather than part of simulating one, which is why it
lives here and not in `mujoco_ros2_control`. Both services were once built into
the simulation node unconditionally; now nothing advertises them unless a model
asks for them.

Declared in
[`urdf/body_services/probe_bodies.urdf.xacro`](urdf/body_services/probe_bodies.urdf.xacro),
twice - which is the point of the parameters:

```xml
<!-- outside <ros2_control>: the plugin exports nothing to ros2_control -->
<mujoco_ros2_plugin name="body_services"
                    plugin="mujoco_ros2_control_examples/BodyServices"/>

<mujoco_ros2_plugin name="body_services_alt"
                    plugin="mujoco_ros2_control_examples/BodyServices">
  <param name="get_body_state_service">alt/get_body_state</param>
  <param name="set_body_pose_service">alt/set_body_pose</param>
</mujoco_ros2_plugin>
```

| `<param>` | Default | Meaning |
|-----------|---------|---------|
| `get_body_state_service` | `mujoco_get_body_state` | Name of the read service. |
| `set_body_pose_service` | `mujoco_set_body_pose` | Name of the teleport service. |

A node's name does not prefix service names - only its namespace does - so the
declaration's `name` does not affect where the services land. That is what the
parameters are for: several models in one simulation each get their own pair
instead of fighting over the defaults. Both instances read and write the same
simulation, so `/alt/set_body_pose` followed by `/mujoco_get_body_state` sees the
move.

Three things are easy to get wrong when copying this:

- **Only a body whose sole joint is free can be teleported.** `probe_float` can
  be; `probe_fixed` is welded to the world and the set service rejects it. A weld
  is not a pose you can write - MuJoCo has no degrees of freedom to write it into.
- **Gravity is off in this model** so `probe_float` stays where it is put. With
  gravity on, a teleported free body starts falling on the next step and a
  read-back a moment later no longer matches what was written.
- **The write is not deferred to the next step.** It happens in the service
  callback, under both simulation locks, followed by `mj_forward()`. That is
  deliberate: under `synchronous_mode` nothing advances the simulation except a
  step request, so a queued pose would not land until the client asked for a step
  - while a client that sets a pose and reads it straight back expects to see what
  it just wrote. Try it with `synchronous:=true`.

| Argument      | Default | Description                                                    |
|---------------|---------|----------------------------------------------------------------|
| `headless`    | `false` | Run without the MuJoCo viewer window.                          |
| `synchronous` | `false` | Start paused; advance only on `/mujoco_step_simulation`.        |

For what writing such a plugin involves, see
[Writing a Sensor Plugin](../mujoco_ros2_control/README.md#writing-a-sensor-plugin).

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

Four tests check more than that the node starts. The Unitree G1 test brings the
robot up on its default generated stepping-stones terrain, and the `ur_multi`
test asserts every arm reached the merged model, that the bases really are evenly
spaced in a row, that the arm behind each prefix really is the type that prefix
names, and that the controller manager instantiated one hardware component per
arm. The `tactile` test covers the plugin path end to end: that `pluginlib`
resolved this package's out-of-tree `TouchGridSensor`, that the published array
has the taxel count and labelled channel-major layout the MJCF asks for, and that
the resting pad's weight actually registers on a taxel - which is what fails if
the touch_grid site ends up facing the wrong way.

The `body_services` test covers the other out-of-tree plugin, and runs in
synchronous mode because that is the mode pinning down its contract: a pose
written and read straight back, with no step in between, must come back
unchanged. It also checks that the second declaration serves a working pair
rather than merely existing, that a teleport through one instance is visible
through the other, that `/mujoco_reset` restores the body state, and that a
welded body and an unknown body name are both rejected rather than crashing.

These run in CI in their own workflow, `.github/workflows/ci-examples.yml`, which
builds the package and clones `franka_description` from source — it has no jazzy
release — so every example runs there, Franka included.

The interactive MuJoCo viewer is never opened by the tests. `DISABLE_OPENGL`
controls offscreen rendering instead: leave it unset (or `0`) and the GL-backed
sensors render through EGL; set `DISABLE_OPENGL=1` to drop them and run on a
machine with no working GL. CI runs with GL **on**, so the camera and LiDAR paths
are covered there, and a GL test that skips fails the job.

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
