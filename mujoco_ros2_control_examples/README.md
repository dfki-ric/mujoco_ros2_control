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
include/  src/                (the TouchGridSensor and BodyServices example plugins)
test/     per-robot launch smoke tests
```

## Examples

### Franka

Franka arm (optionally with a modular NVIDIA IndustReal benchmark board).
Requires [`franka_description`](https://github.com/frankarobotics/franka_description).

> **Jazzy:** `franka_description` isn't on apt. Clone its `jazzy` branch into
> your workspace `src/` and build it:
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
| `decompose_industreal_peg_trays` | `false` | Convex-decompose the pick/insert tray meshes with CoaCD at launch, so the peg hole survives instead of being filled by MuJoCo's convex-hull collision. Cached per-settings (see below), so this cost is paid once per install. Requires `coacd`/`trimesh`. |
| `load_imrk_table`  | `true`         | Load the IMRK task table as an additional scene model.    |
| `load_gripper`     | `true`         | Attach an end-effector.                                  |
| `ee_id`            | `franka_hand`  | End-effector: `none`, `franka_hand`.                     |
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

Stream settings are in `config/franka/realsense_d435.yaml`. Simulation uses the built-in MuJoCo RGB-D renderer, so `librealsense2`/`realsense2_camera` aren't required, only the description package:

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-realsense2-description
```

Set `load_realsense:=false` to omit the camera and mount entirely. `render_realsense:=false` keeps the mount and camera in the robot description but skips rendering its images. `headless:=true` only disables the interactive viewer window; it does not affect camera rendering, which happens offscreen through EGL regardless.

#### The camera mount mesh

The mount is Franka's own printable bracket, from the *3D Printable Camera Mount Guide* R02241. It's **not** redistributed here (Franka ships no licence with it), so `DOWNLOAD_FRANKA_CAMERA_MOUNT_ASSET` fetches it from `download.franka.de` at configure time and installs one file from it, so the mesh only ever exists in the install tree.

MuJoCo's STL decoder rejects meshes over 200000 faces and the mount is 295768, so `mujoco_ros2_control/scripts/stl_to_obj.py` converts it to OBJ (lossless, since normals are recomputed by every consumer anyway). The download URL isn't pinned to a revision, so CMake checks the archive's SHA256 and warns rather than fails if it changes.

See also [Offline builds](#offline-builds).

### Universal Robots (UR)

UR arm loaded from [`ur_description`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description).

```bash
ros2 launch mujoco_ros2_control_examples ur.launch.py ur_type:=ur10e rviz:=false
```

| Argument  | Default | Description                                                      |
|-----------|---------|--------------------------------------------------------------------------|
| `ur_type` | `ur5e`  | `ur3`, `ur5`, `ur10`, `ur3e`, `ur5e`, `ur10e`, `ur16e`, `ur20`, ... |
| `rviz`    | `true`  | Start RViz.                                                      |

UR ships `.dae` visual meshes and `.stl` collision meshes. `xacro2mjcf.py` symlinks the `.stl` meshes into `<mujoco_files_path>/meshes` and substitutes the collision mesh for the MuJoCo-unsupported `.dae` visual.

#### Several arms in a row

`ur_multi.launch.py` stands several UR arms side by side in one MuJoCo world. By default they're three different arms (a ur3e, a ur5e and a ur10e):

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

Each arm is prefixed with its own type (`ur3e_`, `ur5e_`, `ur10e_`), so every joint, frame, hardware component and controller name says which robot it belongs to (`ur10e_joint_trajectory_controller`, `ur3e_wrist_3_link`, ...). A type repeated in the list gets an occurrence suffix, so `ur_types:=ur5e,ur5e` yields `ur5e_` and `ur5e_2_`.

One description holds every arm: a single `robot_description`, a single MJCF, a single controller manager. Each arm lives under its `tf_prefix`, gets its own `MujocoSystem` hardware component, and can be commanded independently through its own `<type>_joint_trajectory_controller` and `<type>_ft_sensor_broadcaster`; a shared `joint_state_broadcaster` publishes every joint.

`ur_multi.urdf.xacro` is the single source of the prefixes; the launch file reads them back off the generated description rather than deriving them again. An arm is `urdf/ur/ur_arm.urdf.xacro`, the same macro the single-arm example instantiates once, so the two can't drift apart. Because controller names follow the arm types, `config/ur/ur_multi_controllers.yaml` matches the default `ur_types`; a different list needs matching entries there.

Each arm's MuJoCo actuator torque limits come from that arm's own type (`ur_max_effort` in `urdf/ur/mujoco_actuators.urdf.xacro`, copied from `ur_description`'s joint limits), gains scaled from the ones tuned on the ur5e. Previously the ur5e row was hard-coded for every arm, silently giving a ur3e 2.8x its real shoulder torque and capping a ur10e at 45% of its own.

### Unitree H1

Unitree H1 humanoid. The upstream URDF and meshes are **not** redistributed here; they're downloaded from [`unitree_ros`](https://github.com/unitreerobotics/unitree_ros) and patched at build time (`patches/unitree_h1.urdf.patch`).

```bash
ros2 launch mujoco_ros2_control_examples unitree_h1.launch.py
```

See [Offline builds](#offline-builds) to build the other examples without downloading these.

### Unitree G1

Unitree G1 29-DOF humanoid, carrying a depth camera (`/d435/...`) and a LiDAR (`/lidar_head/...`). As with the H1, the upstream URDF and meshes are downloaded from [`unitree_ros`](https://github.com/unitreerobotics/unitree_ros) at build time.

```bash
ros2 launch mujoco_ros2_control_examples unitree_g1.launch.py
```

The 29 joints are commanded through five `JointGroupPositionController`s (`left_leg`, `right_leg`, `waist`, `left_arm`, `right_arm`), each named `<group>_position_controller`. Each forwards a position setpoint per joint; the stiff PD that turns it into torque lives in the hardware (`kp`/`kd` per joint in `urdf/unitree_g1/ros2_control.urdf.xacro`), so the robot holds its start pose until a command arrives.

```bash
# left_arm joints, in config order: shoulder p/r/y, elbow, wrist r/p/y
ros2 topic pub --once /left_arm_position_controller/commands \
  std_msgs/msg/Float64MultiArray '{data: [0.3, 0.25, 0.0, 1.5, 0.15, 0.0, 0.0]}'
```

#### Low-level interface (`unitree_hg` LowCmd/LowState)

`low_level:=true` drives the robot the way the real G1 is driven: a policy sends `unitree_hg/LowCmd` on `/lowcmd` with a per-motor position, velocity, torque and PD gain, and reads `unitree_hg/LowState` on `/lowstate`. No controller is involved: the `UnitreeHgLowLevel` plugin (`src/unitree_hg_low_level.cpp`) runs inside the simulation as a top-level `<mujoco_ros2_plugin>` and closes the motor PD loop straight into `mjData::qfrc_applied`:

```
tau = kp * (q_des - q) + kd * (dq_des - dq) + tau_ff
```

It runs on the simulation thread, so it closes on every physics step rather than once per controller-manager period, matching how the real motor drivers run their PD faster than the policy that sends targets. The joints then export no command interfaces (`urdf/unitree_g1/ros2_control.urdf.xacro` drops them when `low_level` is set); every state interface and broadcaster stays as it was.

The messages have no binary release and building them under our own package name would change the type a policy publishes, so `unitree_hg` is pulled out of the upstream `unitree_ros2` repo via the `.repos` file next to this README:

```bash
cd <workspace>
vcs import --input src/mujoco_ros2_control/mujoco_ros2_control_examples/.repos src/
mv src/unitree_ros2/cyclonedds_ws/src/unitree/unitree_hg src/unitree_hg
rm -rf src/unitree_ros2
sed -i '/<build_depend>rosidl_default_generators<\/build_depend>/a\  <build_depend>rosidl_generator_dds_idl</build_depend>' \
  src/unitree_hg/package.xml
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --packages-up-to mujoco_ros2_control_examples
```

The `sed` line patches a gap in `unitree_hg`'s own `package.xml` (its CMake needs `rosidl_generator_dds_idl` but never declares it, so `rosdep install` skips it and the build fails at CMake configure). The Dockerfile and `{humble,jazzy}-examples.yml` apply the same patch. Skip it and the plugin is simply left out of the build; `rosdep install` then needs `--skip-keys unitree_hg`.

```bash
ros2 launch mujoco_ros2_control_examples unitree_g1.launch.py low_level:=true
ros2 topic hz /lowstate
```

Joints stay **slack** (zero torque, not held) until the first `LowCmd` arrives, and again after `/mujoco_reset` (the plugin detects simulation time moving backwards). Holding a pose is a publisher's job, and the example launches one: `config/unitree_g1/hold_start_message.yaml` is a `LowCmd` holding the start pose, published on `/lowcmd` at 50 Hz so the robot stands there instead of collapsing while a policy comes up. `q`/`kp`/`kd` in that file match the ros2_control block's own values, so the robot stands the same way in both modes; `crc` is ignored by the plugin.

It keeps publishing rather than sending once because the hold starts before the simulation and is already on the wire when the plugin subscribes, whereas `--wait-matching-subscriptions` only publishes after the subscription count updates (checked every 100 ms) - long enough for the robot to drop off its spawn height and topple. A one-shot hold stood the robot up 1 run in 4; the repeating one, 4 in 4.

Nothing stands the hold down, so once a policy starts it's one of two publishers on `/lowcmd`; whichever publishes faster wins, since the plugin applies the last command received each step, and a G1 policy at 500 Hz beats this 50 Hz hold. To skip it, use `hold_pose:=false` and command `/lowcmd` yourself from the first message:

```bash
ros2 launch mujoco_ros2_control_examples unitree_g1.launch.py \
  low_level:=true hold_pose:=false
```

Parameters live in `config/unitree_g1/unitree_g1_controllers.yaml` under `unitree_g1_low_level`, and each also accepts a `<param>` of the same name on the declaration:

| Parameter                | Default              | Meaning                                                              |
|--------------------------|-----------------------|------------------------------------------------------------------------|
| `joints`                 | -                    | Joints to drive, in LowCmd motor order. Required.                     |
| `motor_indices`          | `0..N-1`             | `motor_cmd`/`motor_state` slot per joint.                            |
| `effort_limits`          | empty                | Torque clamp per joint; one value applies to all, empty = unclamped.  |
| `imu_sensor_name`        | `imu_in_pelvis`      | IMU site; names the three MuJoCo sensors. `""` leaves `imu_state` zero. |
| `command_topic`          | `/lowcmd`            | LowCmd topic.                                                        |
| `state_topic`            | `/lowstate`          | LowState topic.                                                      |
| `mode_machine`           | `0`                  | Echoed in LowState; the SDK examples copy it back into LowCmd.        |
| `rate`                   | `500.0`              | LowState publish rate, simulated Hz. Doesn't affect the PD loop.      |
| `joy_topic`, `joy.*`     | `/joy`, unmapped     | Gamepad mapping packed into `LowState::wireless_remote`.              |

Torque isn't clamped unless `effort_limits` is set (MuJoCo doesn't limit `qfrc_applied` on its own), and there's no controller lifecycle since the plugin lives as long as the simulation. It's simulation-only by nature: a `ControllerInterface` runs unchanged against the real robot, this replaces the simulation half of that pair.

#### Stepping-stones terrain

By default this example spawns on a Voronoi stepping-stones field generated by `mujoco_ros2_control`'s `prepare_terrain.py`: stones of varying height with tilted tops, separated by gaps and holes, over a 1 m pit. The defaults pave a 30x30 m field with a few hundred stones. Pass `terrain:=flat` for a plain ground plane.

```bash
ros2 launch mujoco_ros2_control_examples unitree_g1.launch.py
ros2 launch mujoco_ros2_control_examples unitree_g1.launch.py terrain:=flat
ros2 launch mujoco_ros2_control_examples unitree_g1.launch.py terrain_difficulty:=0.3
```

| Argument             | Default  | Meaning                                                                   |
|----------------------|----------|------------------------------------------------------------------------------|
| `terrain`            | `stones` | A `stones` field, or a `flat` ground plane.                               |
| `terrain_difficulty` | `0.5`    | `0` = large, close, level stones; `1` = small, far apart, uneven.         |
| `terrain_seed`       | `42`     | RNG seed; the same seed always yields the same field.                      |
| `terrain_collider`   | `mesh`   | `mesh` = one convex prism per stone, `hfield` = a single heightfield geom. |
| `terrain_field_half` | `15.0`   | Half-extent of the square field, m (so 30x30 m).                          |
| `terrain_height_var` | `0.10`   | How far stone tops sit above/below `z=0` at difficulty 1, m.              |
| `terrain_tilt_deg`   | `8.0`    | How far stone tops tilt out of horizontal at difficulty 1, degrees.        |
| `terrain_hole_frac`  | `0.15`   | Fraction of stones dropped at difficulty 1, leaving holes. `0` = none.     |

Difficulty scales every knob together, but each is also its own knob (a hard field can still be made flat with `terrain_height_var:=0 terrain_tilt_deg:=0`, or unbroken with `terrain_hole_frac:=0`). The spawn platform at the origin stays flat and level whatever the surrounding terrain does. `mesh` gives crisp edges and exact tilted tops at one collision geom per stone; `hfield` is a single geom, but MuJoCo interpolates between samples so vertical walls become slopes one cell wide.

The generator is also a standalone CLI, useful for inspecting a field before launching:

```bash
ros2 run mujoco_ros2_control prepare_terrain.py --out-dir /tmp/terrain --difficulty 1.0
```

The scene it writes stands in for `mujoco_ros2_control`'s `mjcf/scene.xml`; pass one or the other to `xacro2mjcf`, never both, since they define the same skybox, groundplane material and floor geom.

### Tactile pad (touch_grid)

A 6 cm pad on a prismatic joint, resting on the ground plane, with a MuJoCo `touch_grid` sensor imaging its contact face. This example is about the sensor rather than the robot: a worked example of a `MujocoRos2ControlSensorInterface` plugin living **outside** `mujoco_ros2_control`.

```bash
ros2 launch mujoco_ros2_control_examples tactile.launch.py
ros2 launch mujoco_ros2_control_examples tactile.launch.py headless:=true
ros2 topic echo /pad_touch/touch_grid
```

A `touch_grid` emits `nchannel * width * height` values per step, which doesn't fit ros2_control's scalar state interfaces, so [`TouchGridSensor`](src/touch_grid_sensor.cpp) exports none and publishes a `std_msgs/Float64MultiArray` instead. The array is channel-major with `layout.dim` labelled `channel`, `height`, `width`, indexable as `data[k * width * height + j * width + i]`. Channels are the force components `[normal, tangent, tangent]` followed by torque `[torsional, rolling, rolling]`, truncated to the model's `nchannel` (the example declares 3, so it publishes forces only).

The MJCF side needs the engine plugin in an `<extension>` block, and the URDF names the sensor plugin ([`urdf/tactile/tactile_pad.urdf.xacro`](urdf/tactile/tactile_pad.urdf.xacro)):

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

<!-- outside <ros2_control>: it exports no state interfaces, it publishes a
     topic, so it gets its own node named pad_touch and its own thread -->
<mujoco_ros2_plugin name="pad_touch"
                    plugin="mujoco_ros2_control_examples/TouchGridSensor">
    <param name="site">pad_touch_site</param>
</mujoco_ros2_plugin>
```

Everything else is a parameter of the `pad_touch` node, defaulting to the `<param>` of the same name on the declaration:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `site` | the declaration name | The site the `touch_grid` is attached to. |
| `topic` | `<declaration name>/touch_grid` | Topic to publish on. |
| `frame_id` | the site name | Frame the taxel values are expressed in. Informational only, `Float64MultiArray` carries no header. |
| `publish` | `true` | Set `false` to step the sensor without producing topic traffic. |
| `rate` | `100.0` | Sampling rate in simulated Hz. `<param>` only: the base class reads it before the plugin is configured. |

```yaml
pad_touch:
  ros__parameters:
    topic: fingertip/touch
    publish: true
```

Three things to watch when copying this:

- **The site images along its negative z.** It must sit *above* the contact face looking down through the body. Placed flush with the face, or flipped with `zaxis="0 0 -1"`, it sees nothing and publishes a correctly shaped block of zeros.
- **The pad has to be free to move.** A body welded to the world joins the world's weld group, and MuJoCo generates no contacts within a weld group.
- **`mujoco.sensor.touch_grid` is a MuJoCo engine plugin** and must be loaded before the model is compiled. `mujoco_ros2_control` does that by default from its own `lib/mujoco_plugin` directory; see [MuJoCo Engine Plugins](../mujoco_ros2_control/README.md#mujoco-engine-plugins).

For what writing such a plugin involves, see [Writing a Sensor Plugin](../mujoco_ros2_control/README.md#writing-a-sensor-plugin). The other examples use the plugins shipped with `mujoco_ros2_control`: `ImuSensor` and `PoseSensor` on the Unitree robots, `ForceTorqueSensor` on Franka and UR.

### Body services (read and teleport bodies)

Two probe boxes and no robot. Like the tactile pad, this example is about the plugin: reaching into a running simulation from outside, and a plugin that brings its **own generated service types** rather than reusing an existing interface package.

```bash
ros2 launch mujoco_ros2_control_examples body_services.launch.py
ros2 launch mujoco_ros2_control_examples body_services.launch.py headless:=true

ros2 service call /mujoco_get_body_state \
    mujoco_ros2_control_examples/srv/GetBodyState "{body_name: probe_float}"

ros2 service call /mujoco_set_body_pose \
    mujoco_ros2_control_examples/srv/SetBodyPose \
    "{body_name: probe_float, x: 0.5, y: -0.25, z: 1.75, qw: 1.0}"
```

[`BodyServices`](src/body_services.cpp) serves two services: one reads a body's world pose and twist, the other teleports a body with a single free joint. Teleporting is for *driving* a simulation (an external training loop, a scripted scenario) rather than part of simulating one, hence it lives here and not in `mujoco_ros2_control`.

Declared twice in [`urdf/body_services/probe_bodies.urdf.xacro`](urdf/body_services/probe_bodies.urdf.xacro), which is the point of the parameters:

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

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `get_body_state_service` | `mujoco_get_body_state` | Name of the read service. |
| `set_body_pose_service` | `mujoco_set_body_pose` | Name of the teleport service. |

Both default to the `<param>` of the same name on the declaration, so a node's name doesn't prefix service names (only its namespace does): several models in one simulation get their own pair instead of fighting over the defaults. Both instances read and write the same simulation, so `/alt/set_body_pose` followed by `/mujoco_get_body_state` sees the move.

Three things to watch when copying this:

- **Only a body whose sole joint is free can be teleported.** `probe_float` can be; `probe_fixed` is welded to the world and the set service rejects it.
- **Gravity is off in this model**, so `probe_float` stays where it's put. With gravity on, a teleported free body starts falling on the next step.
- **The write isn't deferred to the next step.** It happens in the service callback, under both simulation locks, followed by `mj_forward()` - a client that sets a pose and reads it straight back sees what it just wrote. Try it with `synchronous:=true`.

| Argument      | Default | Description                                                    |
|---------------|---------|----------------------------------------------------------------|
| `headless`    | `false` | Run without the MuJoCo viewer window.                          |
| `synchronous` | `false` | Start paused; advance only on `/mujoco_step_simulation`.        |

For what writing such a plugin involves, see [Writing a Sensor Plugin](../mujoco_ros2_control/README.md#writing-a-sensor-plugin).

## Offline builds

Four asset groups are fetched at configure time and can each be turned off:

| CMake option                        | Default | Fetches                                       |
|--------------------------------------|---------|-----------------------------------------------|
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

With the peg assets off, launch Franka without the peg rows (`load_industreal_board:=false`, or a `task_board_config` with `*_peg_*` disabled). With the camera mount off, launch with `load_realsense:=false`. The gears are in-tree and always available.

## Tests

`test/` holds one launch smoke test per robot: each converts the robot to MJCF, starts `mujoco_ros2_control` headless, and asserts the node comes up. A test self-skips when its robot description package isn't installed, or (for the Unitree robots) when the upstream assets weren't downloaded.

Four tests check more than that the node starts. The Unitree G1 test brings the robot up on the default stepping-stones terrain. The `ur_multi` test asserts every arm reached the merged model, the bases are evenly spaced, each arm behind a prefix is the type that prefix names, and the controller manager instantiated one hardware component per arm. The `tactile` test covers the plugin path end to end: `pluginlib` resolving the out-of-tree `TouchGridSensor`, the published array's taxel count and layout, and that the resting pad's weight actually registers on a taxel. The `body_services` test runs in synchronous mode (the mode that pins down its contract: a pose written and read back with no step in between must come back unchanged), and checks the second declaration serves a working pair, a teleport through one instance is visible through the other, `/mujoco_reset` restores body state, and a welded body or unknown name are rejected rather than crashing.

These run in CI in `.github/workflows/{humble,jazzy}-examples.yml`, which builds the package and clones `franka_description` from source (it has no jazzy release), so every example runs there, Franka included.

The interactive MuJoCo viewer is never opened by the tests. `DISABLE_OPENGL` controls offscreen rendering: leave it unset (or `0`) and the GL-backed sensors render through EGL; set `DISABLE_OPENGL=1` to drop them on a machine with no working GL. CI runs with GL **on**, so the camera and LiDAR paths are covered.

```bash
colcon test --packages-select mujoco_ros2_control_examples
DISABLE_OPENGL=1 colcon test --packages-select mujoco_ros2_control_examples   # headless
```

## Task table assets

See [`urdf/industreal/`](urdf/industreal) and [`meshes/industreal/`](meshes/industreal). Collision meshes were generated with [Phobos](https://github.com/dfki-ric/phobos) and [CoACD](https://github.com/SarahWeiii/CoACD); use `mujoco_ros2_control/scripts/run_coacd.py` to decompose a mesh into collision-friendly components. The URDF-to-MJCF converter picks the pieces up automatically: a `<collision>` mesh is replaced by every file in a same-named folder (extension stripped) beside it, which is what `run_coacd.py`'s `decompose_mesh()` writes. The Franka example does this on demand for the peg trays (`decompose_industreal_peg_trays` above), since a single collision mesh takes its convex hull and fills in the peg-shaped hole a tray is meant to have.

`decompose_mesh()` caches by settings, not just by folder presence: it writes a `<name>.coacd_settings.json` beside the output folder recording the input hash and every CoaCD argument. A call whose settings and input hash match is skipped; a call whose folder exists but settings differ rebuilds it. A folder with no settings file (`meshes/industreal/gears/`, generated offline with Phobos) is left alone either way. A `<name>.coacd_ongoing` marker sits beside a mesh while it's being worked on, useful to check which mesh is running mid-batch or which one didn't finish after a crash; the output folder is built under a temporary name and renamed into place last, so a killed run never leaves a half-written folder that looks finished.

`coacd_node.py` wraps `decompose_mesh()` as a ROS node the way `xacro2mjcf.py` is wrapped: one process for every pending tray. `franka.launch.py` chains it before the model converter with `RegisterEventHandler(OnProcessExit(...))`, the same way the converter chains into the simulator, since CoaCD takes tens of seconds per mesh and calling it inline would freeze the whole launch. `prepare_terrain_node.py` wraps `prepare_terrain.py`'s `build_scene()` the same way for the G1's stepping-stones terrain.

`coacd` and `trimesh` have no apt/rosdep package, and the `ros:jazzy` image's system Python is [PEP 668](https://peps.python.org/pep-0668/) externally-managed, so a bare `pip install` is refused. Don't reach for `--break-system-packages`: `coacd`/`trimesh` pull in an unpinned numpy, and installing that system-wide shadows `python3-numpy` for every process on the machine, while `trimesh` also imports the apt-built `python3-scipy` (built against numpy 1.x) at import time, so a numpy 2.x install breaks scipy for everyone. Use a venv instead, pinned to numpy 1.x, bridged onto `PYTHONPATH` rather than activated:

```bash
uv venv .venv --python 3.12
uv pip install --python .venv/bin/python "numpy<2" coacd trimesh
export PYTHONPATH="$(pwd)/.venv/lib/python3.12/site-packages:$PYTHONPATH"
```

The bridge matters because `coacd_node.py` runs via `ros2 launch` under its own shebang, resolving the ROS-sourced system Python rather than the venv's interpreter, so the packages need to be importable via `PYTHONPATH`. This is what the `Dockerfile` and `{humble,jazzy}-examples.yml` do (as a Docker `ENV` and a `$GITHUB_ENV` entry). For local development, add the same `PYTHONPATH` export to whatever script sources `install/setup.bash`.

`test/test_franka_industreal_decompose_example.test.py` exercises this end to end: launches `franka.launch.py` with the flag set and asserts the tray meshes actually got decomposed (an undecomposed tray still loads fine in MuJoCo, so the node coming up isn't enough of a check). It self-skips when `coacd`/`trimesh` aren't importable. CoaCD is slow at default settings, budget several minutes for a cold run over all twelve tray meshes; the launch argument caches its output, so this is paid once per install.

The default `config/franka/industreal_task_board.yaml` describes one unified board: a medium-dark-grey optical breadboard, the 8/12/16 mm round and rectangular peg rows, and the gear base plus its three movable gears. Pass an alternate file without changing the package:

```bash
ros2 launch mujoco_ros2_control_examples franka.launch.py \
  task_board_config:=/absolute/path/to/my_task_board.yaml
```

### Asset provenance and licences

Three of the four asset groups are downloaded at build time and never redistributed here: the Unitree H1 URDF and meshes, the IndustReal peg and tray meshes from [`IsaacGymEnvs`](https://github.com/isaac-sim/IsaacGymEnvs/tree/main/isaacgymenvs/tasks/industreal), and the Franka camera mount. See [Offline builds](#offline-builds).

The gears are the exception, and the one asset group **not covered by this package's licence**. They're a derivative work of [`IndustRealKit`](https://github.com/NVlabs/industrealkit) CAD, can't be re-fetched, and carry a NVIDIA License limiting them and any derivative to **non-commercial research or evaluation use**. Details are in [`meshes/industreal/README.md`](meshes/industreal/README.md); the licence text is in `meshes/industreal/licenses/`. Build with a `task_board_config` whose `gears.enabled` is `false` for a board without that restriction.

The board doesn't model IndustReal's electrical-connector task: its plugs and sockets are commercial parts NVIDIA identifies by part number but doesn't redistribute, so only the printable trays are public.

## License

BSD-3-Clause, Copyright (c) 2025 DFKI GmbH, Robotics Innovation Center, the same terms as the rest of this repository. See [`LICENSE`](LICENSE).

One exception: the gear meshes under `meshes/industreal/gears/` are a derivative of NVIDIA IndustRealKit CAD and stay under the NVIDIA License, restricted to non-commercial research or evaluation use. They're the only files in this package not covered by the line above. Everything else pulled from third parties (the Unitree H1 description, the IndustReal peg meshes, the Franka camera mount) is downloaded at build time and never redistributed here, so no third-party terms attach to this repository for them. See [Asset provenance and licences](#asset-provenance-and-licences).
