# URDF Configuration (for usage with xacro2mjcf script)

To use this package with an existing robot description (URDF or Xacro), create a **Xacro wrapper file** that merges:

- your existing robot description,
- the MuJoCo configuration, and
- the ROS 2 Control configuration.

For reference, see the `urdf` directories in the provided examples ([franka](https://github.com/dfki-ric/mujoco_ros2_control/blob/main/mujoco_ros2_control_examples/urdf/franka/franka.urdf.xacro), [unitree](https://github.com/dfki-ric/mujoco_ros2_control/blob/main/mujoco_ros2_control_examples/urdf/unitree_h1/unitree_h1.urdf.xacro)).

---

## MuJoCo-Specific Elements

```xml
<mujoco>
    <!-- Compiler options:
         https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler -->
    <compiler
        meshdir="/tmp/mujoco/meshes"
        discardvisual="true"
        autolimits="false"
        balanceinertia="true"/>

    <!-- Global simulation options:
         https://mujoco.readthedocs.io/en/stable/XMLreference.html#option -->
    <option
        integrator="implicitfast"
        gravity="0 0 -9.81"
        impratio="10"
        cone="elliptic"
        solver="Newton">
        <flag multiccd="enable"/>
    </option>

    <!-- Add elements/tags to an MJCF body or any of its children -->
    <reference name="${prefix}left_inner_finger">
        <body gravcomp="1"/>            <!-- Enable gravity compensation -->
        <joint damping="10"/>           <!-- Add damping to all child joints -->

        <!-- Modify a child geom with the given name -->
        <geom
            name="geom1"
            friction="0.7"
            mass="0"
            priority="1"
            solimp="0.95 0.99 0.001"
            solref="0.004 1"/>
    </reference>

    <!-- Define an RGB-D camera:
         https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-camera -->
    <reference name="camera_link">
        <camera
            name="camera"
            mode="fixed"
            fovy="45"
            quat="0.5 0.5 -0.5 -0.5"/>
    </reference>

    <!-- Camera pose sensors relative to the world frame -->
    <sensor>
        <framepos
            name="camera_link_pose"
            objtype="xbody"
            objname="camera_link"
            reftype="body"
            refname="world"/>

        <framequat
            name="camera_link_quat"
            objtype="xbody"
            objname="camera_link"
            reftype="body"
            refname="world"/>
    </sensor>

    <!-- Actuator definition:
         https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator -->
    <actuator>
        <position
            name="pos_finger_joint1"
            joint="${arm_id}_finger_joint1"
            kp="1000"
            forcelimited="true"
            forcerange="-120 120"
            ctrllimited="true"
            ctrlrange="0 0.04"
            user="1"/>
    </actuator>
</mujoco>
```

## ROS 2 Control Hardware Example
```xml
<ros2_control name="${prefix}${name}" type="system">
    <hardware>
        <plugin>mujoco_ros2_control/MujocoSystem</plugin>
    </hardware>

    <!-- Joint with position + velocity + acceleration PID control -->
    <joint name="joint1">
        <command_interface name="position"/>
        <command_interface name="velocity"/>
        <command_interface name="acceleration"/>

        <param name="kp">1000.0</param>
        <param name="ki">0.0</param>
        <param name="kd">0.01</param>

        <!-- Only required when using position + velocity control -->
        <param name="kvff">0.01</param>

        <!-- Required when using position + velocity + acceleration control -->
        <param name="kaff">0.01</param>

        <state_interface name="position"/>
        <state_interface name="velocity"/>
    </joint>

    <!-- Joint with torque (effort) control -->
    <joint name="joint2">
        <command_interface name="effort"/>

        <state_interface name="position">
            <param name="initial_value">1.0</param>
        </state_interface>

        <state_interface name="velocity">
            <param name="initial_value">0.0</param>
        </state_interface>
    </joint>
</ros2_control>
```

## ROS 2 Control Sensor Interfaces

Sensors are declared inside the `<ros2_control>` block and matched to MuJoCo sensors via a `<param>` naming the MuJoCo object (site, xbody, geom, ...) they're attached to.

| Path | Selected by | Can do |
|---|---|---|
| **Sensor plugin** (recommended) | a `<param name="plugin">` naming a class | whatever the plugin implements: state interfaces, topics, or both |
| **Built-in classifier** (deprecated) | no `plugin` param | **IMU**, **Force/Torque** and **Pose** only, inferred from the state interface names |

The three built-in types also ship as plugins with the same state interfaces, so the standard broadcasters work either way:

| Plugin class | Sensor type | Broadcaster |
|---|---|---|
| `mujoco_ros2_control/ImuSensor` | IMU | `imu_sensor_broadcaster` |
| `mujoco_ros2_control/ForceTorqueSensor` | Force/Torque | `force_torque_sensor_broadcaster` |
| `mujoco_ros2_control/PoseSensor` | Pose | `pose_broadcaster` |

The classifier keeps working for existing models but isn't scheduled to grow: it dispatches on a substring match over state interface names, so it can express only these three types and never a sensor whose output doesn't fit ros2_control's scalar state interfaces. Prefer a plugin for new sensors. A plugin needs no change to this package, see [Writing a Sensor Plugin](#writing-a-sensor-plugin).

Note: a plugin must be named by a `<param>`, not an attribute on `<sensor>` - ros2_control's URDF parser discards unknown attributes there.

### IMU Sensor

Reads orientation (framequat), angular velocity (gyro) and linear acceleration (accelerometer) from a MuJoCo site. Use with [imu_sensor_broadcaster](https://control.ros.org/rolling/doc/ros2_controllers/imu_sensor_broadcaster/doc/userdoc.html). Each of the three MuJoCo sensors is optional.

```xml
<ros2_control name="MySystem" type="system">
    <hardware>
        <plugin>mujoco_ros2_control/MujocoSystem</plugin>
    </hardware>

    <!-- ... joints ... -->

    <sensor name="imu_in_pelvis">
        <param name="plugin">mujoco_ros2_control/ImuSensor</param>
        <param name="site">imu_in_pelvis</param>
        <state_interface name="orientation.x"/>
        <state_interface name="orientation.y"/>
        <state_interface name="orientation.z"/>
        <state_interface name="orientation.w"/>
        <state_interface name="angular_velocity.x"/>
        <state_interface name="angular_velocity.y"/>
        <state_interface name="angular_velocity.z"/>
        <state_interface name="linear_acceleration.x"/>
        <state_interface name="linear_acceleration.y"/>
        <state_interface name="linear_acceleration.z"/>
    </sensor>
</ros2_control>
```

The corresponding MuJoCo sensors:
```xml
<mujoco>
    <reference name="pelvis">
        <site name="imu_in_pelvis" size="0.01" pos="0 0 0"/>
    </reference>

    <sensor>
        <gyro name="imu_in_pelvis-angular-velocity" site="imu_in_pelvis" noise="5e-4" cutoff="34.9"/>
        <accelerometer name="imu_in_pelvis-linear-acceleration" site="imu_in_pelvis" noise="1e-2" cutoff="157"/>
        <framequat name="imu_in_pelvis-orientation" objtype="site" objname="imu_in_pelvis"/>
    </sensor>
</mujoco>
```

Controller configuration (`controllers.yaml`):
```yaml
controller_manager:
  ros__parameters:
    imu_broadcaster:
      type: imu_sensor_broadcaster/IMUSensorBroadcaster

imu_broadcaster:
  ros__parameters:
    sensor_name: "imu_in_pelvis"
    frame_id: "imu_in_pelvis"
```

### Force/Torque Sensor

Reads force and torque from a MuJoCo site. Use with [force_torque_sensor_broadcaster](https://control.ros.org/rolling/doc/ros2_controllers/force_torque_sensor_broadcaster/doc/userdoc.html). Either MuJoCo sensor may be absent.

```xml
<sensor name="ft_sensor">
    <param name="plugin">mujoco_ros2_control/ForceTorqueSensor</param>
    <param name="site">ft_site</param>
    <state_interface name="force.x"/>
    <state_interface name="force.y"/>
    <state_interface name="force.z"/>
    <state_interface name="torque.x"/>
    <state_interface name="torque.y"/>
    <state_interface name="torque.z"/>
</sensor>
```

```xml
<mujoco>
    <reference name="link7">
        <site name="ft_site" pos="0 0 0.107" quat="0.92388 0 0 -0.382683"/>
    </reference>
    <sensor>
        <force name="ft_site_force" site="ft_site"/>
        <torque name="ft_site_torque" site="ft_site"/>
    </sensor>
</mujoco>
```

```yaml
controller_manager:
  ros__parameters:
    ft_sensor_broadcaster:
      type: force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster

ft_sensor_broadcaster:
  ros__parameters:
    sensor_name: "ft_sensor"
    frame_id: "link7"
```

### Pose Sensor

Reads position (framepos) and orientation (framequat) of a MuJoCo body. Use with [pose_broadcaster](https://control.ros.org/rolling/doc/ros2_controllers/pose_broadcaster/doc/userdoc.html), useful for a floating-base robot's base link pose. Either MuJoCo sensor may be absent.

```xml
<sensor name="pelvis_pose">
    <param name="plugin">mujoco_ros2_control/PoseSensor</param>
    <param name="body">pelvis</param>
    <state_interface name="position.x"/>
    <state_interface name="position.y"/>
    <state_interface name="position.z"/>
    <state_interface name="orientation.x"/>
    <state_interface name="orientation.y"/>
    <state_interface name="orientation.z"/>
    <state_interface name="orientation.w"/>
</sensor>
```

```xml
<mujoco>
    <sensor>
        <framepos name="pelvis_pose" objtype="body" objname="pelvis" reftype="body" refname="world"/>
        <framequat name="pelvis-orientation" objtype="body" objname="pelvis" reftype="body" refname="world"/>
    </sensor>
</mujoco>
```

```yaml
controller_manager:
  ros__parameters:
    pelvis_pose_broadcaster:
      type: pose_broadcaster/PoseBroadcaster

pelvis_pose_broadcaster:
  ros__parameters:
    pose_name: "pelvis_pose"
    frame_id: "world"
    tf:
      enable: true
      child_frame_id: "pelvis"
```

### Sensor Matching

The `<param>` inside a `<sensor>` block tells the plugin which MuJoCo object to look for. Supported keys, checked in this order: `site`, `body`, `geom`, `camera`, `light`, `frame`. If none is given, the sensor `name` is used as the match key.

For example, `<param name="site">imu_in_pelvis</param>` matches all MuJoCo sensors whose object name is `imu_in_pelvis`. The three shipped plugins reuse these keys, so a model written for the built-in classifier keeps matching once a `plugin` param is added.

### Writing a Sensor Plugin

A sensor plugin implements
[`mujoco_ros2_control::MujocoRos2ControlSensorInterface`](include/mujoco_ros2_control/mujoco_ros2_control_sensor_interface.hpp)
and is loaded by name through `pluginlib`. Use this to add a sensor type the built-in classifier can't express, or to expose output that doesn't fit ros2_control's scalar `StateInterface` model - a MuJoCo
`touch_grid`, for instance, yields `nchannel * width * height` values per step, which belongs on a topic rather than several hundred interfaces.

Four methods, of which only the first two are required:

| Method | Called | Purpose |
|---|---|---|
| `registerSensor(node, mujoco_model, sensor_info, state_interfaces)` | once, while the hardware component initialises | read the `<param>` entries, resolve MuJoCo sensor addresses, append state interfaces. Return `false` to reject the declaration (after logging why); the sensor is skipped and the rest keep loading. |
| `read(mujoco_data)` | every `read()` cycle, in the control loop | copy this step's values out of `mjData::sensordata` |
| `activate()` / `deactivate()` | from the component's `on_activate()` / `on_deactivate()` | start and stop publishing |

Rules for a plugin that behaves in the control loop:

- **Storage must not move.** Every `double` exported as a `StateInterface` is handed out as a pointer, so it must live in the plugin object itself, never a container that can reallocate.
- **`read()` runs in the control loop.** Keep it allocation-free. Publish only through `realtime_tools::RealtimePublisher`, never a plain publisher, which can block.
- **The `node` passed to `registerSensor()` is the simulation node**, already spinning, so declaring parameters and creating publishers there is fine. It's deliberately not the hardware component's own node: `get_node()` on the component still returns `nullptr` at that point.
- **Nothing may link the plugin library.** `pluginlib` has to `dlopen` it itself, or `class_loader` registers the factories outside its own bookkeeping and reports "no factory exists for it" when the class is requested.

Helpers for reading configuration and resolving MuJoCo sensor addresses are in
[`sensor_lookup.hpp`](include/mujoco_ros2_control_plugins/sensor_lookup.hpp):
`get_param()`, `get_bool_param()`, `resolve_object_name()`, `find_sensor_adr()`
and `export_state_interfaces()`.

The `declare_*_param()` helpers in the same header read one key from both the node parameter and the `<param>` on the declaration, node parameter wins, falling back to the value passed in. Prefer them over `get_param()` for anything a user might retune, so a robot can be described entirely in its URDF and still be reconfigured from the controller YAML.

```cpp
using mujoco_ros2_control_plugins::declare_param;
using mujoco_ros2_control_plugins::declare_double_param;

topic_ = declare_param(node, sensor_info, "topic", sensor_info.name + "/touch");
cutoff_ = declare_double_param(node, sensor_info, "cutoff", 20.0);
```

One helper per type - `declare_param()` (string), `declare_bool_param()`, `declare_int_param()`, `declare_double_param()`, and the three `declare_*_array_param()` - each callable once per key, from `registerSensor()`/`configure()` only. Where the key lands in the YAML follows from the node: a `<mujoco_ros2_plugin>` has its own, named after the declaration; a `<sensor>` inside `<ros2_control>` shares the simulation node, so its keys go under `mujoco_ros2_control`.

#### Out-of-tree packages

`mujoco_ros2_control` installs its public headers and re-exports the MuJoCo it fetched, so a plugin can live in any package. The header
[`touch_grid_sensor.hpp`](../mujoco_ros2_control_examples/include/mujoco_ros2_control_examples/touch_grid_sensor.hpp)
and its
[implementation](../mujoco_ros2_control_examples/src/touch_grid_sensor.cpp) in
`mujoco_ros2_control_examples` are a complete worked example.

`package.xml`:
```xml
<depend>mujoco_ros2_control</depend>
<depend>hardware_interface</depend>
<depend>pluginlib</depend>
<depend>rclcpp</depend>
<depend>realtime_tools</depend>
```

Register the class at the bottom of the `.cpp`:
```cpp
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    my_package::MySensor, mujoco_ros2_control::MujocoRos2ControlSensorInterface)
```

Describe it in `my_package_plugins.xml` (`path` is the library name without the `lib` prefix or `.so` suffix):
```xml
<library path="my_package_plugins">
    <class
            name="my_package/MySensor"
            type="my_package::MySensor"
            base_class_type="mujoco_ros2_control::MujocoRos2ControlSensorInterface">
        <description>What this sensor reads and how it reports it.</description>
    </class>
</library>
```

`CMakeLists.txt` (`find_package(mujoco REQUIRED)` resolves to the same `libmujoco.so` the simulator uses, since `mujoco_ros2_control` re-exports it):
```cmake
find_package(mujoco_ros2_control REQUIRED)
find_package(mujoco REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(my_package_plugins SHARED src/my_sensor.cpp)
ament_target_dependencies(my_package_plugins
    mujoco_ros2_control hardware_interface pluginlib rclcpp realtime_tools)
target_link_libraries(my_package_plugins mujoco::mujoco)

install(TARGETS my_package_plugins LIBRARY DESTINATION lib)

pluginlib_export_plugin_description_file(
    mujoco_ros2_control my_package_plugins.xml)
```

The description file is exported **against `mujoco_ros2_control`**, not the package that owns the plugin - that's what puts the class on the list the simulator's class loader searches. Then name the class from the URDF:

```xml
<sensor name="fingertip_touch">
    <param name="plugin">my_package/MySensor</param>
    <param name="site">touch_site</param>
</sensor>
```

A plugin that fails to load, or whose `registerSensor()` returns `false`, is logged as an error and skipped; the simulation comes up without it. Check the node's output for `loaded plugin` lines to confirm what was picked up.

## MuJoCo Engine Plugins

MuJoCo's own [engine plugins](https://mujoco.readthedocs.io/en/stable/programming/extension.html) are separate from the sensor plugins above: they extend the physics engine and are referenced from an `<extension>` block in the MJCF, which `xacro2mjcf.py` copies out of the `<mujoco>` section of your description.

```xml
<mujoco>
    <extension>
        <plugin plugin="mujoco.sensor.touch_grid"/>
    </extension>

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
```

`libmujoco` never scans for plugin libraries itself, so they must be `dlopen`ed before the model is compiled, or the compiler can't resolve `<extension>`. The node does that on startup, controlled by two read-only parameters (set at launch):

| Parameter | Default | Description |
|---|---|---|
| `mujoco_plugin_directories` | *(empty)* | Directories scanned before the model is compiled; every shared library in each is loaded. Left empty, the package's own `lib/mujoco_plugin` directory is scanned, holding the plugins shipped with MuJoCo: `mujoco.sensor.touch_grid`, `mujoco.elasticity.cable`, `mujoco.pid` and the `mujoco.sdf.*` set. |
| `mujoco_plugin_libraries` | *(empty)* | Explicit library paths, loaded after the directory scan. |

Setting `mujoco_plugin_directories` **replaces** the default directory rather than adding to it - include the package's own path if you still want MuJoCo's plugins:

```python
{"mujoco_plugin_directories": [
    os.path.join(get_package_prefix("mujoco_ros2_control"), "lib", "mujoco_plugin"),
    "/opt/my_plugins",
]}
```

A configured directory that doesn't exist is warned about and skipped; a path in `mujoco_plugin_libraries` that doesn't exist is fatal, since `mj_loadPluginLibrary()` reports a failed `dlopen` through `mju_error`, which terminates the process.

## Side-Channel Sensors (cameras and lidars)

These are **not** ros2_control interfaces. Each instance runs as its own ROS node with its own publisher and worker thread, so they don't appear in `<ros2_control>` blocks and aren't broadcast through controllers.

### RGB-D Camera

A depth-camera node can be mounted in two ways. Both publish the same topics and read the same per-instance ROS parameters (keyed by the node name); they differ only in how camera pose and field of view are defined.

#### From a `<camera>` element (fixed FOV)

Every MuJoCo `<camera>` declared under a `<reference>` body spawns a depth-camera node named after the camera. The body defines its pose, vertical FOV comes from the MJCF `fovy`, horizontal FOV follows from the `width`/`height` aspect ratio. These cameras are always created and ignore the `enabled` flag.

```xml
<reference name="camera_link">
    <camera name="camera" mode="fixed" fovy="45" quat="0.5 0.5 -0.5 -0.5"/>
</reference>
```

```yaml
camera:
  ros__parameters:
    width: 640
    height: 480
    frequency: 30.0
    color_image: true
    depth_image: true
    point_cloud: true
```

#### From a `<site>` (config-driven, like the lidar)

A depth camera can also attach to MuJoCo sites, mirroring the GL lidar. Each site frame is interpreted as a **REP-103 optical frame** (+Z forward, +X right, +Y down), so place the site in your `*_optical_frame` - no orientation fix-up needed, the plugin converts it to the OpenGL camera convention internally. FOV and resolution come from ROS parameters, so intrinsics live in config instead of the model. The published `header.frame_id` is the site's **render frame** (its parent body).

A site camera is instantiated only if its `enabled` param is `true`, so camera sites can stay in the MJCF and be toggled at runtime. The node (and its params block and topics) is named after the site **with the prefix stripped**, so `mjCam_d435` becomes node `d435` publishing on `/d435/...`.

Three prefixes select how color and depth are sourced (grouped into one camera by the stripped `<name>`):

| Site | Meaning |
|---|---|
| `mjCam_<name>` | one site for **both** color and depth - a single shared render pass |
| `mjCamOpt_<name>` + `mjCamDepth_<name>` | color from the optical site, depth from the depth site - **two render passes** (true parallax). Each stream's `frame_id` is its own site's render frame; the cloud is colored from a render at the depth site so RGB and depth stay aligned. |

**Single site (shared render):**
```xml
<reference name="d435_depth_optical_frame">
    <site name="mjCam_d435" pos="0 0 0" euler="0 0 0"/>
</reference>
```

**Separate optical/depth sites (two frames):**
```xml
<reference name="d435_color_optical_frame">
    <site name="mjCamOpt_d435" pos="0 0 0" euler="0 0 0"/>
</reference>
<reference name="d435_depth_optical_frame">
    <site name="mjCamDepth_d435" pos="0 0 0" euler="0 0 0"/>
</reference>
```

```yaml
d435:                                  # site name(s) with the prefix stripped
  ros__parameters:
    enabled: true
    frequency: 30.0
    width: 640
    height: 480
    fovy: 64.9                       # vertical FOV [deg]
    fovx: 90.5                       # horizontal FOV [deg]; <= 0 → derive from fovy + aspect
    color_image: true
    depth_image: true
    point_cloud: true
```

Set only `fovy` (leave `fovx: 0.0`) for square pixels, where horizontal FOV is `2·atan(tan(fovy/2)·width/height)`. Set **both** to fix the two fields of view independently (e.g. to match a calibrated RealSense): MuJoCo renders square angular pixels, so the scene is rendered at the aspect implied by `(fovx, fovy)` and resampled to `width × height`, producing independent `fx`/`fy` in `camera_info` and the point cloud.

Only enabled streams are rendered: with separate sites, the color pass is skipped when `color_image` is off, the depth pass when both `depth_image` and `point_cloud` are off. The three prefixes and topic namespace can be changed on the `_mujoco_rgbd_camera_probe` node (`site_prefix`, `optical_site_prefix`, `depth_site_prefix`) and per camera (`topic_namespace`).

#### Topics

| Output | Topic | Type |
|---|---|---|
| Color | `/<name>/color/image_raw` (+ `camera_info`) | `sensor_msgs/Image` |
| Depth | `/<name>/depth/image_rect_raw` (+ `camera_info`) | `sensor_msgs/Image` |
| Cloud | `/<name>/depth/points` | `sensor_msgs/PointCloud2` |

Full parameter reference: [`mujoco_rgbd_camera_parameters.yaml`](src/mujoco_rgbd_camera_parameters.yaml).

### GL Depth-Buffer Lidar

A GPU-rendered lidar attaches to any MuJoCo site whose name starts with the configured prefix (default: `mjLidar_`). The published `frame_id` is the parent body name; the site's local pose defines the lidar frame. The node is named after the site with the prefix stripped, so `mjLidar_head` becomes node `head` publishing on `/head/...`. The global `site_prefix` can be changed on the `_mujoco_gl_lidar_probe` node.

A lidar is instantiated only if its site exists in the MJCF **and** its per-site ROS params set `enabled: true`, so lidar sites can stay in the MJCF and be toggled at runtime.

```xml
<reference name="head_link">
    <site name="mjLidar_head" pos="0 0 0.05" quat="1 0 0 0"/>
</reference>
```

```yaml
head:                     # site "mjLidar_head" with the "mjLidar_" prefix stripped
  ros__parameters:
    enabled: true
    output: cloud           # 'scan' or 'cloud'
    frequency: 10.0
    horizontal_min_angle: -1.5708
    horizontal_max_angle:  1.5708
    horizontal_samples: 1800
    vertical_min_angle: -0.2618
    vertical_max_angle:  0.2618
    vertical_samples: 16
    range_min: 0.05
    range_max: 30.0
    render_height: 256
```

| Output mode | Topic | Type |
|---|---|---|
| `scan` | `/<name>/scan` | `sensor_msgs/LaserScan` |
| `cloud` | `/<name>/points` | `sensor_msgs/PointCloud2` |

Full parameter reference: [`mujoco_gl_lidar_parameters.yaml`](src/mujoco_gl_lidar_parameters.yaml).
