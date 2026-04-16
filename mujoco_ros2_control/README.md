# URDF Configuration (for usage with xacro2mjcf script)

To use this package with an existing robot description (URDF or Xacro), create a **Xacro wrapper file** that merges:

- your existing robot description,
- the MuJoCo configuration, and
- the ROS 2 Control configuration.

For reference, see the `urdf` directories in the provided examples ([franka](https://github.com/dfki-ric/mujoco_ros2_control/blob/main/examples/franka_mujoco/urdf/franka.urdf.xacro), [unitree](https://github.com/dfki-ric/mujoco_ros2_control/blob/main/examples/unitree_h1_mujoco/urdf/unitree_h1.urdf.xacro)).

---

## MuJoCo-Specific Elements

The following snippet shows how to integrate MuJoCo configuration elements into your robot description:

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
        <!-- Add per-body and per-joint configuration -->
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
        <!-- Position sensor:
             https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-framepos -->
        <framepos
            name="camera_link_pose"
            objtype="body"
            objname="camera_link"
            reftype="body"
            refname="world"/>

        <!-- Orientation sensor:
             https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-framequat -->
        <framequat
            name="camera_link_quat"
            objtype="body"
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
Below is an example of how to declare a ROS 2 Control system using PID and torque control:
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

Sensors are declared inside the `<ros2_control>` block and are automatically matched to MuJoCo sensors via a `<param>` that specifies the MuJoCo object (site, body, geom, etc.) the sensor is attached to. Three sensor types are supported: **IMU**, **Force/Torque**, and **Pose**.

The sensor type is determined automatically from the state interface names you declare.

### IMU Sensor

Reads orientation (framequat), angular velocity (gyro), and linear acceleration (accelerometer) from a MuJoCo site. Use with the standard [imu_sensor_broadcaster](https://control.ros.org/rolling/doc/ros2_controllers/imu_sensor_broadcaster/doc/userdoc.html).

```xml
<ros2_control name="MySystem" type="system">
    <hardware>
        <plugin>mujoco_ros2_control/MujocoSystem</plugin>
    </hardware>

    <!-- ... joints ... -->

    <sensor name="imu_in_pelvis">
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

The corresponding MuJoCo sensors must be defined in the `<mujoco>` section:
```xml
<mujoco>
    <!-- Create the site on the body where the IMU is located -->
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

Reads force and torque from a MuJoCo site. Use with the standard [force_torque_sensor_broadcaster](https://control.ros.org/rolling/doc/ros2_controllers/force_torque_sensor_broadcaster/doc/userdoc.html).

```xml
<sensor name="ft_sensor">
    <param name="site">ft_site</param>
    <state_interface name="force.x"/>
    <state_interface name="force.y"/>
    <state_interface name="force.z"/>
    <state_interface name="torque.x"/>
    <state_interface name="torque.y"/>
    <state_interface name="torque.z"/>
</sensor>
```

The corresponding MuJoCo sensors:
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

Controller configuration:
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

Reads position (framepos) and orientation (framequat) of a MuJoCo body. Use with the standard [pose_broadcaster](https://control.ros.org/rolling/doc/ros2_controllers/pose_broadcaster/doc/userdoc.html). This is useful for floating-base robots to get the base link pose.

```xml
<sensor name="pelvis_pose">
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

The corresponding MuJoCo sensors:
```xml
<mujoco>
    <sensor>
        <framepos name="pelvis_pose" objtype="body" objname="pelvis" reftype="body" refname="world"/>
        <framequat name="pelvis-orientation" objtype="body" objname="pelvis" reftype="body" refname="world"/>
    </sensor>
</mujoco>
```

Controller configuration:
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

The `<param>` inside a `<sensor>` block tells the plugin which MuJoCo object to look for. Supported keys are: `site`, `body`, `geom`, `camera`, `light`, `frame`. If no param is provided, the sensor `name` is used as the match key.

For example, `<param name="site">imu_in_pelvis</param>` will match all MuJoCo sensors whose object name is `imu_in_pelvis`.
