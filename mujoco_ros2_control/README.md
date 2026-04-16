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
