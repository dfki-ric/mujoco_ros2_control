# URDF Configuration
## Mujoco specific elements
```mujoco element
<mujoco>
    <!-- define compiler options (https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler) -->
    <compiler meshdir="/tmp/mujoco/meshes" discardvisual="true" autolimits="false" balanceinertia="true"/>
    
    <!-- define mujoco options (https://mujoco.readthedocs.io/en/stable/XMLreference.html#option) -->
    <option integrator="implicitfast" gravity="0 0 -9.81" impratio="10" cone="elliptic" solver="Newton">
        <flag multiccd="enable" />
    </option>
    
    <!-- add elements/tags to a mjcf body or child of a body
    <reference name="${prefix}left_inner_finger">
        <body gravcomp="1"/>  <!-- add gravcomp do the referenced body -->
        <joint damping="10"/> <!-- add damping to every child joint of the referenced body -->
        <!-- add tags to the geom with the given name, that is a child of the reference body -->
        <geom name="geom1" friction="0.7" mass="0" priority="1" solimp="0.95 0.99 0.001" solref="0.004 1"/>
    </reference>
    
    <!-- define a rgbd camera in mujoco (https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-camera) -->
    <reference name="camera_link">
        <camera name="camera" mode="fixed" fovy="45" quat="0.5 0.5 -0.5 -0.5"/>
    </reference>
    
    <!-- define a a pose sensor for the camera link with respect to the world frame -->
    <sensor>
        <!-- https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-framepos -->
        <framepos name="camera_link_pose" objtype="body" objname="camera_link" reftype="body" refname="world"/>
        <!-- https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-framequat -->
        <framequat name="camera_link_quat" objtype="body" objname="camera_link" reftype="body" refname="world"/>
    </sensor>
</mujoco>
```
## ROS2 Control hardware example:
```ros2_control
<ros2_control name="${prefix}${name}" type="system">
    <hardware>
        <plugin>mujoco_ros2_control/MujocoSystem</plugin>
    <hardware>
    
    <!-- example joint with position + velocity + acceleration control -->
    <joint name="joint1">
        <command_interface name="position"/>
        <command_interface name="velocity"/>
        <command_interface name="acceleration"/>
        <param name="kp">1000.0</param>
        <param name="ki">0.0</param>
        <param name="kd">0.01</param>
        <!-- only needed if position and velocity control -->
        <param name="kvff">0.01</param>
        <!-- only needed if position, velocity and acceleration control -->
        <param name="kaff">0.01</param>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
    </joint>
    
    <!-- example joint with position + velocity + acceleration control -->
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