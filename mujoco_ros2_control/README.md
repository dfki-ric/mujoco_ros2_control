# MuJoCo ROS2 Control

## Introduction

Welcome to the documentation of the **MuJoCo ROS2 Control**! This documentation provides an overview of the project, its purpose, and how to use it effectively.

## Features

- **Integration between ROS2 Control and MuJoCo:** MuJoCo ROS2 Control is a ROS 2 control plugin that provides integration between the Mujoco physics engine and the ROS 2 control framework.

## Installation

To use the **MuJoCo ROS2 Control**, follow these steps:
1. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
2. Install the Dependencies <br />
   ``` bash
   $ sudo apt-get install -y \
      libglfw3-dev \
      libx11-dev \
      xorg-dev \
      ros-humble-urdf \
      ros-humble-xacro \
      ros-humble-rviz2 \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-controller-manager

   ```
3. Install MuJoCo <br />
   Because it comes to problems with cmake that mujoco files aren't found, it is recommended to build and install mujoco from source.
   ``` bash
   git clone https://github.com/deepmind/mujoco
   mkdir mujoco/build
   cd mujoco/build
   cmake ..
   cmake --build ..
   cmake --install ..
   ```
4. Clone the Package in your ros2 workspace and compile it with colcon.

## Usage

Here are some guidelines on how to use the **MuJoCo ROS2 Control** effectively:

1. Prepare the mesh directory for MuJoCo
   - Mesh files for MuJoCo must be in a single folder
   - Only stl and obj files can be used in mujoco
   - It is recommendet to use a seperate mesh folder for mujoco
     - The original meshes can be used for ROS and rviz2
2. Create a launch file for the Simulation (like in panda_mujoco)
   - start the ```robot_state_publisher``` Node
   - use the [```xacro2mjcf.py```](xacro2mjcf_8py.html) node to create the required mjcf file within the launchfile
     - give absolute paths to the needed files or pass robot_descriptions as string as parameters
     - define the path for the output file and the generated files (the output file must be in the files path)
   - you must pass the following parameters to the node:
     - ```robot_description``` robot_description (as string)
     - ```robot_model_path``` the mujoco model file path
     - a ros2_control parameter yaml file path
   - you can provide the following parameters
     - ```simulation_frequency``` simulation frequency (default 1000Hz)
     - ```clock_publisher_frequency``` ros clock publish frequency (default 0Hz => no limitation)
     - ```real_time_factor``` realtime factor for mujoco (1.0 is realtime)
     - ```show_gui``` enable disable the gui (default true)

## URDF Configuration
### Mujoco specific elements
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
### ROS2 Control hardware example:
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
      
## License

The **MuJoCo ROS2 Control** is released under the [BSD 3-Clause](LICENSE). Please review the license file for more information.

## About

Provide a brief description of the project and the organization or individuals behind it. Include contact information or links to the project website, if applicable.