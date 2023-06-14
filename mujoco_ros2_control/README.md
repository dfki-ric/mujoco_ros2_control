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
      
## License

The **MuJoCo ROS2 Control** is released under the [Apache-2](license.md). Please review the license file for more information.

## About

Provide a brief description of the project and the organization or individuals behind it. Include contact information or links to the project website, if applicable.