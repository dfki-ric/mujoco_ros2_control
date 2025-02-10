---
title: 'MujocoRos2Control: A hardware interface for mujoco'
tags:
  - Mujoco
  - Simulation
authors:
  - name: Adrian Danzglock
    orcid: 0009-0004-4715-2973
    equal-contrib: true
    affiliation: 1
  - name: Vamsi Krishna Origanti
    orcid: 0009-0007-1696-5201
    equal-contrib: true
    affiliation: 1
affiliations:
 - name: DFKI, Germany
   index: 1
date: 13 August 2017
bibliography: paper.bib
---
# Summary
The Mujoco ROS 2 control hardware interface enables the use of Mujoco with ROS 2. 
The implementation of the system interface is partially based on the gz_ros2_control implementation. However, unlike gz_ros2_control, the interface is designed as a standalone class. To facilitate the use of URDF files in Mujoco, a URDF-to-MJCF conversion Python script has been developed. This script can generate an MJCF file with the desired attributes and sensors from one or multiple URDF files.

# Statement of need
The development and testing of control algorithms for robotic systems often requires accurate and efficient physical simulations. Simulators are an important tool for researchers and practitioners. They enable the training of reinforcement learning (RL) strategies, the testing of controllers and the refinement of robot designs without the need for physical hardware. However, many of the existing physics engines integrated into ROS2 (Robot Operating System 2) lack the precision, computational power and flexibility required to simulate complex kinematic robots with realistic dynamics.

MuJoCo is a state-of-the-art physics engine known for its accurate handling of rigid body dynamics and computational efficiency, making it an excellent choice for simulating robotic systems. However, direct integration with ROS2, a middleware widely used in robotics, is limited. This gap prevents researchers from utilizing MuJoCo's capabilities in ROS2-based workflows.

Our software, MuJoCoROS2Control, bridges this gap by providing an intuitive and functional interface for using MuJoCo as a physics engine in ROS2 environments. The interface supports key robot sensors such as Depth cameras and ft-sensors and enables realistic perception-based simulations. The main focus is on enabling robust simulations for kinematic robots and providing a platform where users can train reinforcement learning algorithms and test controllers with high accuracy.

# Implementation
## General Structure

Mujoco_ros2_control is essentially composed of four main components:


1. Preparation of the robot description
  - Because ros uses the URDF file format to describe the robot and mujoco uses the MJCF file format, it was required to convert a urdf to mjcf with passing important options and not by urdf supported features to mjcf by using our own urdf 2 mjcf scripts.
  With this scripts it is possible to have fixed joints (that arent merged to a single object) in mjcf and it is possible to declare sensors and other mujoco specific stuff in the urdf, to initialize it in the mujoco simulation
  
1. ROS 2 Control
    - This component provides the ROS 2 hardware interface and allows the use of Mujoco actuators for joint control (in addition to standard torque control). Furthermore, this component publishes the system time and visualizes the other two components (sensors and GUI).
    - It uses the SystemInterface from ros2 control to update the state and command interfaces required for ros2. The actual state is read from mujocos data object that holds the actual state in form of q (qpos for position), qd (qvel for velocity), and \tau (qfrc_applied for effort).
    - Because Mujoco uses applied torques to control the joints internal, we integrated a optional simple pid controller for combinations of position, velocity and acceleration command interfaces as alternative to the usage of mujocos actuators (that can be defined in the mjcf) .

2. Sensors
    - Sensors are launched as independent nodes in ROS 2. Apart from the camera sensor, they utilize a sensor class that individual sensor classes can inherit from, enabling easy integration of additional sensors.

    - Sensors must be defined in the URDF/MJCF file. During runtime, they can be read and published using the Mujoco data object.

3. Visualization (GUI)

    - The visualization utilizes the GUI from the Simulate.cc example of Mujoco, which is incorporated as an external library.

## Helpers Scripts
Because mujoco uses mjcf and not urdf as input model and the default structure of a mjcf don't include fixed joints (they are merged), we created our own converter to create a mjcf from one or multiple urdf files. This conversion makes sure that we can use a urdf from ros that is also used by the controller manager, is used in mujoco. Because the urdf format don't provide all optional elements of mjcf, it is possible to add additional options or elements to an existing element, by refering it in the mujoco element of the urdf.

# Citations

# Figures
<!-- TODO -->
# Acknowledgements
<!-- TODO -->
# References
<!-- TODO -->