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
<!-- TODO -->
# Implementation
## General Structure

Mujoco_ros2_control is essentially composed of three main components:

1. ROS 2 Control
    - This component provides the ROS 2 hardware interface and allows the use of Mujoco actuators for joint control (in addition to standard torque control). Furthermore, this component publishes the system time and visualizes the other two components (sensors and GUI).
    - Because Mujoco uses applied torques to control the joints internal, we integrated a opttional simple pid controller for combinations of position, velocity and acceleration command interfaces as alternative to the usage of mujocos actuators (that can be defined in the mjcf).

2. Sensors
    - Sensors are launched as independent nodes in ROS 2. Apart from the camera sensor, they utilize a sensor class that individual sensor classes can inherit from, enabling easy integration of additional sensors.

    - Sensors must be defined in the URDF/MJCF file. During runtime, they can be read and published using the Mujoco data object.

3. Visualization (GUI)

    - The visualization utilizes the GUI from the Simulate.cc example of Mujoco, which is incorporated as an external library.

## Helpers
Because mujoco uses mjcf and not urdf as input model and the default structure of a mjcf don't include fixed joints (they are merged), we created our own converter to create a mjcf from one or multiple urdf files. This conversion makes sure that we can use a urdf from ros that is also used by the controller manager, is used in mujoco. Because the urdf format don't provide all optional elements of mjcf, it is possible to add additional options or elements to an existing element, by refering it in the mujoco element of the urdf.

# Citations
<!-- TODO -->
# Figures
<!-- TODO -->
# Acknowledgements
<!-- TODO -->
# References
<!-- TODO -->