---
title: 'MujocoROS2Control: Seamless MuJoCo Integration with ROS 2 for Robot Simulation and Control'
tags:
  - MuJoCo
  - Simulation
  - ROS 2
  - Robotics
authors:
  - name: Adrian Danzglock
    orcid: 0009-0004-4715-2973
    affiliation: 1
  - name: Vamsi Krishna Origanti
    orcid: 0009-0007-1696-5201
    affiliation: 1
affiliations:
 - name: Robotics Innovation Center, German Research Center for Artificial Intelligence (DFKI), Bremen, Germany
   index: 1
date: "2025-03-03"
bibliography: paper.bib
---

# Summary

The `MujocoROS2Control` hardware interface enables seamless integration between MuJoCo [@todorov2012mujoco], a high-performance physics engine, and ROS 2 [@ros2_control], a widely adopted middleware for robotic systems. This interface provides an efficient solution for simulating and controlling robots using MuJoCo’s physics capabilities within the ROS 2 ecosystem.

To support ROS-based workflows, we developed a dedicated URDF-to-MJCF conversion script. This tool translates URDF models into MJCF (MuJoCo XML format), preserving kinematic and dynamic properties and allowing custom MuJoCo-specific parameters such as sensors, actuators, and collision definitions to be specified directly in the URDF. This conversion ensures compatibility and adaptability for simulation.

`MujocoROS2Control` bridges the gap between ROS 2 and MuJoCo, offering a streamlined workflow for simulation, controller testing, and reinforcement learning.

# Statement of Need

Developing and validating control algorithms for robotic systems often requires extensive testing, which on physical hardware can be expensive, time-consuming, and subject to wear. Accurate simulation environments are essential for safe and scalable development.

MuJoCo (Multi-Joint dynamics with Contact) is a fast and accurate physics engine designed for robotics, control, biomechanics, and reinforcement learning. It provides:
- Precise multi-body dynamics with advanced numerical integration,
- High-speed simulation for real-time control and machine learning,
- Sophisticated contact modeling and soft constraint handling,
- Flexible actuator models and comprehensive sensor support.

Despite these capabilities, MuJoCo lacks native support for ROS 2, limiting its adoption in modern robotic development pipelines. `MujocoROS2Control` addresses this gap, enabling users to simulate ROS 2-compatible robots in MuJoCo with minimal overhead.

Our framework has been successfully used to test components such as force-torque sensor gravity compensation, torque-based Cartesian controllers, and Dynamic Movement Primitives (DMP)-based skill reproduction [@Fabisch2024].

# Implementation

`MujocoROS2Control` integrates MuJoCo with the `ros2_control` framework. A key component is the URDF-to-MJCF converter, which maintains fixed joints (which MuJoCo typically collapses), allows sensor and actuator tags within URDFs, and generates MJCF files compatible with MuJoCo’s expectations.

The interface supports:
- Direct torque control,
- PID-based position/velocity/acceleration control,
- MuJoCo’s native actuator models.

Joint states and simulation time are published for synchronization with the ROS 2 system time (`/clock`). Sensors defined in the URDF (force-torque, IMU, pose, RGB-D camera) are exposed as individual ROS nodes using `realtime_tools` [@realtime_tools] to maintain real-time performance.

# Examples

## Franka FR3 with IndustRealKit Gears

This example integrates the Franka FR3 robot [@franka_description] with high-resolution gear models from the IndustRealKit [@tang2023industreal]. MuJoCo actuators are used to generate joint torques, although the implemented PID control and torque control are also supported.
For the high-resolution collision modeling, we use CoaCD [@wei2022coacd] to create multiple convex hulls from complex mesh geometry. The URDF-to-MJCF converter automatically replaces original mesh files in the mjcf, when the converted files are in the same directory.

![Franka FR3 controlled with ROS 2 Joint Trajectory Controller](./figures/franka_rgbd_example.png)


## IMRK System

In this example, we simulate the iMRK system developed at DFKI Bremen, consisting of two KUKA LBR iiwa 14 robots [@Mrongaimrk]. A ROS2 Cartesian impedance controller (migrated version of [@mayr2024cartesian]) is used for each arm. MuJoCo actuators manage the Robotiq 2F grippers, and force-torque sensors are simulated at both end-effectors.

![IMRK with Robotiq 2F Gripper and Robotiq FT300 Sensor](./figures/kuka_imrk_example.png)

(Note: The IMRK robot description is internal and not included in the public examples.)

# Conclusion

`MujocoROS2Control` enables high-fidelity robotic simulation by integrating MuJoCo with ROS 2. Through its robust conversion utilities, sensor bridging, and actuator support, it provides a reliable framework for control development, validation, and machine learning research.

Its lightweight design and fast simulation performance make it well-suited for force-based control, trajectory optimization, and reinforcement learning applications. Future enhancements may include support for additional sensor types, multi-robot coordination, and real-time closed-loop learning.

# Acknowledgements

This library was initiated and developed at the Robotics Innovation Center, German Research Center for Artificial Intelligence (DFKI GmbH), Bremen, Germany, as part of the HARTU Project. This project received funding from the European Union’s Horizon Europe research and innovation program under grant agreement No. 101092100.

# References