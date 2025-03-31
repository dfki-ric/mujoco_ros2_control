# Mujoco Ros2 Control

[//]: <> (TODO description of the software)

The MuJoCo ROS 2 control hardware interface is designed to enable seamless integration between MuJoCo, a high-performance physics engine, and ROS 2, a widely used middleware for robotic systems. This interface provides a robust and efficient solution for leveraging MuJoCo’s powerful simulation capabilities within the ROS 2 ecosystem, enabling realistic physics-based robot simulation and control.

[//]: <> (TODO contribution of DFKI / other partners)

**software name:** Mujoco Ros2 Control

Mujoco Ros2 Control was initiated and is currently developed at the
[Robotics Innovation Center](http://robotik.dfki-bremen.de/en/startpage.html) of the
[German Research Center for Artificial Intelligence (DFKI)](http://www.dfki.de) in Bremen.

![https://www.dfki.de/](image.png)

## Motivation
The development and testing of control algorithms for robotic systems is a crucial step in ensuring their reliability, safety, and efficiency. However, conducting these tests on physical hardware can be expensive, time-consuming, and prone to mechanical wear and tear. To overcome these challenges, accurate and efficient physical simulations have become an indispensable tool for researchers, engineers, and roboticists. These simulations enable comprehensive testing of robot controllers, planning algorithms, and perception systems in a controlled, repeatable, and risk-free environment.

## Getting Started
To use MuJoCo Ros2 control, you must create a launchfile (you can use the examples as reference):
- start the ```robot_state_publisher``` Node
- use the ```xacro2mjcf.py``` node to create the required mjcf file within the launchfile
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

For the urdf creation you can take a look at ![URDF Configuration](./mujoco_ros2_control/README.md)

### Examples
We provide one example with the franka description and the gears from the IndustRealKit that can be started with ```ros2 launch franka_mujoco franka.launch.py```
![RGBD Camera inside of MuJoCo](./paper/figures/franka_rgbd_example.png)

and one example with a unitree H1 that can be started with ```ros2 launch unitree_mujoco unitree.launch.py```
![Unitree H1 with floating joint between world and pelvis](./paper/figures/unitree_h1_example.png)

### Docker
To start you can use the ![dockerfile](./Dockerfile) to create a docker container with MuJoCo Ros2 control and its examples.
```docker build docker build -t "mujoco_ros2_control" .```

To try out the examples you can follow this steps to run a container with mujoco_ros2_control and its examples:
```bash
# Build the container
docker build docker build -t "mujoco_ros2_control" .
# create the network for the container
docker network create ros
#give permissions to use X11 with docker 
xhost +local:docker
# starts the container with the franka example
docker run \
    --network="ros" \
    --device="/dev/dri:/dev/dri" \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.Xdocker \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -it mujoco_ros2_control bash
xhost -local:docker
```


## Requirements / Dependencies
```
libglfw3-dev
libx11-dev
xorg-dev
ros-humble-urdf
ros-humble-xacro
ros-humble-rviz2
ros-humble-ros2-control
ros-humble-ros2-controllers
ros-humble-controller-manager
ros-humble-pcl-ros
ros-humble-perception-pcl
libopencv-dev
ros-humble-pcl-conversions
ros-humble-cv-bridge
libpcl-dev
ros-humble-urdfdom-py
```

[//]: <> (TODO which dependencies do I need?)

## Installation
To use the **MuJoCo ROS2 Control**, follow these steps:
1. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
2. Install the Dependencies <br />
   ``` bash
   $ apt-get update && apt-get install -y \
        git \
        libglfw3-dev \
        libx11-dev \
        xorg-dev \
        ros-humble-urdf \
        ros-humble-xacro \
        ros-humble-rviz2 \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-controller-manager \
        ros-humble-pcl-ros \
        ros-humble-perception-pcl \
        ros-humble-urdfdom-py \
        libopencv-dev \
        ros-humble-pcl-conversions \
        ros-humble-cv-bridge \
        libpcl-dev
   ```
3. Install MuJoCo <br />
   Because it comes to problems with cmake that mujoco files aren't found, it is recommended to build and install mujoco from source.
   ``` bash
   git clone https://github.com/deepmind/mujoco -b 3.2.7
   mkdir mujoco/build
   cd mujoco/build
   cmake ..
   cmake --build ..
   cmake --install ..
   ```
4. Build the ros package.
   ```bash
    git clone <package url>
    colcon build
   ```

## Documentation
Run ```doxygen Doxyfile``` in the mujoco_ros2_control directory
[//]: <> (TODO complete documentation, a link to it, or instructions that tell the user how to build it)

## Testing

[//]: <> (TODO document how to run the tests)

## Coverage (required by Stage 2)

[//]: <> (TODO document how the code coverage can be accessed)

## Deployment

[//]: <> (TODO document how to deploy the software)

## Current State

[//]: <> (TODO is it actively developed?)

## Bug Reports

To search for bugs or report them, please use GitHubs issue tracker at:

[//]: <> (TODO put a link to the issue tracker here)

## Referencing

[//]: <> (TODO preferred way of referencing this software, e.g., use publication ...)

## Releases

[//]: <> (TODO release guidelines)

[//]: <> (TODO describe the versioning approach, for example:)

### Semantic Versioning

Semantic versioning must be used, that is, the major version number will be
incremented when the API changes in a backwards incompatible way, the minor
version will be incremented when new functionality is added in a backwards
compatible manner, and the patch version is incremented for bugfixes,
documentation, etc.

## License

<!--[//]: <> (Note: This section is redundant with the LICENSE file. You can omit it if you want.)

[//]: <> (TODO license)

[//]: <> (The New BSD license is recommended by the software board
          ([source](http://wiki.dfki.uni-bremen.de/index.php/Open_Source_License_Recommendation source)).
          This is the corresponding text for the README.md:)-->

Mujoco Ros2 Control is distributed under the [3-clause BSD license](https://opensource.org/licenses/BSD-3-Clause).

## Maintainer / Authors / Contributers

[//]: <> (TODO document who contributes to the software)

[//]: <> (Your employee has the copyright of your work. If you collaborate with other partners,
          the copyright is shared between involved institutes. You can write, for example,)
Adrian Danzglock,       adrian.danzglock@dfki.de \
Vamsi Krishna Origanti, vamsi.origanti@dfki.de

Copyright 2025, DFKI GmbH / Robotics Innovation Center

[//]: <> (if the software is a result of a cooperation of the DFKI  RIC and the Robotics Research Group.)