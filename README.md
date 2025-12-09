[![status](https://joss.theoj.org/papers/390ea08b16519c6913a9bd4e516ed08b/status.svg)](https://joss.theoj.org/papers/390ea08b16519c6913a9bd4e516ed08b)
# Mujoco ROS2 Control

The MuJoCo ROS 2 control hardware interface is designed to enable seamless integration between MuJoCo, a high-performance physics engine, and ROS 2, a widely used middleware for robotic systems. This interface provides a robust and efficient solution for leveraging MuJoCo’s powerful simulation capabilities within the ROS 2 ecosystem, enabling realistic physics-based robot simulation and control.

**software name:** Mujoco ROS2 Control

Mujoco ROS2 Control was initiated and is currently developed at the
[Robotics Innovation Center](http://robotik.dfki-bremen.de/en/startpage.html) of the
[German Research Center for Artificial Intelligence (DFKI)](http://www.dfki.de) in Bremen.

![https://www.dfki.de/web](./images/dfki_logo.png)

## Motivation
The development and testing of control algorithms for robotic systems is a crucial step in ensuring their reliability, safety, and efficiency. However, conducting these tests on physical hardware can be expensive, time-consuming, and prone to mechanical wear and tear. To overcome these challenges, accurate and efficient physical simulations have become an indispensable tool for researchers, engineers, and roboticists. These simulations enable comprehensive testing of robot controllers, planning algorithms, and perception systems in a controlled, repeatable, and risk-free environment.

## Compatibility
| ROS 2 Version           | Branch | Status |
|------------------------|--------|--------|
| Humble                 | [main](https://github.com/dfki-ric/mujoco_ros2_control)   | stable |
| Jazzy                  | [jazzy](https://github.com/dfki-ric/mujoco_ros2_control/tree/jazzy)  | unstable|

## Examples
We provide one example with the franka description and the gears from the IndustRealKit that can be started with 
```bash
  ros2 launch franka_mujoco franka.launch.py
```
![](./paper/figures/franka_rgbd_example.png)
*RGBD Camera inside of MuJoCo ([MuJoCo Visualizer](https://github.com/google-deepmind/mujoco/tree/main/simulate) \[left\], [rviz2](https://index.ros.org/p/rviz2/) \[right\])*

and one example with a unitree H1 that can be started with
```bash
  ros2 launch unitree_h1_mujoco unitree_h1.launch.py
```
![](./paper/figures/unitree_h1_example.png)
*Unitree H1 with floating joint between world and pelvis ([MuJoCo Visualizer](https://github.com/google-deepmind/mujoco/tree/main/simulate) \[left\], [rqt_joint_trajectory_controller](https://index.ros.org/p/rqt_joint_trajectory_controller/)/[rviz2](https://index.ros.org/p/rviz2/) \[right\])*

### Docker
To start you can use the ![dockerfile](./Dockerfile) to create a docker container with MuJoCo ROS2 control and its examples.

To try it out you can build and start the Docker image by execute the [run_docker_gui.sh](https://github.com/dfki-ric/mujoco_ros2_control/blob/main/run_docker_gui.sh) script.

The script will open a terminal where you can start the examples with the above mentioned launch commands (the terminal has already a sourced workspace and autocompletion).


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

## Installation
To use the **MuJoCo ROS2 Control**, follow these steps:
1. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
2. Install the Dependencies <br />
   ``` bash
   sudo apt-get update && sudo apt-get install -y \
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
        ros-humble-franka-description \
        libpcl-dev
   ```
3. Build the ros package.
   1. Create a ROS 2 workspace (can be skipped when a workspace exist)
   ```bash
    mkdir -p ~/mujoco_ws/src
   ```
   2. Clone and build the repo (and optionally use rosdep to pull deps)
   ```bash
    cd ~/mujoco_ws/src
    git clone https://github.com/dfki-ric/mujoco_ros2_control
    rosdep update && rosdep install --from-paths . --ignore-src --rosdistro humble -y # optional
    cd ~/mujoco_ws
    colcon build
   ```
4. Source the workspace
   ```bash
   cd ~/mujoco_ws
   source install/setup.bash
   ```
4. Test the ros package (Optional)
   ```bash
   launch_test src/mujoco_ros2_pkgs/mujoco_ros2_control/test/simple_launch.test.py 
   ```


## Getting Started
To use MuJoCo ROS2 control, you must create a launchfile (you can use the examples as reference):
### 1. Generate MJCF Using `xacro2mjcf.py` Node
- Add the `xacro2mjcf.py` node in your launch file to create the MJCF file at launch time.
  - ```python
    # Define the output location
    mujoco_model_path = "/tmp/mujoco"                               # Must be absolute path
    mujoco_model_file = os.path.join(mujoco_model_path, "main.xml") # Root file of the mjcf model
    
    # Process the xacro file and create the robot description
    robot_description = {
        'robot_description': xacro.process_file(
            franka_xacro_filepath,
            mappings={
                "name": "franka",
                "mujoco": "true"
            }
        ).toprettyxml(indent="  ")
    }
    
    additional_files = []
    # Mujoco Scene file
    additional_files.append(os.path.join(get_package_share_directory("mujoco_ros2_control"), "mjcf", "scene.xml"))
    additional_files.append(os.path.join(get_package_share_directory("task_table_mujoco"), "urdf", "task_table.urdf.xacro"))

    # Define the xacro2mjcf node
    xacro2mjcf = Node(
        package="mujoco_ros2_control",
        executable="xacro2mjcf.py",
        parameters=[
            {"robot_descriptions": [robot_description["robot_description"]]}, # Robot descriptions of actuated robots
            {"input_files": additional_files},        # Files that are added to mujoco but not to ros2_control
            {"output_file": mujoco_model_file},       # Mujoco output file
            {"mujoco_files_path": mujoco_model_path}, # Mujoco project folder
            # Floating base related params:
            {"floating": True},
            {"base_link": "pelvis"}, 
            {"initial_position": "0 0 1.05"},
            {"initial_orientation": "0 0 0"}
        ],
    )
    ```
- **Required setup:**
  - Use **absolute paths** for all file references.
  - Or pass `robot_description` as a **string parameter** (not a file path).
  - Set:
    - `output_file`: full path to the generated `.xml` MJCF file.
    - `mujoco_files_path`: directory where MJCF and related files will be stored.
      - **`output_file` must be inside `mujoco_files_path`.**


### 2. Required Parameters for `mujoco_ros2_control` Node
| Parameter              | Description                                       |
|------------------------|---------------------------------------------------|
| `robot_description`    | URDF string (not a file path)                     |
| `robot_model_path`     | Path to the generated MJCF `.xml` file            |
| ROS 2 control YAML     | Path to controller config (e.g., `*.yaml`)        |


### Launch Flow (in [launch.py](https://github.com/dfki-ric/mujoco_ros2_control/blob/main/examples/franka_mujoco/launch/franka.launch.py) file)
```python
def create_nodes(context: LaunchContext):
    namespace = ""
    mujoco_model_path = "/tmp/mujoco"
    mujoco_model_file = os.path.join(mujoco_model_path, "main.xml")

    # Set file paths to the wrapper package
    # (the package contains a robot description with aditional mujoco related configs)
    franka_xacro_filepath = [...]

    # Process the xacro file and create the robot description
    robot_description = [...]
    
    additional_files = [...]

    # Define the xacro2mjcf node
    xacro2mjcf = Node(
        package="mujoco_ros2_control",
        executable="xacro2mjcf.py",
        parameters=[...],
    )

    # Define the robot state publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        [...]
    )

    # Path to the ros2 control parameters file
    ros2_control_params_file = [...]

    # Define the mujoco node
    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            robot_description,
            ros2_control_params_file,
            {"simulation_frequency": 500.0},
            {"realtime_factor": 1.0},
            {"robot_model_path": mujoco_model_file},
            {"show_gui": True},
        ],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ]
    )

    # Wait with the start of mujoco_ros2_control until the xacro2mjcf is done
    start_mujoco = RegisterEventHandler(
        OnProcessExit(
            target_action=xacro2mjcf,
            on_exit=[
                LogInfo(msg="Created mujoco xml, starting mujoco node..."),
                mujoco
            ],
        )
    )

    # Define controller spawners
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            ["/", "controller_manager"],
        ],
    )

    # Register an event handler to start controllers once mujoco is up
    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="Starting joint state broadcaster..."),
                load_joint_state_broadcaster,
                [...],
            ],
        )
    )

    # Return the nodes and handlers
    return [
        robot_state_publisher,
        xacro2mjcf,
        start_mujoco,
        load_controllers
    ]

def generate_launch_description():

    return LaunchDescription([
        OpaqueFunction(function=create_nodes)  # Use OpaqueFunction for node creation
    ])
```

For the urdf creation you can take a look at ![URDF Configuration](./mujoco_ros2_control/README.md)

## Documentation
Run ```doxygen Doxyfile``` in the mujoco_ros2_control directory

## Bug Reports

To search for bugs or report them, please use GitHubs ![issue tracker](trhttps://github.com/dfki-ric/mujoco_ros2_control/issues)

[//]: <> (TODO put a link to the issue tracker here)

## Releases

### Semantic Versioning

Semantic versioning must be used, that is, the major version number will be
incremented when the API changes in a backwards incompatible way, the minor
version will be incremented when new functionality is added in a backwards
compatible manner, and the patch version is incremented for bugfixes,
documentation, etc.

## License
Mujoco ROS2 Control is distributed under the [3-clause BSD license](https://opensource.org/licenses/BSD-3-Clause).

## Maintainer / Authors / Contributers
Adrian Danzglock,       adrian.danzglock@dfki.de \
Vamsi Krishna Origanti, vamsi.origanti@dfki.de

Copyright 2025, DFKI GmbH / Robotics Innovation Center

## Funding
MuJoCo ROS2 Control was initiated and developed at Robotics Innovation Center, German Research Center for Artificial Intelligence (DFKI GmbH) at Bremen, Germany as part of the HARTU Project. This project has received funding from the European Union’s research and innovation program Horizon Europe under grant agreement No. 101092100.

![!\[\]()](https://eufunds.me/wp-content/uploads/2021/09/EU-flag-Horizon-Europe.jpg)
