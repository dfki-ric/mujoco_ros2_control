# Creating a scene with Phobos and high resolution collision meshes
## Create a scene with ![Phobos](https://github.com/dfki-ric/phobos)
0. Install ![Blender](https://docs.blender.org/manual/en/latest/getting_started/installing/index.html#installing-blender) and ![Phobos](https://github.com/dfki-ric/phobos/wiki/Installation#phobos)
1. Importing Meshes in ![Blender](https://docs.blender.org/manual/en/latest/index.html)
    - ![Topbar ‣ File ‣ Import](https://docs.blender.org/manual/en/latest/files/import_export.html)
2. Mesh Preparation
    - Set the ![origin](https://docs.blender.org/manual/en/latest/scene_layout/object/origin.html#set-origin) and the ![transform](https://docs.blender.org/manual/en/latest/scene_layout/object/properties/transforms.html#transform) of the meshes and export them as ![stl](https://docs.blender.org/manual/en/latest/files/import_export/stl.html#exporting) (Topbar ‣ File ‣ export)
    -  It's advisable to organize these meshes in a collection for ease of access.
3. Scene Setup
    - Create the scene by copy and paste the original mesh to the position where you want it
4. Setting Phobos Types
    - Select all meshes in the scene that should be in the urdf model and click in the ![phobos menu](https://github.com/dfki-ric/phobos/wiki/Phobos-GUI) on ![set Phobostype](https://github.com/dfki-ric/phobos/wiki/Operators#set-phobostype) and set it to visual
5. Create Links [![wiki](https://github.com/dfki-ric/phobos/wiki/Kinematic-Skeleton)]
    - Select all visual elements and click on ![Create Link(s)](https://github.com/dfki-ric/phobos/wiki/Operators#create-link)
    - When the parenting wasn't done by creating the links, establish the hierarchical structure of the links by ![assigning parent links](https://docs.blender.org/manual/en/latest/animation/armatures/bones/editing/parenting.html#parenting) in the Object Properties of each link.
6. Mass and Inertials 
    - Select all links and generate inertial properties using the Phobos menu. [![wiki](https://github.com/dfki-ric/phobos/wiki/Mass-and-Inertia)]
7. Exporting the model [![wiki](https://github.com/dfki-ric/phobos/wiki/Export)]
    - Choose the correct path and export the visuals, links, and inertials as a URDF model via the Phobos menu.
8. Handling Missing Elements
    - if visual or collision elements are absent in the ![links](http://wiki.ros.org/urdf/XML/link), ensure they are added.
    - if the ![joint](http://wiki.ros.org/urdf/XML/joint) to world or another parent is missing add it. 
    - Consider streamlining URDF generation by employing ![Xacro](http://wiki.ros.org/xacro) macros for repetitive objects. Verify that the paths to exported STL meshes are correctly specified.

## Create high resolution collision objects with ![CoACD](https://github.com/SarahWeiii/CoACD)
0. Install ![CoACD](https://github.com/SarahWeiii/CoACD?tab=readme-ov-file#1-installation)
1. Mesh Decomposition
    - Utilize the ![run_coacd.py](mujoco_ros2_control/scripts/run_coacd.py) script to decompose meshes into collision-friendly components.
    - Execute the script using Python 3: ```python3 run_coacd.py -i <input_file>.stl -o stl```
    - If the collision object exhibits holes or missing elements, consider running the decomposition script without preprocessing:  ```python3 run_coacd.py -i <input_file>.stl -o stl -pm off```
