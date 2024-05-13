# Install MuJoCo
Because it comes to problems with cmake that mujoco files arent found, it is recommended to build and install mujoco from source.
```
git clone https://github.com/deepmind/mujoco
mkdir mujoco/build
cd mujoco/build
cmake ..
cmake --build .
cmake --install .
```

# Creating a scene with Phobos and high resolution collision meshes
## Create a scene with ![Phobos](https://github.com/dfki-ric/phobos)
1. Import the meshes you want to use
    - File -> import
2. Set the origin and the scale of the meshes and export them as stl (File -> export)
    - It is recommended to save keep the meshes in a collection, to copy the mesh from there to use it.
3. Create the scene by copy and paste the original mesh to the position where you want it
4. Select all meshes that should be in the urdf model and click in the phobos menu on ```set Phobostype```and set it to visual
5. Select all visual elements and click on ```Create Link(s)```
    - Define the tree structure of the links by selecting the correct parent for the links in the Object Properties of each link
6. Select all Links and click in the phobos menu on ```Create inertials```
7. Select the Visuals, the links and the inertials and click on ```Export Model```in the phobos menu after setting a correct path
    - Select urdf under Models
8. When the visual and collision elements in the links are missing, add them to the links and maybe cleanup the urdf, that you use a xacro macro for repeating objects.
    - Use the correct path to the exported stl meshes

## Create high resolution collision objects with ![CoACD](https://github.com/SarahWeiii/CoACD)
1. Use the ```run_coacd.py``` script to create the decomposed meshes by running ```python3 run_coacd.py -i <input_file>.stl -o stl```
    - When the collision object has holes or something that is missing after the normal run it without preprocessing by adding ```-pm off```
