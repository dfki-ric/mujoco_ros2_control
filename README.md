# Install MuJoCo
Because it comes to problems with cmake that mujoco files arent found, it is recommended to build and install mujoco from source.
```
git clone https://github.com/deepmind/mujoco
mkdir mujoco/build
cd mujoco/build
cmake ..
cmake --build ..
cmake --install ..```