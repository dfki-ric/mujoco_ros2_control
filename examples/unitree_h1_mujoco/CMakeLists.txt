cmake_minimum_required(VERSION 3.8)
project(unitree_h1_mujoco)

# find dependencies
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch urdf config meshes
        DESTINATION share/${PROJECT_NAME}
)   

ament_package()