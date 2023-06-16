cmake_minimum_required(VERSION 3.8)
project(mujoco_ros2_pkgs)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})        

ament_package()
