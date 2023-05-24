cmake_minimum_required(VERSION 3.5)
project(mujoco_ros2_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(OpenGL_GL_PREFERENCE LEGACY)

## Find catkin macros and libraries
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# msgs
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(urdf REQUIRED)
find_package(mujoco REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(angles REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(glfw3 REQUIRED)

## Specify additional locations of header files
add_definitions(
        ${rclcpp_DEFINITIONS}
        ${mujoco_DEFINITIONS}
)
link_directories(
        ${rclcpp_LIBRARY_DIRS}
        ${ament_cmake_DIRS}
        /usr/local/lib
        #/opt/mujoco/lib
)
include_directories(
        include
        ${rclcpp_INCLUDE_DIRS}
        #/opt/mujoco/include
)
ament_lint_auto_find_test_dependencies()

add_library(mujoco_system_plugins SHARED
        src/mujoco_system.cpp
        )
ament_target_dependencies(mujoco_system_plugins
        angles
        hardware_interface
        rclcpp
        urdf
        )

add_library(mujoco_visualization_utils src/mujoco_visualization_utils.cpp)
ament_target_dependencies(mujoco_visualization_utils
        rclcpp
        glfw3
)

add_executable(${PROJECT_NAME} src/mujoco_ros2_control_plugin.cpp
        )
ament_target_dependencies(${PROJECT_NAME}
        angles
        controller_manager
        hardware_interface
        rclcpp
        yaml_cpp_vendor
        glfw3
        urdf
        mujoco
        )
target_link_libraries(
        mujoco_ros2_control
        mujoco
        glfw
        mujoco_visualization_utils
)

## Install
install(TARGETS
        ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
        )

install(TARGETS
        mujoco_system_plugins
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(PROGRAMS
        scripts/xacro2mjcf.sh
        DESTINATION share/${PROJECT_NAME}/)

ament_export_include_directories(
        include
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_libraries(
        ${PROJECT_NAME}
        mujoco_system_plugins
)


pluginlib_export_plugin_description_file(mujoco_ros2_control mujoco_system_plugins.xml)

ament_package()
