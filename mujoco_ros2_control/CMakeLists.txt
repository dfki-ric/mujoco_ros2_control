cmake_minimum_required(VERSION 3.16)
project(mujoco_ros2_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-g)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")

set(OpenGL_GL_PREFERENCE LEGACY)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        ament_cmake
        rclcpp
        std_msgs
        geometry_msgs
        rosgraph_msgs
        controller_manager
        hardware_interface
        urdf
        ament_lint_auto
        glfw3
        realtime_tools
        OpenCV
        PCL
        pcl_conversions
        cv_bridge
        generate_parameter_library
        Eigen3
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(mujoco 3.0.0 REQUIRED)
add_subdirectory(${CMAKE_SOURCE_DIR}/libsimulate)
## Specify additional locations of header files
add_definitions(
        ${rclcpp_DEFINITIONS}
        ${mujoco_DEFINITIONS}
)
link_directories(
        ${rclcpp_LIBRARY_DIRS}
        ${ament_cmake_DIRS}
        ${PCL_LIBRARY_DIRS}
)
include_directories(
        include
        ${rclcpp_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${pcl_conversions_INCLUDE_DIRS}
)

generate_parameter_library(${PROJECT_NAME}_parameters
        src/${PROJECT_NAME}_parameters.yaml
        )

generate_parameter_library(mujoco_rgbd_camera_parameters
        src/mujoco_rgbd_camera_parameters.yaml
        )

ament_lint_auto_find_test_dependencies()

add_library(mujoco_system_plugins SHARED
        src/mujoco_system.cpp)
ament_target_dependencies(mujoco_system_plugins
        hardware_interface
        rclcpp
        urdf
        )


add_library(mujoco_visualization src/mujoco_visualization.cpp)
ament_target_dependencies(mujoco_visualization
        rclcpp
        glfw3
        )
#target_link_libraries(mujoco_visualization
#        mujoco::libsimulate
#        )


add_executable(${PROJECT_NAME}
        src/mujoco_ros2_control_plugin.cpp
        src/mujoco_depth_camera.cpp
        src/pose_sensor.cpp src/mujoco_ros2_sensors.cpp
        src/simulate_gui.cpp)
ament_target_dependencies(${PROJECT_NAME}
        controller_manager
        hardware_interface
        rclcpp
        urdf
        mujoco
        glfw3
        realtime_tools
        OpenCV
        cv_bridge
        PCL
        pcl_conversions
)
target_link_libraries(
        mujoco_ros2_control
        mujoco_visualization
        mujoco
        glfw
        ${PCL_LIBRARIES}
        ${PROJECT_NAME}_parameters
        mujoco_rgbd_camera_parameters
        mujoco::libsimulate
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

install(TARGETS
        mujoco_visualization
        DESTINATION lib
        )
        
install(DIRECTORY mjcf DESTINATION share/${PROJECT_NAME}/)

ament_python_install_package(scripts)

install(PROGRAMS
        scripts/xacro2mjcf.py
        DESTINATION lib/${PROJECT_NAME})

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
