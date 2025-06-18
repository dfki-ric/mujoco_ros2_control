cmake_minimum_required(VERSION 3.16)
project(mujoco_ros2_control)

# Set default C standards
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Set default C++ standards
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Enable debug build
set(CMAKE_BUILD_TYPE Debug)

# Export compile commands for IDEs
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# Set OpenGL preferences
set(OpenGL_GL_PREFERENCE LEGACY)

# Package dependencies
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
    realtime_tools
    OpenCV
    PCL
    pcl_conversions
    cv_bridge
    generate_parameter_library
    Eigen3
    tf2_ros
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

# Set Eigen3
set(EIGEN3_FOUND TRUE)
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# Fetch MuJoCo
include(FetchContent)
FetchContent_Declare(
  mujoco
  GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
  GIT_TAG 3.2.7  # MuJoCo version
)

# Optionally set MuJoCo build options
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
set(BUILD_SIMULATE ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(mujoco)

# Install the ccd-config.cmake to the appropriate location
install(FILES ${CMAKE_BINARY_DIR}/_deps/ccd-build/ccd-config.cmake
        DESTINATION share/${PROJECT_NAME}/cmake)

# Set LibSimulate usage flag
set(USE_LIBSIMULATE ON CACHE BOOL "Use LibSimulate" FORCE)
message("Setting USE_LIBSIMULATE to: ${USE_LIBSIMULATE}")

# Configure the header file with CMake variables
configure_file(cmake/config.h.in ${CMAKE_BINARY_DIR}/config.h)

# Add compile definitions and directories
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
    ${CMAKE_BINARY_DIR}/_deps/mujoco-src/include
    ${CMAKE_BINARY_DIR}/_deps/mujoco-src/simulate
    ${CMAKE_BINARY_DIR}
)

# Generate parameter libraries
generate_parameter_library(${PROJECT_NAME}_parameters
    src/${PROJECT_NAME}_parameters.yaml
)

generate_parameter_library(mujoco_rgbd_camera_parameters
    src/mujoco_rgbd_camera_parameters.yaml
)

# Add system plugin
add_library(mujoco_system_plugins SHARED src/mujoco_system.cpp)
ament_target_dependencies(mujoco_system_plugins
    hardware_interface
    rclcpp
    urdf
)

# Add executable for the main ROS2 control node
add_executable(${PROJECT_NAME}
    src/mujoco_ros2_control_plugin.cpp
    src/mujoco_depth_camera.cpp
    src/imu_sensor.cpp
    src/pose_sensor.cpp
    src/wrench_sensor.cpp
    src/mujoco_ros2_sensors.cpp
)

ament_target_dependencies(${PROJECT_NAME}
    controller_manager
    hardware_interface
    rclcpp
    urdf
    realtime_tools
    cv_bridge
    PCL
    pcl_conversions
    tf2_ros
)

# Link libraries to the main executable
target_link_libraries(${PROJECT_NAME}
    mujoco::mujoco
    glfw
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${PROJECT_NAME}_parameters
    mujoco_rgbd_camera_parameters
)

# Add GUI simulation library
add_library(simulate_gui SHARED src/simulate_gui.cpp)
target_link_libraries(simulate_gui PUBLIC glfw)
target_link_libraries(simulate_gui PRIVATE
    mujoco::mujoco
    mujoco::libsimulate
    mujoco::platform_ui_adapter
    glfw
)

# Link simulate_gui to the main project
target_link_libraries(${PROJECT_NAME} simulate_gui)

# Install targets
install(TARGETS
    simulate_gui
    DESTINATION lib
)

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

# Install MJCF models
install(DIRECTORY mjcf DESTINATION share/${PROJECT_NAME}/)

# Install Python scripts
ament_python_install_package(scripts)
install(PROGRAMS
    scripts/xacro2mjcf.py
    scripts/urdf2mjcf.py
    DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(include)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_libraries(
    ${PROJECT_NAME}
    mujoco_system_plugins
)

pluginlib_export_plugin_description_file(mujoco_ros2_control mujoco_system_plugins.xml)

# Finalize package
ament_package()

#############
## Testing ##
#############
if(BUILD_TESTING)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_ros REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)
    install(DIRECTORY test/data/ DESTINATION share/${PROJECT_NAME}/test_data)

    function(add_ros_isolated_launch_test path)
        set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
        add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
    endfunction()

    add_ros_isolated_launch_test(test/simple_launch.test.py)
endif()

#############
## Doxygen ##
#############
find_package(Doxygen)
if(BUILD_DOC AND DOXYGEN_FOUND)
    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    configure_file(${doxyfile_in} ${doxyfile} @ONLY)
    message("Doxygen build started.")
    add_custom_target(doc ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)
    install(DIRECTORY doc/images DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/doc)
else()
    if(NOT DOXYGEN_FOUND)
        message("Doxygen needs to be installed to generate the documentation")
    endif()
endif()
