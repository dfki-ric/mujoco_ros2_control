cmake_minimum_required(VERSION 3.6)
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
set(CMAKE_BUILD_TYPE Release)

# Export compile commands for IDEs
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# Set OpenGL preferences (GLVND enables OpenGL::EGL for offscreen rendering)
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

# Package dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    rclcpp
    std_msgs
    rosgraph_msgs
    controller_manager
    hardware_interface
    urdf
    ament_lint_auto
    ament_index_cpp
    realtime_tools
    OpenCV
    PCL
    pcl_conversions
    cv_bridge
    generate_parameter_library
    Eigen3
    tf2_ros
    pluginlib
    sensor_msgs
    std_srvs
    rosidl_default_generators
    tinyxml2_vendor
    TinyXML2
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
  GIT_TAG 3.12.0  # MuJoCo version
)

# Optionally set MuJoCo build options.
# These have to use MuJoCo's own MUJOCO_BUILD_* spellings: its CMakeLists never
# reads the generic BUILD_EXAMPLES/BUILD_SIMULATE names, so samples, tests and
# benchmarks were built regardless (pulling in abseil, googletest and benchmark).
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_SIMULATE ON CACHE BOOL "" FORCE)
set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL "" FORCE)
set(MUJOCO_SIMULATE_USE_SYSTEM_GLFW ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(mujoco)

## Workaround by creating dummy files in each of the necessary by --symlink-install
file(GLOB _mujoco_dep_configs
    "${CMAKE_BINARY_DIR}/_deps/ccd-build/*.cmake"
    "${CMAKE_BINARY_DIR}/_deps/ccd-build/*.pc"
    "${CMAKE_BINARY_DIR}/_deps/qhull-build/*.cmake"
    "${CMAKE_BINARY_DIR}/_deps/qhull-build/*.pc"
)
foreach(_dep_file ${_mujoco_dep_configs})
    get_filename_component(_fname "${_dep_file}" NAME)
    set(_stub "${CMAKE_BINARY_DIR}/${_fname}")
    if(NOT EXISTS "${_stub}")
        file(WRITE "${_stub}" "# Auto-generated stub by mujoco_ros2_control workaround\n")
    endif()
endforeach()

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
    ${CMAKE_BINARY_DIR}/_deps/lodepng-src
)

# Generate parameter libraries
generate_parameter_library(${PROJECT_NAME}_parameters
    src/${PROJECT_NAME}_parameters.yaml
)

generate_parameter_library(mujoco_rgbd_camera_parameters
    src/mujoco_rgbd_camera_parameters.yaml
)

generate_parameter_library(mujoco_gl_lidar_parameters
    src/mujoco_gl_lidar_parameters.yaml
)

rosidl_generate_interfaces(${PROJECT_NAME}
    srv/StepSimulation.srv
    DEPENDENCIES builtin_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME}
    rosidl_typesupport_cpp
)

# Add system plugin
add_library(mujoco_system_plugins SHARED
    src/mujoco_system.cpp
    src/mujoco_ros2_control_sensors.cpp
    src/mujoco_ros2_control_sensor_loader.cpp
)
ament_target_dependencies(mujoco_system_plugins
    hardware_interface
    pluginlib
    rclcpp
    urdf
)

# mujoco_system.cpp and mujoco_ros2_control_sensors.cpp call mj_name2id(), mj_id2name() and
# mj_forward(), so this plugin library has to carry libmujoco itself rather
# than relying on whichever process happens to dlopen it.
target_link_libraries(mujoco_system_plugins
    mujoco::mujoco
)

# The MujocoRos2ControlSensorInterface implementations shipped with this package, one file
# per plugin under include/ and src/mujoco_ros2_control_plugins.
# Nothing may link this library: pluginlib has to be the one that dlopens it, or
# class_loader registers its factories outside its own bookkeeping and then
# reports "no factory exists for it" when the class is requested.
add_library(mujoco_ros2_control_plugins SHARED
    src/mujoco_ros2_control_plugins/imu_sensor.cpp
    src/mujoco_ros2_control_plugins/force_torque_sensor.cpp
    src/mujoco_ros2_control_plugins/pose_sensor.cpp
)
ament_target_dependencies(mujoco_ros2_control_plugins
    hardware_interface
    pluginlib
    rclcpp
    realtime_tools
    std_msgs
)
target_link_libraries(mujoco_ros2_control_plugins
    mujoco::mujoco
)

set(MUJOCO_ROS2_CONTROL_NODE_TARGET ${PROJECT_NAME}_node)

# Add executable for the main ROS2 control node.
# The CMake target name must differ from PROJECT_NAME because rosidl creates a
# generated interface target with the package name.
# The camera and lidar implementations. A library of their own because two
# things need them: the node, whose site-prefix discovery still creates them
# directly, and the <mujoco_ros2_plugin> plugins that wrap them. Compiling them
# into the executable would leave the plugin .so with undefined symbols.
add_library(mujoco_sensor_implementations SHARED
    src/mujoco_depth_camera.cpp
    src/mujoco_gl_lidar.cpp
)
ament_target_dependencies(mujoco_sensor_implementations
    rclcpp
    cv_bridge
    PCL
    pcl_conversions
    sensor_msgs
)
target_link_libraries(mujoco_sensor_implementations
    mujoco::mujoco
    OpenGL::EGL
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    mujoco_rgbd_camera_parameters
    mujoco_gl_lidar_parameters
)

# The MujocoRos2PluginInterface implementations shipped with this package: thin
# adapters that let the camera and lidar above be declared as
# <mujoco_ros2_plugin> instead of found by a site prefix.
# Nothing may link this library: pluginlib has to be the one that dlopens it.
add_library(mujoco_ros2_plugins SHARED
    src/mujoco_ros2_plugins/depth_camera_sensor.cpp
    src/mujoco_ros2_plugins/lidar_sensor.cpp
)
ament_target_dependencies(mujoco_ros2_plugins
    hardware_interface  # sensor_lookup.hpp, for the shared <param> readers
    pluginlib
    rclcpp
    sensor_msgs
)
target_link_libraries(mujoco_ros2_plugins
    mujoco::mujoco
    mujoco_sensor_implementations
)

add_executable(${MUJOCO_ROS2_CONTROL_NODE_TARGET}
    src/mujoco_ros2_control_plugin.cpp
    src/mujoco_ros2_plugin_loader.cpp
)

set_target_properties(${MUJOCO_ROS2_CONTROL_NODE_TARGET}
    PROPERTIES OUTPUT_NAME ${PROJECT_NAME}
)

ament_target_dependencies(${MUJOCO_ROS2_CONTROL_NODE_TARGET}
    controller_manager
    hardware_interface
    rclcpp
    urdf
    ament_index_cpp
    realtime_tools
    cv_bridge
    PCL
    pcl_conversions
    sensor_msgs
    std_srvs
    tf2_ros
)

# Link libraries to the main executable
target_link_libraries(${MUJOCO_ROS2_CONTROL_NODE_TARGET}
    mujoco::mujoco
    mujoco_sensor_implementations
    tinyxml2::tinyxml2
    glfw
    OpenGL::EGL
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${PROJECT_NAME}_parameters
    mujoco_rgbd_camera_parameters
    mujoco_gl_lidar_parameters
    "${cpp_typesupport_target}"
)

# Add GUI simulation library
add_library(simulate_gui SHARED src/simulate_gui.cpp)
target_link_libraries(simulate_gui PUBLIC glfw)
target_link_libraries(simulate_gui PRIVATE
    mujoco::mujoco
    mujoco::platform_ui_adapter
    lodepng
    glfw
)

# Link simulate_gui to the main node
target_link_libraries(${MUJOCO_ROS2_CONTROL_NODE_TARGET} simulate_gui)

# Install targets
install(TARGETS
    simulate_gui
    DESTINATION lib
)

install(TARGETS
    ${MUJOCO_ROS2_CONTROL_NODE_TARGET}
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    mujoco_system_plugins
    mujoco_ros2_control_plugins
    mujoco_sensor_implementations
    mujoco_ros2_plugins
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# MuJoCo builds its engine plugins but has no install rules for them, so they
# never leave the build tree on their own. Install them into a directory of
# their own: the node dlopen's every shared library it finds there, so it must
# hold nothing else. $ORIGIN/.. lets them resolve libmujoco.so in lib/.
set(MUJOCO_ENGINE_PLUGIN_TARGETS sensor elasticity actuator sdf_plugin)
foreach(_plugin IN LISTS MUJOCO_ENGINE_PLUGIN_TARGETS)
    if(TARGET ${_plugin})
        set_target_properties(${_plugin} PROPERTIES INSTALL_RPATH "$ORIGIN/..")
        install(TARGETS ${_plugin} LIBRARY DESTINATION lib/mujoco_plugin)
    else()
        message(WARNING "MuJoCo engine plugin target '${_plugin}' not found; it will be unavailable at runtime.")
    endif()
endforeach()

# Install the public headers. Needed by out-of-tree packages implementing
# MujocoRos2ControlSensorInterface; without this only the rosidl-generated srv headers
# land in the include prefix that ament_export_include_directories advertises.
install(DIRECTORY include/ DESTINATION include)

# Install MJCF models
install(DIRECTORY mjcf DESTINATION share/${PROJECT_NAME}/)

# Install Python scripts
ament_python_install_package(scripts)
install(PROGRAMS
    scripts/xacro2mjcf.py
    scripts/urdf2mjcf.py
    scripts/dae2stl.py
    scripts/prepare_terrain.py
    DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(include)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rosidl_default_runtime)
# MuJoCo installs its own package config into this prefix (lib/cmake/mujoco), so
# re-exporting it lets downstream packages -- engine plugins in particular -- get
# mujoco::mujoco for exactly the version fetched here, instead of fetching a
# second copy that would not share libmujoco.so at runtime.
ament_export_dependencies(mujoco)
ament_export_dependencies(hardware_interface)
ament_export_dependencies(pluginlib)
ament_export_libraries(
    mujoco_system_plugins
    mujoco_sensor_implementations
)

pluginlib_export_plugin_description_file(mujoco_ros2_control mujoco_system_plugins.xml)
pluginlib_export_plugin_description_file(mujoco_ros2_control mujoco_ros2_control_sensor_plugins.xml)
pluginlib_export_plugin_description_file(mujoco_ros2_control mujoco_ros2_plugins.xml)

# Finalize package
ament_package()

#############
## Testing ##
#############
set(BUILD_TESTING ON CACHE BOOL "" FORCE) # Set to off/ comment out to disable building tests
if(BUILD_TESTING)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_ros REQUIRED)
    find_package(ament_cmake_pytest REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)
    install(DIRECTORY test/data/ DESTINATION share/${PROJECT_NAME}/test_data)

    function(add_ros_isolated_launch_test path)
        set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
        add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
    endfunction()

    add_ros_isolated_launch_test(test/simple_launch.test.py)
    add_ros_isolated_launch_test(test/test_pose_rigid.test.py)
    add_ros_isolated_launch_test(test/test_multi_urdf.test.py)
    add_ros_isolated_launch_test(test/test_synchronous_rl.test.py TIMEOUT 90)
    add_ros_isolated_launch_test(test/test_sensor_plugins.test.py TIMEOUT 120)

    ament_add_pytest_test(test_sensors_unit
        test/test_sensors_unit.py
        TIMEOUT 60
    )

    ament_add_pytest_test(test_prepare_terrain
        test/test_prepare_terrain.py
        TIMEOUT 120
    )
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
