cmake_minimum_required(VERSION 3.16)
project(mujoco_ros2_control_simulate_gui)

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
        glfw3
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(mujoco 3.0.0 REQUIRED)

add_subdirectory(libsimulate)

## Specify additional locations of header files
add_definitions(
        ${mujoco_DEFINITIONS}
)
link_directories(
        ${ament_cmake_DIRS}
)
include_directories(
        include
)

# Use target_include_directories for libsimulate
target_include_directories(libsimulate PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/libsimulate
        )

add_library(${PROJECT_NAME} SHARED src/simulate_gui.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC
        glfw
        )

target_link_libraries(${PROJECT_NAME} PRIVATE
        mujoco::libsimulate
        mujoco::platform_ui_adapter
        )

## Install

# Install header files
install(DIRECTORY include/ DESTINATION include)
install(FILES
        libsimulate/simulate.h
        libsimulate/glfw_adapter.h
        libsimulate/platform_ui_adapter.h
        DESTINATION
        include)

install(TARGETS
        ${PROJECT_NAME}
        DESTINATION lib
        )

install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

ament_export_dependencies(mujoco)
ament_export_include_directories(include)
ament_export_libraries(
        ${PROJECT_NAME}
)

ament_package()
