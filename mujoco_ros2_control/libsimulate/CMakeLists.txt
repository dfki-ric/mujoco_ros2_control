# Copyright 2021 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)

# INTERPROCEDURAL_OPTIMIZATION is enforced when enabled.
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
# Default to GLVND if available.
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)

# This line has to appear before 'PROJECT' in order to be able to disable incremental linking
set(MSVC_INCREMENTAL_DEFAULT ON)

set(MUJOCO_DEP_VERSION_lodepng
    b4ed2cd7ecf61d29076169b49199371456d4f90b
    CACHE STRING "Version of `lodepng` to be fetched."
)

project(
  mujoco_simulate
  VERSION 3.2.7
  DESCRIPTION "MuJoCo simulate binaries"
  HOMEPAGE_URL "https://mujoco.org"
)

enable_language(C)
enable_language(CXX)
if(APPLE)
  enable_language(OBJC)
  enable_language(OBJCXX)
endif()

option(SIMULATE_BUILD_EXECUTABLE "Build the simulate executable binary." ON)
option(SIMULATE_GLFW_DYNAMIC_SYMBOLS "Whether to resolve GLFW symbols dynamically." OFF)

# Check if we are building as standalone project.
set(SIMULATE_STANDALONE ON)
set(_INSTALL_SIMULATE ON)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(SIMULATE_STANDALONE ON)
  # If standalone, do not install the samples.
  set(_INSTALL_SIMULATE OFF)
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../cmake")

if(SIMULATE_STANDALONE)
  include(SimulateOptions)
endif()
include(SimulateDependencies)

set(MUJOCO_SIMULATE_COMPILE_OPTIONS "${AVX_COMPILE_OPTIONS}" "${EXTRA_COMPILE_OPTIONS}")
set(MUJOCO_SIMULATE_LINK_OPTIONS "${EXTRA_LINK_OPTIONS}")

if(MUJOCO_HARDEN)
  if(WIN32)
    set(MUJOCO_SIMULATE_LINK_OPTIONS "${MUJOCO_SIMULATE_LINK_OPTIONS}" -Wl,/DYNAMICBASE)
  else()
    set(MUJOCO_SIMULATE_COMPILE_OPTIONS "${MUJOCO_SIMULATE_COMPILE_OPTIONS}" -fPIE -fPIC)
    if(APPLE)
      set(MUJOCO_SIMULATE_LINK_OPTIONS "${MUJOCO_SIMULATE_LINK_OPTIONS}" -Wl,-pie)
    else()
      set(MUJOCO_SIMULATE_LINK_OPTIONS "${MUJOCO_SIMULATE_LINK_OPTIONS}" -pie)
    endif()
  endif()
endif()

# Fetch lodepng dependency.
if(NOT TARGET lodepng)
  FetchContent_Declare(
    lodepng
    GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
    GIT_TAG ${MUJOCO_DEP_VERSION_lodepng}
  )

  FetchContent_GetProperties(lodepng)
  if(NOT lodepng_POPULATED)
    FetchContent_Populate(lodepng)
    # This is not a CMake project.
    set(LODEPNG_SRCS ${lodepng_SOURCE_DIR}/lodepng.cpp)
    set(LODEPNG_HEADERS ${lodepng_SOURCE_DIR}/lodepng.h)
    add_library(lodepng STATIC ${LODEPNG_HEADERS} ${LODEPNG_SRCS})
    target_compile_options(lodepng PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
    target_link_options(lodepng PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})
    target_include_directories(lodepng PUBLIC ${lodepng_SOURCE_DIR})
  endif()
endif()

# Simulate library
add_library(platform_ui_adapter OBJECT)
target_sources(
  platform_ui_adapter
  PUBLIC glfw_adapter.h glfw_dispatch.h platform_ui_adapter.h
  PRIVATE glfw_adapter.cc glfw_dispatch.cc platform_ui_adapter.cc
)
target_compile_options(platform_ui_adapter PRIVATE ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
if(APPLE)
  target_sources(platform_ui_adapter PUBLIC glfw_corevideo.h PRIVATE glfw_corevideo.mm)
  target_link_libraries(platform_ui_adapter PUBLIC "-framework CoreVideo")
endif()
target_include_directories(
  platform_ui_adapter PUBLIC .
                             $<TARGET_PROPERTY:glfw,INTERFACE_INCLUDE_DIRECTORIES>
)
target_link_libraries(platform_ui_adapter PUBLIC mujoco::mujoco)
if(SIMULATE_GLFW_DYNAMIC_SYMBOLS)
  target_compile_definitions(platform_ui_adapter PUBLIC mjGLFW_DYNAMIC_SYMBOLS)
endif()
add_library(mujoco::platform_ui_adapter ALIAS platform_ui_adapter)

add_library(libsimulate STATIC $<TARGET_OBJECTS:platform_ui_adapter>)
set_target_properties(libsimulate PROPERTIES OUTPUT_NAME simulate)
add_library(mujoco::libsimulate ALIAS libsimulate)

target_sources(
  libsimulate
  PUBLIC simulate.h
  PRIVATE simulate.cc array_safety.h
)
target_include_directories(libsimulate PUBLIC .)
target_compile_options(libsimulate PRIVATE ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
target_link_libraries(libsimulate PUBLIC lodepng mujoco::platform_ui_adapter mujoco::mujoco)
target_link_options(libsimulate PRIVATE ${MUJOCO_SIMULATE_LINK_OPTIONS})
