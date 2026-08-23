cmake_minimum_required(VERSION 3.8)
project(mujoco_ros2_control_examples)

find_package(ament_cmake REQUIRED)
# Interpreter only, for the configure-time mesh conversion below.
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# The Unitree H1/G1 URDF and meshes are not redistributed here; they are fetched and
# patched from upstream at configure time. Turn this off (e.g. for offline or
# network-restricted builds) to build only the Franka and UR examples.
option(DOWNLOAD_UNITREE_H1_ASSETS
  "Download and patch the Unitree H1 URDF and meshes at configure time" ON)

option(DOWNLOAD_UNITREE_G1_ASSETS
  "Download and patch the Unitree G1 URDF and meshes at configure time" ON)

# The IndustReal peg meshes are the upstream IsaacGymEnvs simulator assets and
# are not redistributed here either. Turn this off for offline builds; the Franka
# example then needs load_industreal_board:=false, or a task_board_config whose
# peg rows are disabled.
option(DOWNLOAD_INDUSTREAL_PEG_ASSETS
  "Download the IndustReal peg meshes from IsaacGymEnvs at configure time" ON)

# Franka's printable D435 wrist mount is published as a download on franka.de
# under plain copyright, with no licence granting redistribution, so it is not
# vendored here either. Turn this off for offline builds; the Franka example
# then needs load_realsense:=false.
option(DOWNLOAD_FRANKA_CAMERA_MOUNT_ASSET
  "Download the Franka printable D435 camera mount at configure time" ON)
###########
# INSTALL #
###########
# Shared example assets. Each robot has its own sub-directory under launch/ is
# flat, while urdf/ and config/ are split per robot to avoid filename clashes
# (every robot ships its own mujoco_actuators / ros2_control / default.rviz).
install(DIRECTORY launch config urdf meshes
        DESTINATION share/${PROJECT_NAME}
)

##############################
# UNITREE H1: GET AND PATCH  #
##############################
if(DOWNLOAD_UNITREE_H1_ASSETS)
  set(ORIGINAL_URDF_NAME "h1_2.urdf")
  set(PATCHED_URDF_NAME "unitree_h1.urdf")
  set(ORIGINAL_URDF_PATH "${CMAKE_CURRENT_BINARY_DIR}/${ORIGINAL_URDF_NAME}")
  set(PATCHED_URDF_PATH "${CMAKE_CURRENT_BINARY_DIR}/${PATCHED_URDF_NAME}")
  set(URDF_URL "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/a3b70cae6fd4a82c0e1ece633d5c6f97e88c9d76/robots/h1_2_description/${ORIGINAL_URDF_NAME}")
  set(PATCH_FILE "${CMAKE_CURRENT_SOURCE_DIR}/patches/unitree_h1.urdf.patch")

  file(DOWNLOAD ${URDF_URL} ${ORIGINAL_URDF_PATH}
       SHOW_PROGRESS
       STATUS DOWNLOAD_STATUS
       TLS_VERIFY ON)

  list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
  if(NOT STATUS_CODE EQUAL 0)
    message(FATAL_ERROR "Failed to download URDF file")
  endif()

  file(COPY ${ORIGINAL_URDF_PATH} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
  file(RENAME "${CMAKE_CURRENT_BINARY_DIR}/${ORIGINAL_URDF_NAME}" "${PATCHED_URDF_PATH}")

  execute_process(
      COMMAND patch ${PATCHED_URDF_PATH} ${PATCH_FILE}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      RESULT_VARIABLE PATCH_RESULT
  )
  if(NOT PATCH_RESULT EQUAL 0)
      message(FATAL_ERROR "Failed to apply patch to URDF file")
  endif()

  ##############
  # GET MESHES #
  ##############
  set(MESH_BASE_URL "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/a3b70cae6fd4a82c0e1ece633d5c6f97e88c9d76/robots/h1_2_description/meshes")
  set(MESH_DEST_DIR "${CMAKE_CURRENT_BINARY_DIR}/h1_2_description_meshes")

  file(MAKE_DIRECTORY ${MESH_DEST_DIR})

  set(MESH_FILES
      wrist_yaw_link.STL
      torso_link.STL
      right_wrist_roll_link.STL
      right_wrist_pitch_link.STL
      right_shoulder_yaw_link.STL
      right_shoulder_roll_link.STL
      right_shoulder_pitch_link.STL
      right_pitch_link.STL
      right_knee_link.STL
      right_hip_yaw_link.STL
      right_hip_roll_link.STL
      right_hip_pitch_link.STL
      right_hand_link.STL
      right_elbow_link.STL
      right_ankle_roll_link.STL
      right_ankle_pitch_link.STL
      right_ankle_link.STL
      right_ankle_B_rod_link.STL
      right_ankle_B_link.STL
      right_ankle_A_rod_link.STL
      right_ankle_A_link.STL
      pelvis.STL
      logo_link.STL
      link22_R.STL link22_L.STL link21_R.STL link21_L.STL
      link20_R.STL link20_L.STL link19_R.STL link19_L.STL
      link18_R.STL link18_L.STL link17_R.STL link17_L.STL
      link16_R.STL link16_L.STL link15_R.STL link15_L.STL
      link14_R.STL link14_L.STL link13_R.STL link13_L.STL
      link12_R.STL link12_L.STL link11_R.STL link11_L.STL
      left_wrist_roll_link.STL
      left_wrist_pitch_link.STL
      left_shoulder_yaw_link.STL
      left_shoulder_roll_link.STL
      left_shoulder_pitch_link.STL
      left_knee_link.STL
      left_hip_yaw_link.STL
      left_hip_roll_link.STL
      left_hip_pitch_link.STL
      left_hand_link.STL
      left_elbow_link.STL
      left_ankle_roll_link.STL
      left_ankle_pitch_link.STL
      left_ankle_B_rod_link.STL
      left_ankle_B_link.STL
      left_ankle_A_rod_link.STL
      left_ankle_A_link.STL
      R_thumb_proximal_base.STL
      R_thumb_proximal.STL
      R_thumb_intermediate.STL
      R_thumb_distal.STL
      R_ring_proximal.STL
      R_ring_intermediate.STL
      R_pinky_proximal.STL
      R_pinky_intermediate.STL
      R_middle_proximal.STL
      R_middle_intermediate.STL
      R_index_proximal.STL
      R_index_intermediate.STL
      R_hand_base_link.STL
      L_thumb_proximal_base.STL
      L_thumb_proximal.STL
      L_thumb_intermediate.STL
      L_thumb_distal.STL
      L_ring_proximal.STL
      L_ring_intermediate.STL
      L_pinky_proximal.STL
      L_pinky_intermediate.STL
      L_middle_proximal.STL
      L_middle_intermediate.STL
      L_index_proximal.STL
      L_index_intermediate.STL
      L_hand_base_link.STL
  )

  # Loop and download each file
  foreach(MESH_FILE ${MESH_FILES})
      set(REMOTE_FILE "${MESH_BASE_URL}/${MESH_FILE}")
      set(LOCAL_FILE "${MESH_DEST_DIR}/${MESH_FILE}")
      file(DOWNLOAD ${REMOTE_FILE} ${LOCAL_FILE}
           STATUS DOWNLOAD_STATUS
           TLS_VERIFY ON)
      list(GET DOWNLOAD_STATUS 0 DL_STATUS)
      if(NOT DL_STATUS EQUAL 0)
          message(FATAL_ERROR "Failed to download ${MESH_FILE}")
      endif()
  endforeach()

  install(FILES ${PATCHED_URDF_PATH}
          DESTINATION share/${PROJECT_NAME}/urdf/unitree_h1
  )

  install(DIRECTORY ${MESH_DEST_DIR}/
          DESTINATION share/${PROJECT_NAME}/meshes/unitree_h1
          FILES_MATCHING PATTERN "*.STL")
endif()

##############################
# UNITREE G1: GET AND PATCH  #
##############################
# The upstream G1 29-DOF URDF already carries the d435 (depth camera), mid360
# (LiDAR) and IMU mount frames, so no patch is needed to add sensors. We only
# strip its embedded <mujoco> block (this package adds its own) and rewrite the
# relative mesh paths to resolve against this package's install space.
if(DOWNLOAD_UNITREE_G1_ASSETS)
  set(G1_ORIGINAL_URDF_NAME "g1_29dof_rev_1_0.urdf")
  set(G1_PATCHED_URDF_NAME "unitree_g1.urdf")
  set(G1_ORIGINAL_URDF_PATH "${CMAKE_CURRENT_BINARY_DIR}/${G1_ORIGINAL_URDF_NAME}")
  set(G1_PATCHED_URDF_PATH "${CMAKE_CURRENT_BINARY_DIR}/${G1_PATCHED_URDF_NAME}")
  set(G1_URDF_URL "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/a3b70cae6fd4a82c0e1ece633d5c6f97e88c9d76/robots/g1_description/${G1_ORIGINAL_URDF_NAME}")

  file(DOWNLOAD ${G1_URDF_URL} ${G1_ORIGINAL_URDF_PATH}
       SHOW_PROGRESS
       STATUS G1_DOWNLOAD_STATUS
       TLS_VERIFY ON)

  list(GET G1_DOWNLOAD_STATUS 0 G1_STATUS_CODE)
  if(NOT G1_STATUS_CODE EQUAL 0)
    message(FATAL_ERROR "Failed to download G1 URDF file")
  endif()

  file(COPY ${G1_ORIGINAL_URDF_PATH} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
  file(RENAME "${CMAKE_CURRENT_BINARY_DIR}/${G1_ORIGINAL_URDF_NAME}" "${G1_PATCHED_URDF_PATH}")

  # (1) drop the embedded <mujoco>...</mujoco> block, (2) point mesh paths at
  # this package. Done with sed (range-delete + substitution) so it is robust to
  # upstream line-number drift, unlike a line-based patch.
  execute_process(
      COMMAND sed -i -e "/<mujoco>/,/<\\/mujoco>/d"
                     -e "s#filename=\"meshes/#filename=\"file://$(find mujoco_ros2_control_examples)/meshes/unitree_g1/#g"
                     ${G1_PATCHED_URDF_PATH}
      RESULT_VARIABLE G1_PATCH_RESULT
  )
  if(NOT G1_PATCH_RESULT EQUAL 0)
      message(FATAL_ERROR "Failed to patch G1 URDF file")
  endif()

  ##############
  # GET MESHES #
  ##############
  set(G1_MESH_BASE_URL "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/a3b70cae6fd4a82c0e1ece633d5c6f97e88c9d76/robots/g1_description/meshes")
  set(G1_MESH_DEST_DIR "${CMAKE_CURRENT_BINARY_DIR}/g1_description_meshes")

  file(MAKE_DIRECTORY ${G1_MESH_DEST_DIR})

  set(G1_MESH_FILES
      pelvis.STL
      pelvis_contour_link.STL
      logo_link.STL
      head_link.STL
      waist_yaw_link_rev_1_0.STL
      waist_roll_link_rev_1_0.STL
      torso_link_rev_1_0.STL
      left_hip_pitch_link.STL
      left_hip_roll_link.STL
      left_hip_yaw_link.STL
      left_knee_link.STL
      left_ankle_pitch_link.STL
      left_ankle_roll_link.STL
      right_hip_pitch_link.STL
      right_hip_roll_link.STL
      right_hip_yaw_link.STL
      right_knee_link.STL
      right_ankle_pitch_link.STL
      right_ankle_roll_link.STL
      left_shoulder_pitch_link.STL
      left_shoulder_roll_link.STL
      left_shoulder_yaw_link.STL
      left_elbow_link.STL
      left_wrist_roll_link.STL
      left_wrist_pitch_link.STL
      left_wrist_yaw_link.STL
      left_rubber_hand.STL
      right_shoulder_pitch_link.STL
      right_shoulder_roll_link.STL
      right_shoulder_yaw_link.STL
      right_elbow_link.STL
      right_wrist_roll_link.STL
      right_wrist_pitch_link.STL
      right_wrist_yaw_link.STL
      right_rubber_hand.STL
  )

  foreach(MESH_FILE ${G1_MESH_FILES})
      set(REMOTE_FILE "${G1_MESH_BASE_URL}/${MESH_FILE}")
      set(LOCAL_FILE "${G1_MESH_DEST_DIR}/${MESH_FILE}")
      file(DOWNLOAD ${REMOTE_FILE} ${LOCAL_FILE}
           STATUS G1_MESH_DOWNLOAD_STATUS
           TLS_VERIFY ON)
      list(GET G1_MESH_DOWNLOAD_STATUS 0 G1_DL_STATUS)
      if(NOT G1_DL_STATUS EQUAL 0)
          message(FATAL_ERROR "Failed to download ${MESH_FILE}")
      endif()
  endforeach()

  install(FILES ${G1_PATCHED_URDF_PATH}
          DESTINATION share/${PROJECT_NAME}/urdf/unitree_g1
  )

  install(DIRECTORY ${G1_MESH_DEST_DIR}/
          DESTINATION share/${PROJECT_NAME}/meshes/unitree_g1
          FILES_MATCHING PATTERN "*.STL")
endif()
###############################
# INDUSTREAL PEGS: GET MESHES #
###############################
if(DOWNLOAD_INDUSTREAL_PEG_ASSETS)
  # Post-processed simulator meshes of the IndustReal peg task, pinned to the
  # commit recorded in meshes/industreal/README.md so a rebuild cannot silently
  # change the geometry the task-board layout was measured against. IndustRealKit
  # itself publishes only the printable CAD, and does so through Git LFS, which
  # file(DOWNLOAD) cannot resolve - it would fetch the pointer file instead.
  set(PEG_BASE_URL "https://raw.githubusercontent.com/isaac-sim/IsaacGymEnvs/aeed298638a1f7b5421b38f5f3cc2d1079b6d9c3/assets/industreal/mesh/industreal_pegs")
  set(PEG_DEST_DIR "${CMAKE_CURRENT_BINARY_DIR}/industreal_pegs")

  file(MAKE_DIRECTORY ${PEG_DEST_DIR})

  # One free peg plus its pick and insert tray per shape/size, matching the
  # peg_assembly macro in urdf/industreal/industreal_task_board.urdf.xacro.
  set(PEG_FILES "")
  foreach(PEG_SHAPE round rectangular)
    foreach(PEG_SIZE 8 12 16)
      list(APPEND PEG_FILES
           "industreal_${PEG_SHAPE}_peg_${PEG_SIZE}mm.obj"
           "industreal_tray_pick_${PEG_SHAPE}_peg_${PEG_SIZE}mm.obj"
           "industreal_tray_insert_${PEG_SHAPE}_peg_${PEG_SIZE}mm.obj")
    endforeach()
  endforeach()

  foreach(PEG_FILE ${PEG_FILES})
    file(DOWNLOAD "${PEG_BASE_URL}/${PEG_FILE}" "${PEG_DEST_DIR}/${PEG_FILE}"
         STATUS DOWNLOAD_STATUS
         TLS_VERIFY ON)
    list(GET DOWNLOAD_STATUS 0 DL_STATUS)
    if(NOT DL_STATUS EQUAL 0)
      list(GET DOWNLOAD_STATUS 1 DL_ERROR)
      # Leave no truncated file behind: a partial .obj would be installed and
      # then fail inside MuJoCo instead of here.
      file(REMOVE "${PEG_DEST_DIR}/${PEG_FILE}")
      message(FATAL_ERROR "Failed to download ${PEG_FILE}: ${DL_ERROR}")
    endif()
  endforeach()

  install(DIRECTORY ${PEG_DEST_DIR}/
          DESTINATION share/${PROJECT_NAME}/meshes/industreal/pegs
          FILES_MATCHING PATTERN "*.obj")
endif()

#################################
# FRANKA CAMERA MOUNT: GET MESH #
#################################
if(DOWNLOAD_FRANKA_CAMERA_MOUNT_ASSET)
  # The mount Franka Robotics documents in guide R02241, shipped as a ZIP of two
  # STL/STEP pairs. The URL is unversioned, so the archive cannot be pinned the
  # way the IndustReal peg commit is; the recorded hash below is only checked to
  # warn when upstream republishes, because the visual origin in
  # urdf/franka/realsense_d435_wrist.urdf.xacro is measured against this
  # geometry. See "The camera mount mesh" in README.md.
  set(MOUNT_URL "https://download.franka.de/camera_mount_guide.zip")
  set(MOUNT_ARCHIVE "${CMAKE_CURRENT_BINARY_DIR}/camera_mount_guide.zip")
  set(MOUNT_DEST_DIR "${CMAKE_CURRENT_BINARY_DIR}/franka_camera_mount")
  set(MOUNT_STL "RealSenseD435_Camera_Mount.STL")
  set(MOUNT_OBJ "RealSenseD435_Camera_Mount.obj")
  set(MOUNT_KNOWN_SHA256 "0b8c53aadee5978132b1a7f2e2ef3a127b2a6e5321a1da31f27dede87b637913")

  file(DOWNLOAD ${MOUNT_URL} ${MOUNT_ARCHIVE}
       SHOW_PROGRESS
       STATUS DOWNLOAD_STATUS
       TLS_VERIFY ON)

  list(GET DOWNLOAD_STATUS 0 DL_STATUS)
  if(NOT DL_STATUS EQUAL 0)
    list(GET DOWNLOAD_STATUS 1 DL_ERROR)
    # Leave no truncated archive behind: extraction would fail on it with a
    # less informative message than the download error itself.
    file(REMOVE ${MOUNT_ARCHIVE})
    message(FATAL_ERROR "Failed to download ${MOUNT_URL}: ${DL_ERROR}")
  endif()

  file(SHA256 ${MOUNT_ARCHIVE} MOUNT_SHA256)
  if(NOT MOUNT_SHA256 STREQUAL MOUNT_KNOWN_SHA256)
    message(WARNING
      "${MOUNT_URL} no longer matches the recorded SHA256 "
      "(got ${MOUNT_SHA256}). Franka may have revised the mount; re-check the "
      "visual origin in urdf/franka/realsense_d435_wrist.urdf.xacro and update "
      "MOUNT_KNOWN_SHA256 plus the README.")
  endif()

  # `cmake -E tar` rather than file(ARCHIVE_EXTRACT) so this stays within the
  # CMake version this package declares. The archive also carries a generic
  # 1/4"-20 mount and both STEP sources; only the STL is used. Extraction
  # overwrites, so the destination is created but never cleared - removing it
  # first raced with libarchive opening its output files.
  file(MAKE_DIRECTORY ${MOUNT_DEST_DIR})
  execute_process(
      COMMAND ${CMAKE_COMMAND} -E tar xf ${MOUNT_ARCHIVE}
      WORKING_DIRECTORY ${MOUNT_DEST_DIR}
      RESULT_VARIABLE EXTRACT_RESULT
      ERROR_VARIABLE EXTRACT_ERROR
  )
  if(NOT EXTRACT_RESULT EQUAL 0)
    message(FATAL_ERROR
      "Failed to extract ${MOUNT_ARCHIVE} into ${MOUNT_DEST_DIR}: ${EXTRACT_ERROR}")
  endif()

  if(NOT EXISTS "${MOUNT_DEST_DIR}/${MOUNT_STL}")
    message(FATAL_ERROR
      "${MOUNT_STL} is missing from ${MOUNT_URL}; its contents have changed.")
  endif()

  # MuJoCo's STL decoder rejects anything over 200000 faces and the mount is
  # 295768, so it is rewritten as an OBJ - the same triangles through a loader
  # without that limit, rather than a decimation that would no longer match the
  # measured visual origin.
  execute_process(
      COMMAND ${Python3_EXECUTABLE}
              "${CMAKE_CURRENT_SOURCE_DIR}/scripts/stl_to_obj.py"
              "${MOUNT_DEST_DIR}/${MOUNT_STL}"
              "${MOUNT_DEST_DIR}/${MOUNT_OBJ}"
      RESULT_VARIABLE CONVERT_RESULT
  )
  if(NOT CONVERT_RESULT EQUAL 0)
    file(REMOVE "${MOUNT_DEST_DIR}/${MOUNT_OBJ}")
    message(FATAL_ERROR "Failed to convert ${MOUNT_STL} to OBJ")
  endif()

  install(FILES "${MOUNT_DEST_DIR}/${MOUNT_OBJ}"
          DESTINATION share/${PROJECT_NAME}/meshes/franka/realsense_d435
  )
endif()

#########
# TESTS #
#########
if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Run launch tests with an isolated ROS_DOMAIN_ID so concurrent example tests
  # (each starting its own controller_manager / mujoco_ros2_control node) do not
  # see each other on the graph.
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()

  add_ros_isolated_launch_test(test/test_franka_example.test.py)
  add_ros_isolated_launch_test(test/test_ur_example.test.py)
  add_ros_isolated_launch_test(test/test_ur_multi_example.test.py)
  add_ros_isolated_launch_test(test/test_unitree_h1_example.test.py)
  add_ros_isolated_launch_test(test/test_unitree_g1_example.test.py)
endif()

ament_package()
