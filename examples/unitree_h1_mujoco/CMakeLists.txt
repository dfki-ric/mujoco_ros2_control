cmake_minimum_required(VERSION 3.8)
project(unitree_h1_mujoco)

# find dependencies
find_package(ament_cmake REQUIRED)


######################
# GET AND PATCH URDF #
######################
set(ORIGINAL_URDF_NAME "h1_2.urdf")
set(PATCHED_URDF_NAME "unitree_h1.urdf")
set(ORIGINAL_URDF_PATH "${CMAKE_BINARY_DIR}/${ORIGINAL_URDF_NAME}")
set(PATCHED_URDF_PATH "${CMAKE_BINARY_DIR}/${PATCHED_URDF_NAME}")
set(URDF_URL "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/a3b70cae6fd4a82c0e1ece633d5c6f97e88c9d76/robots/h1_2_description/${ORIGINAL_URDF_NAME}")
set(PATCH_FILE "${CMAKE_SOURCE_DIR}/patches/unitree_h1.urdf.patch")

file(DOWNLOAD ${URDF_URL} ${ORIGINAL_URDF_PATH}
     SHOW_PROGRESS
     STATUS DOWNLOAD_STATUS
     TLS_VERIFY ON)

list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
if(NOT STATUS_CODE EQUAL 0)
  message(FATAL_ERROR "Failed to download URDF file")
endif()

file(COPY ${ORIGINAL_URDF_PATH} DESTINATION ${CMAKE_BINARY_DIR})
file(RENAME "${CMAKE_BINARY_DIR}/${ORIGINAL_URDF_NAME}" "${PATCHED_URDF_PATH}")

execute_process(
    COMMAND patch ${PATCHED_URDF_PATH} ${PATCH_FILE}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    RESULT_VARIABLE PATCH_RESULT
)
if(NOT PATCH_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to apply patch to URDF file")
endif()

##############
# GET MESHES #
##############
set(MESH_BASE_URL "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/a3b70cae6fd4a82c0e1ece633d5c6f97e88c9d76/robots/h1_2_description/meshes")
set(MESH_DEST_DIR "${CMAKE_BINARY_DIR}/h1_2_description_meshes")

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


###########
# INSTALL #
###########
install(DIRECTORY launch urdf config
        DESTINATION share/${PROJECT_NAME}
)

install(FILES ${PATCHED_URDF_PATH}
        DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY ${MESH_DEST_DIR}/
        DESTINATION share/${PROJECT_NAME}/meshes
        FILES_MATCHING PATTERN "*.STL")

ament_package()
