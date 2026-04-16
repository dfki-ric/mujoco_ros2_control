/**
 * @file mujoco_sensors.cpp
 * @brief Sensor handling for the MuJoCo ros2_control hardware interface.
 *
 * @author Adrian Danzglock
 * @date 2023
 *
 * @license BSD 3-Clause License
 * @copyright Copyright (c) 2023, DFKI GmbH
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 *    and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 *    and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of DFKI GmbH nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <mujoco_ros2_control/mujoco_sensors.hpp>

namespace mujoco_ros2_control {

    void MujocoSensors::registerSensors(
            mjModel *mujoco_model,
            const hardware_interface::HardwareInfo &hardware_info,
            std::vector<hardware_interface::StateInterface> &state_interfaces) {

        // Pre-reserve to prevent vector reallocation that would invalidate
        // data pointers registered with state interfaces.
        imus_.reserve(hardware_info.sensors.size());
        ft_sensors_.reserve(hardware_info.sensors.size());
        pose_sensors_.reserve(hardware_info.sensors.size());

        for (const auto &sensor_info : hardware_info.sensors) {
            // Classify sensor type by inspecting state interface names
            bool has_imu_interfaces = false;
            bool has_ft_interfaces = false;
            bool has_pose_interfaces = false;

            for (const auto &si : sensor_info.state_interfaces) {
                if (si.name.find("angular_velocity") != std::string::npos ||
                    si.name.find("linear_acceleration") != std::string::npos) {
                    has_imu_interfaces = true;
                    break;
                }
                if (si.name.find("force") != std::string::npos ||
                    si.name.find("torque") != std::string::npos) {
                    has_ft_interfaces = true;
                }
                if (si.name.find("position") != std::string::npos ||
                    si.name.find("orientation") != std::string::npos) {
                    has_pose_interfaces = true;
                }
            }

            // Resolve the object name used to match MuJoCo sensors.
            // Supports all MuJoCo object types: site, body, geom, camera, light, frame.
            std::string match_name;
            bool found_param = false;
            for (const auto &key : {"site", "body", "geom", "camera", "light", "frame"}) {
                auto it = sensor_info.parameters.find(key);
                if (it != sensor_info.parameters.end()) {
                    match_name = it->second;
                    found_param = true;
                    break;
                }
            }
            if (!found_param) {
                match_name = sensor_info.name;
            }

            // --- IMU Sensor ---
            if (has_imu_interfaces) {
                ImuData imu;
                imu.name = sensor_info.name;

                for (int id = 0; id < mujoco_model->nsensor; id++) {
                    int sensor_type = mujoco_model->sensor_type[id];
                    int obj_id = mujoco_model->sensor_objid[id];
                    int obj_type = mujoco_model->sensor_objtype[id];
                    const char *obj_name_c = mj_id2name(mujoco_model, obj_type, obj_id);
                    if (!obj_name_c) continue;
                    if (std::string(obj_name_c) != match_name) continue;

                    int adr = mujoco_model->sensor_adr[id];
                    if (sensor_type == mjSENS_GYRO) {
                        imu.gyro_sensor_adr = adr;
                    } else if (sensor_type == mjSENS_ACCELEROMETER) {
                        imu.accel_sensor_adr = adr;
                    } else if (sensor_type == mjSENS_FRAMEQUAT) {
                        imu.framequat_sensor_adr = adr;
                    }
                }

                imus_.push_back(imu);
                ImuData &imu_ref = imus_.back();

                for (const auto &si : sensor_info.state_interfaces) {
                    double *data_ptr = nullptr;
                    if (si.name == "orientation.x") data_ptr = &imu_ref.orientation_x;
                    else if (si.name == "orientation.y") data_ptr = &imu_ref.orientation_y;
                    else if (si.name == "orientation.z") data_ptr = &imu_ref.orientation_z;
                    else if (si.name == "orientation.w") data_ptr = &imu_ref.orientation_w;
                    else if (si.name == "angular_velocity.x") data_ptr = &imu_ref.angular_velocity_x;
                    else if (si.name == "angular_velocity.y") data_ptr = &imu_ref.angular_velocity_y;
                    else if (si.name == "angular_velocity.z") data_ptr = &imu_ref.angular_velocity_z;
                    else if (si.name == "linear_acceleration.x") data_ptr = &imu_ref.linear_acceleration_x;
                    else if (si.name == "linear_acceleration.y") data_ptr = &imu_ref.linear_acceleration_y;
                    else if (si.name == "linear_acceleration.z") data_ptr = &imu_ref.linear_acceleration_z;

                    if (data_ptr) {
                        state_interfaces.emplace_back(sensor_info.name, si.name, data_ptr);
                    }
                }
                continue;
            }

            // --- ForceTorque Sensor ---
            if (has_ft_interfaces) {
                ForceTorqueData ft;
                ft.name = sensor_info.name;

                for (int id = 0; id < mujoco_model->nsensor; id++) {
                    int sensor_type = mujoco_model->sensor_type[id];
                    int obj_id = mujoco_model->sensor_objid[id];
                    int obj_type = mujoco_model->sensor_objtype[id];
                    const char *obj_name_c = mj_id2name(mujoco_model, obj_type, obj_id);
                    if (!obj_name_c) continue;
                    if (std::string(obj_name_c) != match_name) continue;

                    int adr = mujoco_model->sensor_adr[id];
                    if (sensor_type == mjSENS_FORCE) {
                        ft.force_sensor_adr = adr;
                    } else if (sensor_type == mjSENS_TORQUE) {
                        ft.torque_sensor_adr = adr;
                    }
                }

                ft_sensors_.push_back(ft);
                ForceTorqueData &ft_ref = ft_sensors_.back();

                for (const auto &si : sensor_info.state_interfaces) {
                    double *data_ptr = nullptr;
                    if (si.name == "force.x") data_ptr = &ft_ref.force_x;
                    else if (si.name == "force.y") data_ptr = &ft_ref.force_y;
                    else if (si.name == "force.z") data_ptr = &ft_ref.force_z;
                    else if (si.name == "torque.x") data_ptr = &ft_ref.torque_x;
                    else if (si.name == "torque.y") data_ptr = &ft_ref.torque_y;
                    else if (si.name == "torque.z") data_ptr = &ft_ref.torque_z;

                    if (data_ptr) {
                        state_interfaces.emplace_back(sensor_info.name, si.name, data_ptr);
                    }
                }
                continue;
            }

            // --- Pose Sensor ---
            if (has_pose_interfaces) {
                PoseData pose;
                pose.name = sensor_info.name;

                for (int id = 0; id < mujoco_model->nsensor; id++) {
                    int sensor_type = mujoco_model->sensor_type[id];
                    int obj_id = mujoco_model->sensor_objid[id];
                    int obj_type = mujoco_model->sensor_objtype[id];
                    const char *obj_name_c = mj_id2name(mujoco_model, obj_type, obj_id);
                    if (!obj_name_c) continue;
                    if (std::string(obj_name_c) != match_name) continue;

                    int adr = mujoco_model->sensor_adr[id];
                    if (sensor_type == mjSENS_FRAMEPOS) {
                        pose.framepos_sensor_adr = adr;
                    } else if (sensor_type == mjSENS_FRAMEQUAT) {
                        pose.framequat_sensor_adr = adr;
                    }
                }

                pose_sensors_.push_back(pose);
                PoseData &pose_ref = pose_sensors_.back();

                for (const auto &si : sensor_info.state_interfaces) {
                    double *data_ptr = nullptr;
                    if (si.name == "position.x") data_ptr = &pose_ref.position_x;
                    else if (si.name == "position.y") data_ptr = &pose_ref.position_y;
                    else if (si.name == "position.z") data_ptr = &pose_ref.position_z;
                    else if (si.name == "orientation.x") data_ptr = &pose_ref.orientation_x;
                    else if (si.name == "orientation.y") data_ptr = &pose_ref.orientation_y;
                    else if (si.name == "orientation.z") data_ptr = &pose_ref.orientation_z;
                    else if (si.name == "orientation.w") data_ptr = &pose_ref.orientation_w;

                    if (data_ptr) {
                        state_interfaces.emplace_back(sensor_info.name, si.name, data_ptr);
                    }
                }
                continue;
            }
        }
    }

    void MujocoSensors::readSensors(mjData *mujoco_data) {
        // read IMU sensor data
        for (auto &imu : imus_) {
            if (imu.gyro_sensor_adr >= 0) {
                imu.angular_velocity_x = mujoco_data->sensordata[imu.gyro_sensor_adr];
                imu.angular_velocity_y = mujoco_data->sensordata[imu.gyro_sensor_adr + 1];
                imu.angular_velocity_z = mujoco_data->sensordata[imu.gyro_sensor_adr + 2];
            }
            if (imu.accel_sensor_adr >= 0) {
                imu.linear_acceleration_x = mujoco_data->sensordata[imu.accel_sensor_adr];
                imu.linear_acceleration_y = mujoco_data->sensordata[imu.accel_sensor_adr + 1];
                imu.linear_acceleration_z = mujoco_data->sensordata[imu.accel_sensor_adr + 2];
            }
            if (imu.framequat_sensor_adr >= 0) {
                imu.orientation_w = mujoco_data->sensordata[imu.framequat_sensor_adr];
                imu.orientation_x = mujoco_data->sensordata[imu.framequat_sensor_adr + 1];
                imu.orientation_y = mujoco_data->sensordata[imu.framequat_sensor_adr + 2];
                imu.orientation_z = mujoco_data->sensordata[imu.framequat_sensor_adr + 3];
            }
        }

        // read ForceTorque sensor data
        for (auto &ft : ft_sensors_) {
            if (ft.force_sensor_adr >= 0) {
                ft.force_x = mujoco_data->sensordata[ft.force_sensor_adr];
                ft.force_y = mujoco_data->sensordata[ft.force_sensor_adr + 1];
                ft.force_z = mujoco_data->sensordata[ft.force_sensor_adr + 2];
            }
            if (ft.torque_sensor_adr >= 0) {
                ft.torque_x = mujoco_data->sensordata[ft.torque_sensor_adr];
                ft.torque_y = mujoco_data->sensordata[ft.torque_sensor_adr + 1];
                ft.torque_z = mujoco_data->sensordata[ft.torque_sensor_adr + 2];
            }
        }

        // read Pose sensor data
        for (auto &pose : pose_sensors_) {
            if (pose.framepos_sensor_adr >= 0) {
                pose.position_x = mujoco_data->sensordata[pose.framepos_sensor_adr];
                pose.position_y = mujoco_data->sensordata[pose.framepos_sensor_adr + 1];
                pose.position_z = mujoco_data->sensordata[pose.framepos_sensor_adr + 2];
            }
            if (pose.framequat_sensor_adr >= 0) {
                pose.orientation_w = mujoco_data->sensordata[pose.framequat_sensor_adr];
                pose.orientation_x = mujoco_data->sensordata[pose.framequat_sensor_adr + 1];
                pose.orientation_y = mujoco_data->sensordata[pose.framequat_sensor_adr + 2];
                pose.orientation_z = mujoco_data->sensordata[pose.framequat_sensor_adr + 3];
            }
        }
    }

}  // namespace mujoco_ros2_control
