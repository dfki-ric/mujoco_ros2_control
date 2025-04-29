/**
 * @file mujoco_ros2_sensors.cpp
 * @brief This file contains the implementation of Sensor handler.
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

#include <utility>
#include "mujoco_ros2_sensors/mujoco_ros2_sensors.hpp"

namespace mujoco_ros2_sensors {
    MujocoRos2Sensors::MujocoRos2Sensors(rclcpp::executors::MultiThreadedExecutor::SharedPtr executor, mjModel_ *model,
                                         mjData_ *data, std::map<std::string, Sensors> sensors) {
        this->executor_ = executor;
        this->mujoco_model_ = model;
        this->mujoco_data_ = data;
        this->sensors_ = std::move(sensors);

        std::vector<PoseSensorStruct> pose_sensors;
        std::vector<WrenchSensorStruct> wrench_sensors;
        std::vector<ImuSensorStruct> imu_sensors;
        for (const auto &sensor : sensors_) {
            PoseSensorStruct pose_sensor;
            WrenchSensorStruct wrench_sensor;
            ImuSensorStruct imu_sensor;
            if (sensor.second.sensor_types[0] == mjSENS_FRAMEPOS || sensor.second.sensor_types[0] == mjSENS_FRAMEQUAT) {
                for (size_t i = 0; i < sensor.second.sensor_types.size(); i++) {
                    if (sensor.second.sensor_types[i] == mjSENS_FRAMEPOS) {
                        pose_sensor.body_name = sensor.first;
                        pose_sensor.position_sensor_adr = sensor.second.sensor_addresses[i];
                        pose_sensor.position = true;
                    } else if (sensor.second.sensor_types[i] == mjSENS_FRAMEQUAT) {
                        pose_sensor.body_name = sensor.first;
                        pose_sensor.orientation_sensor_adr = sensor.second.sensor_addresses[i];
                        pose_sensor.orientation = true;
                    }
                    if (pose_sensor.frame_id.empty()) {
                        pose_sensor.frame_id = get_frame_id(sensor.second.sensor_ids[i]);
                    } else if (pose_sensor.frame_id != get_frame_id(sensor.second.sensor_ids[i])) {
                        RCLCPP_WARN(rclcpp::get_logger("sensor_handler"), "Failed to create pose sensor, frames from position and orientation sensors doesn't match");
                        continue;
                    }
                }
                pose_sensors.push_back(pose_sensor);
            }

            if (sensor.second.sensor_types[0] == mjSENS_FORCE || sensor.second.sensor_types[0] == mjSENS_TORQUE) {
                for (size_t i = 0; i < sensor.second.sensor_types.size(); i++) {
                    const auto &sensor_id = sensor.second.sensor_ids[i];
                    

                    if (sensor.second.sensor_types[i] == mjSENS_FORCE) {
                        wrench_sensor.body_name = sensor.first;
                        wrench_sensor.force_sensor_adr = sensor.second.sensor_addresses[i];
                        wrench_sensor.force = true;
                    } else if (sensor.second.sensor_types[i] == mjSENS_TORQUE) {
                        wrench_sensor.body_name = sensor.first;
                        wrench_sensor.torque_sensor_adr = sensor.second.sensor_addresses[i];
                        wrench_sensor.torque = true;
                    }
                    if (wrench_sensor.frame_id.empty()) {
                        wrench_sensor.frame_id = get_frame_id(sensor.second.sensor_ids[i]);
                    } else if (wrench_sensor.frame_id != get_frame_id(sensor.second.sensor_ids[i])) {
                        RCLCPP_WARN(rclcpp::get_logger("sensor_handler"), "Failed to create wrench sensor, frames from position and orientation sensors doesn't match");
                        continue;
                    }
                    RCLCPP_INFO(rclcpp::get_logger("sensor_handler"), "Added Sensor %s in frame %s", wrench_sensor.body_name.c_str(), wrench_sensor.frame_id.c_str());
                }
                wrench_sensors.push_back(wrench_sensor);
            }

            if (sensor.second.sensor_types[0] == mjSENS_ACCELEROMETER || sensor.second.sensor_types[0] == mjSENS_GYRO) {
                for (size_t i = 0; i < sensor.second.sensor_types.size(); i++) {
                    const auto &sensor_id = sensor.second.sensor_ids[i];

                    if (sensor.second.sensor_types[i] == mjSENS_ACCELEROMETER) {
                        imu_sensor.body_name = sensor.first;
                        imu_sensor.accel_sensor_adr = sensor.second.sensor_addresses[i];
                        imu_sensor.accel = true;
                    } else if (sensor.second.sensor_types[i] == mjSENS_GYRO) {
                        imu_sensor.body_name = sensor.first;
                        imu_sensor.gyro_sensor_adr = sensor.second.sensor_addresses[i];
                        imu_sensor.gyro = true;
                    }
                    if (imu_sensor.frame_id.empty()) {
                        imu_sensor.frame_id = get_frame_id(sensor.second.sensor_ids[i]);
                    } else if (imu_sensor.frame_id != get_frame_id(sensor.second.sensor_ids[i])) {
                        RCLCPP_WARN(rclcpp::get_logger("sensor_handler"), "Failed to create imu sensor, frames from gyro (%s) and accel (%s) sensors doesn't match", imu_sensor.frame_id.c_str(), get_frame_id(sensor.second.sensor_ids[i]).c_str());
                        continue;
                    }
                    RCLCPP_INFO(rclcpp::get_logger("sensor_handler"), "Added Sensor %s in frame %s", imu_sensor.body_name.c_str(), imu_sensor.frame_id.c_str());
                }
                imu_sensors.push_back(imu_sensor);
            }
        }

        register_pose_sensors(pose_sensors);
        register_wrench_sensors(wrench_sensors);
        register_imu_sensors(imu_sensors);
    }

    MujocoRos2Sensors::~MujocoRos2Sensors() {
        for (auto &node : pose_sensor_nodes_) {
            node.reset();
        }
        for (auto &obj : pose_sensor_objs_) {
            obj.reset();
        }
        for (auto &node : wrench_sensor_nodes_) {
            node.reset();
        }
        for (auto &obj : wrench_sensor_objs_) {
            obj.reset();
        }
        for (auto &node : imu_sensor_nodes_) {
            node.reset();
        }
        for (auto &obj : imu_sensor_objs_) {
            obj.reset();
        }
    }

    std::string MujocoRos2Sensors::get_frame_id(int sensor_id) {
        if (mujoco_model_->sensor_refid[sensor_id] == -1) {
            const auto &obj_type = mujoco_model_->sensor_objtype[sensor_id];
            const auto &obj_id = mujoco_model_->sensor_objid[sensor_id];
            return mj_id2name(mujoco_model_, obj_type, obj_id);
        } else {
            const auto &frame_id = mujoco_model_->sensor_refid[sensor_id];
            const auto &frame_type = mujoco_model_->sensor_reftype[sensor_id];
            return mj_id2name(mujoco_model_, frame_type, frame_id);
        }
    }

    void MujocoRos2Sensors::register_pose_sensors(const std::vector<PoseSensorStruct> &sensors) {
        pose_sensor_objs_.resize(sensors.size());

        for (size_t i = 0; i < sensors.size(); i++) {
            const auto &sensor = sensors[i];
            if (!sensor.isValid()) {
                std::string value;
                if (sensor.position) {
                    value = "Position";
                } else if (sensor.orientation) {
                    value = "Orientation";
                } else {
                    value = "Nothing";
                }
                RCLCPP_WARN(rclcpp::get_logger("pose_sensor_registration"), "Pose sensor have only %s", value.c_str());
            }
            std::string name = sensor.body_name;

            auto node = pose_sensor_nodes_.emplace_back(rclcpp::Node::make_shared(name + "_pose_sensor", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})));
            executor_->add_node(node);
            pose_sensor_objs_.at(i).reset(new PoseSensor(node, mujoco_model_, mujoco_data_, sensor, stop_, 100.0));
            RCLCPP_INFO(rclcpp::get_logger("pose_sensor_registration"), "[%s] frame: %s", sensor.body_name.c_str(), sensor.frame_id.c_str());
        }
    }

    void MujocoRos2Sensors::register_wrench_sensors(const std::vector<WrenchSensorStruct> &sensors) {
        wrench_sensor_objs_.resize(sensors.size());

        for (size_t i = 0; i < sensors.size(); i++) {
            const auto &sensor = sensors[i];
            if (!sensor.isValid()) {
                std::string value;
                if (sensor.force) {
                    value = "Force";
                } else if (sensor.torque) {
                    value = "Torque";
                } else {
                    value = "Nothing";
                }
                RCLCPP_WARN(rclcpp::get_logger("wrench_sensor_registration"), "Wrench sensor have only %s", value.c_str());
            }
            std::string name = sensor.body_name;

            auto node = wrench_sensor_nodes_.emplace_back(rclcpp::Node::make_shared(name + "_wrench_sensor", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})));
            executor_->add_node(node);
            wrench_sensor_objs_.at(i).reset(new WrenchSensor(node, mujoco_model_, mujoco_data_, sensor, stop_, 100.0));
            RCLCPP_INFO(rclcpp::get_logger("wrench_sensor_registration"), "[%s] frame: %s", sensor.body_name.c_str(), sensor.frame_id.c_str());
        }
    }

    void MujocoRos2Sensors::register_imu_sensors(const std::vector<ImuSensorStruct> &sensors) {
        imu_sensor_objs_.resize(sensors.size());

        for (size_t i = 0; i < sensors.size(); i++) {
            const auto &sensor = sensors[i];
            if (!sensor.isValid()) {
                std::string value;
                if (sensor.gyro) {
                    value = "Gyro";
                } else if (sensor.accel) {
                    value = "Accel";
                } else {
                    value = "Nothing";
                }
                RCLCPP_WARN(rclcpp::get_logger("imu_sensor_registration"), "IMU sensor have only %s", value.c_str());
            }
            std::string name = sensor.body_name;

            auto node = imu_sensor_nodes_.emplace_back(rclcpp::Node::make_shared(name + "_imu_sensor", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})));
            executor_->add_node(node);
            imu_sensor_objs_.at(i).reset(new ImuSensor(node, mujoco_model_, mujoco_data_, sensor, stop_, 1000.0));
            RCLCPP_INFO(rclcpp::get_logger("imu_sensor_registration"), "[%s] frame: %s", sensor.body_name.c_str(), sensor.frame_id.c_str());
        }
    }
}