/**
 * @file mujoco_ros2_sensors.hpp
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
#ifndef MUJOCO_ROS2_CONTROL_MUJOCO_ROS2_SENSORS_HPP
#define MUJOCO_ROS2_CONTROL_MUJOCO_ROS2_SENSORS_HPP

#include "chrono"

// MuJoCo header file
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "cstdio"
#include "GL/gl.h"

// ROS header
#include "rclcpp/rclcpp.hpp"
#include "mujoco_ros2_sensors/pose_sensor.hpp"
#include "mujoco_ros2_sensors/wrench_sensor.hpp"
#include "mujoco_ros2_sensors/imu_sensor.hpp"

using namespace std::chrono_literals;

namespace mujoco_ros2_sensors {
    class MujocoRos2Sensors {
    public:
        struct Sensors {
            int obj_type;
            std::vector<std::string> sensor_names;
            std::vector<int> sensor_ids;
            std::vector<int> sensor_types;
            std::vector<int> sensor_addresses;
            std::vector<int> sensor_dimensions;
        };

        MujocoRos2Sensors(rclcpp::executors::MultiThreadedExecutor::SharedPtr executor, mjModel_ *model, mjData_ *data, std::map<std::string, Sensors> sensors);

        ~MujocoRos2Sensors();
    private:
        std::atomic<bool>* stop_;

        rclcpp::Node::SharedPtr nh_; ///< Shared pointer to the ROS 2 Node object used for communication and coordination.

        mjModel* mujoco_model_ = nullptr; ///< Pointer to the Mujoco model object used for rendering and simulation.
        mjData* mujoco_data_ = nullptr; ///< Pointer to the Mujoco data object representing the current state of the simulation.
        rclcpp::Time stamp_; ///< ROS 2 timestamp representing the time when camera data was last updated.

        std::map<std::string, Sensors> sensors_;
        
        rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

        // Pose Sensor
        std::vector<PoseSensorStruct> pose_sensors_;
        std::vector<rclcpp::Node::SharedPtr> pose_sensor_nodes_;
        std::vector<std::shared_ptr<mujoco_ros2_sensors::PoseSensor>> pose_sensor_objs_;
        void register_pose_sensors(const std::vector<PoseSensorStruct> &sensors);

        // Wrench Sensor
        std::vector<WrenchSensorStruct> wrench_sensors_;
        std::vector<rclcpp::Node::SharedPtr> wrench_sensor_nodes_;
        std::vector<std::shared_ptr<mujoco_ros2_sensors::WrenchSensor>> wrench_sensor_objs_;
        void register_wrench_sensors(const std::vector<WrenchSensorStruct> &sensors);

        // IMU Sensor
        std::vector<ImuSensorStruct> imu_sensors_;
        std::vector<rclcpp::Node::SharedPtr> imu_sensor_nodes_;
        std::vector<std::shared_ptr<mujoco_ros2_sensors::ImuSensor>> imu_sensor_objs_;
        void register_imu_sensors(const std::vector<ImuSensorStruct> &sensors);

        std::string get_frame_id(int sensor_id);
    };
}
#endif //MUJOCO_ROS2_CONTROL_MUJOCO_ROS2_SENSORS_HPP
