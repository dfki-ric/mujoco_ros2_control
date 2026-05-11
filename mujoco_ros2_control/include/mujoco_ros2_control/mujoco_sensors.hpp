/**
 * @file mujoco_sensors.hpp
 * @brief Sensor handling for the MuJoCo ros2_control hardware interface.
 *
 * Provides registration and reading of IMU, ForceTorque, and Pose sensors
 * that are exposed as ros2_control state interfaces.
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

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_SENSORS_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_SENSORS_HPP_

#include <string>
#include <vector>

#include "mujoco/mujoco.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace mujoco_ros2_control {

    struct ImuData {
        std::string name;
        double orientation_x{0.0};
        double orientation_y{0.0};
        double orientation_z{0.0};
        double orientation_w{1.0};
        double angular_velocity_x{0.0};
        double angular_velocity_y{0.0};
        double angular_velocity_z{0.0};
        double linear_acceleration_x{0.0};
        double linear_acceleration_y{0.0};
        double linear_acceleration_z{0.0};
        int gyro_sensor_adr{-1};
        int accel_sensor_adr{-1};
        int framequat_sensor_adr{-1};
    };

    struct ForceTorqueData {
        std::string name;
        double force_x{0.0};
        double force_y{0.0};
        double force_z{0.0};
        double torque_x{0.0};
        double torque_y{0.0};
        double torque_z{0.0};
        int force_sensor_adr{-1};
        int torque_sensor_adr{-1};
    };

    struct PoseData {
        std::string name;
        double position_x{0.0};
        double position_y{0.0};
        double position_z{0.0};
        double orientation_x{0.0};
        double orientation_y{0.0};
        double orientation_z{0.0};
        double orientation_w{1.0};
        int framepos_sensor_adr{-1};
        int framequat_sensor_adr{-1};
    };

    class MujocoSensors {
    public:
        /**
         * @brief Register all sensors from the hardware info.
         *
         * Inspects each sensor's state interfaces to classify it as IMU, ForceTorque,
         * or Pose, then finds matching MuJoCo sensors and registers state interfaces.
         * Supports all MuJoCo object types (site, body, geom, camera, light, frame)
         * for sensor matching.
         */
        void registerSensors(
                mjModel *mujoco_model,
                const hardware_interface::HardwareInfo &hardware_info,
                std::vector<hardware_interface::StateInterface> &state_interfaces);

        /**
         * @brief Read all sensor values from the MuJoCo simulation data.
         */
        void readSensors(mjData *mujoco_data);

    private:
        std::vector<ImuData> imus_;
        std::vector<ForceTorqueData> ft_sensors_;
        std::vector<PoseData> pose_sensors_;
    };

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_SENSORS_HPP_
