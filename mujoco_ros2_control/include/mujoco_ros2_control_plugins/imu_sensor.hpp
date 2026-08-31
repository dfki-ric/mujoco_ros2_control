/**
 * @file imu_sensor.hpp
 * @brief IMU sensor plugin: gyro, accelerometer and frame orientation.
 *
 * @author Adrian Danzglock
 * @date 2026
 *
 * @license BSD 3-Clause License
 * @copyright Copyright (c) 2026, DFKI GmbH
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

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__IMU_SENSOR_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__IMU_SENSOR_HPP_

#include <string>
#include <vector>

#include "mujoco_ros2_control/mujoco_ros2_control_sensor_interface.hpp"

namespace mujoco_ros2_control_plugins {

/**
 * @brief Exports a MuJoCo gyro/accelerometer/framequat triple as IMU state interfaces.
 *
 * The plugin equivalent of the IMU branch of the deprecated built-in classifier
 * in MujocoRos2ControlSensorLoader, and interface-compatible with it, so
 * imu_sensor_broadcaster works unchanged. Each of the three MuJoCo sensors is
 * optional: whichever the model
 * declares against the named object is picked up, the rest stay at their
 * defaults.
 *
 * @code{.xml}
 * <sensor name="link3_imu">
 *   <param name="plugin">mujoco_ros2_control/ImuSensor</param>
 *   <param name="site">link3</param>
 *   <state_interface name="angular_velocity.x"/>
 *   ...
 * </sensor>
 * @endcode
 */
class ImuSensor : public mujoco_ros2_control::MujocoRos2ControlSensorInterface {
public:
    bool registerSensor(
            const rclcpp::Node::SharedPtr &node,
            const mjModel *mujoco_model,
            const hardware_interface::ComponentInfo &sensor_info,
            std::vector<hardware_interface::StateInterface> &state_interfaces) override;

    void read(const mjData *mujoco_data) override;

private:
    std::string object_name_;

    int gyro_adr_{-1};
    int accelerometer_adr_{-1};
    int framequat_adr_{-1};

    double orientation_x_{0.0};
    double orientation_y_{0.0};
    double orientation_z_{0.0};
    double orientation_w_{1.0};
    double angular_velocity_x_{0.0};
    double angular_velocity_y_{0.0};
    double angular_velocity_z_{0.0};
    double linear_acceleration_x_{0.0};
    double linear_acceleration_y_{0.0};
    double linear_acceleration_z_{0.0};

    rclcpp::Logger logger_ = rclcpp::get_logger("imu_sensor");
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__IMU_SENSOR_HPP_
