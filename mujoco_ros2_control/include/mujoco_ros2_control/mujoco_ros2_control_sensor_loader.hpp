/**
 * @file mujoco_ros2_control_sensor_loader.hpp
 * @brief Loads and drives the pluginlib-based MuJoCo sensor handlers.
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

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_SENSOR_LOADER_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_SENSOR_LOADER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_loader.hpp"

#include "mujoco_ros2_control/mujoco_ros2_control_sensor_interface.hpp"

namespace mujoco_ros2_control {

/**
 * @brief Owns the MujocoRos2ControlSensorInterface instances declared in the URDF.
 *
 * Runs alongside MujocoRos2ControlSensors rather than replacing it: only `<sensor>`
 * elements carrying a `plugin` parameter are handled here, and everything else
 * is left to the built-in classifier.
 *
 * @par Destruction order
 * `loader_` is declared before `sensors_` so the instances are destroyed before
 * the class loader unloads their libraries. Reversing these two members unloads
 * the shared objects while live objects still point into them, which shows up as
 * a crash in the destructor rather than at the point of the mistake.
 */
class MujocoRos2ControlSensorLoader {
public:
    MujocoRos2ControlSensorLoader();

    /**
     * @brief Instantiate a plugin for every `<sensor>` that names one.
     *
     * Sensors without a `plugin` parameter are skipped, leaving them to
     * MujocoRos2ControlSensors. A sensor whose plugin fails to load or fails its own
     * registerSensor() is reported and skipped; the simulation still comes up
     * with the remaining sensors.
     *
     * @return The number of plugins successfully registered.
     */
    size_t registerSensors(
            const rclcpp::Node::SharedPtr &node,
            const mjModel *mujoco_model,
            const hardware_interface::HardwareInfo &hardware_info,
            std::vector<hardware_interface::StateInterface> &state_interfaces,
            const rclcpp::Logger &logger);

    /** @brief Forwarded from the hardware component's read(); called per control cycle. */
    void readSensors(const mjData *mujoco_data);

    /** @brief Forwarded from the hardware component's on_activate(). */
    void activate();

    /** @brief Forwarded from the hardware component's on_deactivate(). */
    void deactivate();

    /** @brief True when no `<sensor>` declared a plugin, so the hooks can be skipped. */
    bool empty() const { return sensors_.empty(); }

private:
    /// Declared first: must outlive every instance created from it.
    pluginlib::ClassLoader<MujocoRos2ControlSensorInterface> loader_;
    std::vector<std::shared_ptr<MujocoRos2ControlSensorInterface>> sensors_;
};

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_SENSOR_LOADER_HPP_
