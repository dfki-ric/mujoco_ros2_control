/**
 * @file sensor_lookup.hpp
 * @brief Shared helpers for resolving MuJoCo sensors from a ros2_control <sensor>.
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

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__SENSOR_LOOKUP_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__SENSOR_LOOKUP_HPP_

#include <string>
#include <vector>

#include "mujoco/mujoco.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"

namespace mujoco_ros2_control_plugins {

/**
 * @brief Value of a `<param>` on the declaration, or @p fallback when unset.
 *
 * Templated on the declaration type so the same reader serves both kinds of
 * sensor plugin: hardware_interface::ComponentInfo for a `<sensor>` inside
 * `<ros2_control>`, and mujoco_ros2_control::MujocoRos2PluginInfo for a
 * top-level `<mujoco_ros2_plugin>`. Both carry the params in a `parameters` map.
 */
template <typename SensorInfoT>
inline std::string get_param(
        const SensorInfoT &sensor_info,
        const char *key,
        const std::string &fallback = "") {
    const auto it = sensor_info.parameters.find(key);
    if (it != sensor_info.parameters.end() && !it->second.empty()) {
        return it->second;
    }
    return fallback;
}

/**
 * @brief Value of a boolean `<param>`, or @p fallback when unset or unparsable.
 */
template <typename SensorInfoT>
inline bool get_bool_param(
        const SensorInfoT &sensor_info,
        const char *key,
        bool fallback) {
    const std::string value = get_param(sensor_info, key);
    if (value.empty()) {
        return fallback;
    }
    return value == "true" || value == "True" || value == "1";
}

/**
 * @brief Names the MuJoCo object a `<sensor>` refers to.
 *
 * Checked in order: the `<param>` keys the built-in classifier already
 * understands, then the sensor's own name. Those keys are reused so a model
 * written for the built-in path keeps working once a `plugin` param is added.
 */
template <typename SensorInfoT>
inline std::string resolve_object_name(const SensorInfoT &sensor_info) {
    for (const char *key : {"site", "body", "geom", "camera", "light", "frame"}) {
        const auto it = sensor_info.parameters.find(key);
        if (it != sensor_info.parameters.end() && !it->second.empty()) {
            return it->second;
        }
    }
    return sensor_info.name;
}

/**
 * @brief Address in `mjData::sensordata` of the first sensor of @p sensor_type
 *        attached to an object named @p object_name.
 *
 * @return The address, or -1 when the model declares no such sensor.
 */
inline int find_sensor_adr(
        const mjModel *mujoco_model, const std::string &object_name, int sensor_type) {
    for (int id = 0; id < mujoco_model->nsensor; id++) {
        if (mujoco_model->sensor_type[id] != sensor_type) continue;

        const char *object = mj_id2name(
            mujoco_model, mujoco_model->sensor_objtype[id], mujoco_model->sensor_objid[id]);
        if (object && object_name == object) {
            return mujoco_model->sensor_adr[id];
        }
    }
    return -1;
}

/**
 * @brief Exports one state interface per entry of @p mapping that the `<sensor>` declares.
 *
 * Interfaces the URDF does not ask for are skipped, and entries the plugin does
 * not know about are ignored, matching what the built-in classifier does.
 *
 * @param mapping Interface name paired with storage owned by the plugin object.
 */
inline void export_state_interfaces(
        const hardware_interface::ComponentInfo &sensor_info,
        std::vector<hardware_interface::StateInterface> &state_interfaces,
        const std::vector<std::pair<const char *, double *>> &mapping) {
    for (const auto &declared : sensor_info.state_interfaces) {
        for (const auto &[name, storage] : mapping) {
            if (declared.name == name) {
                state_interfaces.emplace_back(sensor_info.name, declared.name, storage);
                break;
            }
        }
    }
}

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__SENSOR_LOOKUP_HPP_
