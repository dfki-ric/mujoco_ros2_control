/**
 * @file sensor_lookup.hpp
 * @brief Shared helpers for reading a plugin's configuration and resolving its
 *        MuJoCo sensors, used by both kinds of plugin.
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

#include <algorithm>
#include <cstdint>
#include <exception>
#include <sstream>
#include <string>
#include <vector>

#include "mujoco/mujoco.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"

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
 * @brief A `<param>` holding a list: whitespace- or comma-separated, as in a URDF.
 */
inline std::vector<std::string> split_list(const std::string &value) {
    std::string separated = value;
    std::replace(separated.begin(), separated.end(), ',', ' ');
    std::istringstream stream(separated);
    std::vector<std::string> tokens;
    for (std::string token; stream >> token;) {
        tokens.push_back(token);
    }
    return tokens;
}

//
// Node-parameter readers: the same keys, one layer up. The node parameter wins,
// its default is the `<param>` of the same name on the declaration, and that
// falls back to the value given here -- so a robot can be described entirely in
// its URDF and still be retuned from the YAML that configures the controllers,
// without the description having to anticipate which values that will be.
//
// Which YAML block they belong in follows from the node the plugin was given. A
// `<mujoco_ros2_plugin>` has a node of its own, named after the declaration, so
// its parameters are keyed by that name. A `<sensor>` inside `<ros2_control>`
// shares the simulation node, so its parameters live under that node's name
// ("mujoco_ros2_control") and every key it declares has to be unique among the
// sensors that do the same.
//
// Call them from configure() and nowhere else: declaring one parameter twice
// throws, and nothing reads them again afterwards.
//

/**
 * @brief Node parameter, defaulting to the `<param>` of that name, then @p fallback.
 */
template <typename SensorInfoT>
inline std::string declare_param(
        const rclcpp::Node::SharedPtr &node,
        const SensorInfoT &sensor_info,
        const char *key,
        const std::string &fallback = "") {
    return node->declare_parameter<std::string>(key, get_param(sensor_info, key, fallback));
}

/** @brief As declare_param(), for a boolean. */
template <typename SensorInfoT>
inline bool declare_bool_param(
        const rclcpp::Node::SharedPtr &node,
        const SensorInfoT &sensor_info,
        const char *key,
        bool fallback) {
    return node->declare_parameter<bool>(key, get_bool_param(sensor_info, key, fallback));
}

/**
 * @brief As declare_param(), for an integer.
 *
 * A `<param>` that is not a number is reported and ignored, leaving @p fallback
 * as the node parameter's default: a typo in the description costs the value it
 * was meant to set, not the plugin.
 */
template <typename SensorInfoT>
inline std::int64_t declare_int_param(
        const rclcpp::Node::SharedPtr &node,
        const SensorInfoT &sensor_info,
        const char *key,
        std::int64_t fallback) {
    const std::string declared = get_param(sensor_info, key);
    std::int64_t value = fallback;
    if (!declared.empty()) {
        try {
            value = std::stoll(declared);
        } catch (const std::exception &) {
            RCLCPP_WARN(node->get_logger(),
                "<param name=\"%s\"> is not an integer: '%s'. Ignoring it.",
                key, declared.c_str());
        }
    }
    return node->declare_parameter<std::int64_t>(key, value);
}

/** @brief As declare_int_param(), for a floating-point value. */
template <typename SensorInfoT>
inline double declare_double_param(
        const rclcpp::Node::SharedPtr &node,
        const SensorInfoT &sensor_info,
        const char *key,
        double fallback) {
    const std::string declared = get_param(sensor_info, key);
    double value = fallback;
    if (!declared.empty()) {
        try {
            value = std::stod(declared);
        } catch (const std::exception &) {
            RCLCPP_WARN(node->get_logger(),
                "<param name=\"%s\"> is not a number: '%s'. Ignoring it.",
                key, declared.c_str());
        }
    }
    return node->declare_parameter<double>(key, value);
}

/**
 * @brief As declare_param(), for a list. Empty when neither source sets it.
 */
template <typename SensorInfoT>
inline std::vector<std::string> declare_string_array_param(
        const rclcpp::Node::SharedPtr &node,
        const SensorInfoT &sensor_info,
        const char *key) {
    return node->declare_parameter<std::vector<std::string>>(
        key, split_list(get_param(sensor_info, key)));
}

/** @brief As declare_string_array_param(), for a list of integers. */
template <typename SensorInfoT>
inline std::vector<std::int64_t> declare_int_array_param(
        const rclcpp::Node::SharedPtr &node,
        const SensorInfoT &sensor_info,
        const char *key) {
    std::vector<std::int64_t> declared;
    for (const auto &token : split_list(get_param(sensor_info, key))) {
        try {
            declared.push_back(std::stoll(token));
        } catch (const std::exception &) {
            RCLCPP_WARN(node->get_logger(),
                "<param name=\"%s\"> holds a non-integer: '%s'. Ignoring it.",
                key, token.c_str());
        }
    }
    return node->declare_parameter<std::vector<std::int64_t>>(key, declared);
}

/** @brief As declare_string_array_param(), for a list of numbers. */
template <typename SensorInfoT>
inline std::vector<double> declare_double_array_param(
        const rclcpp::Node::SharedPtr &node,
        const SensorInfoT &sensor_info,
        const char *key) {
    std::vector<double> declared;
    for (const auto &token : split_list(get_param(sensor_info, key))) {
        try {
            declared.push_back(std::stod(token));
        } catch (const std::exception &) {
            RCLCPP_WARN(node->get_logger(),
                "<param name=\"%s\"> holds a non-number: '%s'. Ignoring it.",
                key, token.c_str());
        }
    }
    return node->declare_parameter<std::vector<double>>(key, declared);
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
