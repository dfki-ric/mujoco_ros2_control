/**
 * @file mujoco_ros2_control_sensor_loader.cpp
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

#include "mujoco_ros2_control/mujoco_ros2_control_sensor_loader.hpp"

namespace mujoco_ros2_control {

namespace {

/**
 * @brief Guess a sensor's plugin class from its state interface names.
 *
 * The fallback for a `<sensor>` that names no plugin: a substring match on
 * interface names, so it can only ever recognise these three kinds, and
 * anything whose interface names happen to contain "force", "position" or
 * "orientation" is misclassified into one of them.
 *
 * @return The class to load, or empty when nothing matched.
 */
std::string classify(const hardware_interface::ComponentInfo &sensor_info) {
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

    if (has_imu_interfaces) return "mujoco_ros2_control/ImuSensor";
    if (has_ft_interfaces) return "mujoco_ros2_control/ForceTorqueSensor";
    if (has_pose_interfaces) return "mujoco_ros2_control/PoseSensor";
    return "";
}

}  // namespace

MujocoRos2ControlSensorLoader::MujocoRos2ControlSensorLoader()
    : loader_("mujoco_ros2_control", "mujoco_ros2_control::MujocoRos2ControlSensorInterface") {}

size_t MujocoRos2ControlSensorLoader::registerSensors(
        const rclcpp::Node::SharedPtr &node,
        const mjModel *mujoco_model,
        const hardware_interface::HardwareInfo &hardware_info,
        std::vector<hardware_interface::StateInterface> &state_interfaces,
        const rclcpp::Logger &logger) {

    for (const auto &sensor_info : hardware_info.sensors) {
        std::string plugin_class;
        const auto plugin_param = sensor_info.parameters.find(kSensorPluginParam);
        if (plugin_param != sensor_info.parameters.end()) {
            plugin_class = plugin_param->second;
        } else {
            plugin_class = classify(sensor_info);
            if (plugin_class.empty()) {
                continue;
            }
            RCLCPP_WARN(logger,
                "Sensor '%s' names no plugin, so it was matched to '%s' by its state "
                "interfaces. This fallback classifier is deprecated and scheduled for "
                "removal. Add <param name=\"%s\">%s</param> to this <sensor> to make "
                "the choice explicit.",
                sensor_info.name.c_str(), plugin_class.c_str(), kSensorPluginParam,
                plugin_class.c_str());
        }

        std::shared_ptr<MujocoRos2ControlSensorInterface> sensor;
        try {
            sensor = loader_.createSharedInstance(plugin_class);
        } catch (const pluginlib::PluginlibException &e) {
            RCLCPP_ERROR(logger,
                "Sensor '%s': could not load plugin '%s', skipping it: %s",
                sensor_info.name.c_str(), plugin_class.c_str(), e.what());
            continue;
        }

        sensor->set_name(sensor_info.name);

        // Keep the instance alive across registerSensor(): it may hand out
        // pointers into itself, so it must be owned before it is asked to.
        sensors_.push_back(sensor);

        if (!sensor->registerSensor(node, mujoco_model, sensor_info, state_interfaces)) {
            RCLCPP_ERROR(logger,
                "Sensor '%s': plugin '%s' rejected its configuration, skipping it.",
                sensor_info.name.c_str(), plugin_class.c_str());
            sensors_.pop_back();
            continue;
        }

        RCLCPP_INFO(logger, "Sensor '%s': loaded plugin '%s'",
                    sensor_info.name.c_str(), plugin_class.c_str());
    }

    return sensors_.size();
}

void MujocoRos2ControlSensorLoader::readSensors(const mjData *mujoco_data) {
    for (auto &sensor : sensors_) {
        sensor->read(mujoco_data);
    }
}

void MujocoRos2ControlSensorLoader::activate() {
    for (auto &sensor : sensors_) {
        sensor->activate();
    }
}

void MujocoRos2ControlSensorLoader::deactivate() {
    for (auto &sensor : sensors_) {
        sensor->deactivate();
    }
}

}  // namespace mujoco_ros2_control
