/**
 * @file mujoco_ros2_control_plugin_loader.cpp
 * @brief Loads and drives the pluginlib-based MuJoCo <sensor>/<gpio>/<joint> handlers.
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

#include "mujoco_ros2_control/mujoco_ros2_control_plugin_loader.hpp"

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

MujocoRos2ControlPluginLoader::MujocoRos2ControlPluginLoader()
    : loader_("mujoco_ros2_control", "mujoco_ros2_control::MujocoRos2ControlPluginInterface") {}

std::shared_ptr<MujocoRos2ControlPluginInterface> MujocoRos2ControlPluginLoader::load(
        const rclcpp::Node::SharedPtr &node,
        const mjModel *mujoco_model,
        const hardware_interface::ComponentInfo &component_info,
        const std::string &plugin_class,
        const JointLimits &joint_limits,
        std::vector<hardware_interface::StateInterface> &state_interfaces,
        std::vector<hardware_interface::CommandInterface> &command_interfaces,
        const rclcpp::Logger &logger) {

    std::shared_ptr<MujocoRos2ControlPluginInterface> plugin;
    try {
        plugin = loader_.createSharedInstance(plugin_class);
    } catch (const pluginlib::PluginlibException &e) {
        RCLCPP_ERROR(logger,
            "'%s': could not load plugin '%s', skipping it: %s",
            component_info.name.c_str(), plugin_class.c_str(), e.what());
        return nullptr;
    }

    plugin->set_name(component_info.name);

    // Keep the instance alive across registerComponent(): it may hand out
    // pointers into itself, so it must be owned before it is asked to.
    plugins_.push_back(plugin);

    if (!plugin->registerComponent(node, mujoco_model, component_info, joint_limits,
                                    state_interfaces, command_interfaces)) {
        RCLCPP_ERROR(logger,
            "'%s': plugin '%s' rejected its configuration, skipping it.",
            component_info.name.c_str(), plugin_class.c_str());
        plugins_.pop_back();
        return nullptr;
    }

    RCLCPP_INFO(logger, "'%s': loaded plugin '%s'",
                component_info.name.c_str(), plugin_class.c_str());
    return plugin;
}

size_t MujocoRos2ControlPluginLoader::registerSensors(
        const rclcpp::Node::SharedPtr &node,
        const mjModel *mujoco_model,
        const hardware_interface::HardwareInfo &hardware_info,
        std::vector<hardware_interface::StateInterface> &state_interfaces,
        std::vector<hardware_interface::CommandInterface> &command_interfaces,
        const rclcpp::Logger &logger) {

    size_t count = 0;
    for (const auto &sensor_info : hardware_info.sensors) {
        std::string plugin_class;
        const auto plugin_param = sensor_info.parameters.find(kPluginParam);
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
                sensor_info.name.c_str(), plugin_class.c_str(), kPluginParam,
                plugin_class.c_str());
        }

        if (load(node, mujoco_model, sensor_info, plugin_class, JointLimits{},
                 state_interfaces, command_interfaces, logger)) {
            count++;
        }
    }

    return count;
}

size_t MujocoRos2ControlPluginLoader::registerGpios(
        const rclcpp::Node::SharedPtr &node,
        const mjModel *mujoco_model,
        const hardware_interface::HardwareInfo &hardware_info,
        std::vector<hardware_interface::StateInterface> &state_interfaces,
        std::vector<hardware_interface::CommandInterface> &command_interfaces,
        const rclcpp::Logger &logger) {

    size_t count = 0;
    for (const auto &gpio_info : hardware_info.gpios) {
        const auto plugin_param = gpio_info.parameters.find(kPluginParam);
        if (plugin_param == gpio_info.parameters.end()) {
            RCLCPP_ERROR(logger,
                "GPIO '%s' names no plugin. Add <param name=\"%s\">...</param> to this "
                "<gpio>; unlike a <sensor>, there is no built-in GPIO handling to fall "
                "back to.",
                gpio_info.name.c_str(), kPluginParam);
            continue;
        }

        if (load(node, mujoco_model, gpio_info, plugin_param->second, JointLimits{},
                 state_interfaces, command_interfaces, logger)) {
            count++;
        }
    }

    return count;
}

std::map<std::string, ClaimedInterfaces> MujocoRos2ControlPluginLoader::registerJoints(
        const rclcpp::Node::SharedPtr &node,
        const mjModel *mujoco_model,
        const hardware_interface::HardwareInfo &hardware_info,
        const std::map<std::string, JointLimits> &joint_limits,
        std::vector<hardware_interface::StateInterface> &state_interfaces,
        std::vector<hardware_interface::CommandInterface> &command_interfaces,
        const rclcpp::Logger &logger) {

    std::map<std::string, ClaimedInterfaces> claimed;
    for (const auto &joint_info : hardware_info.joints) {
        const auto plugin_param = joint_info.parameters.find(kPluginParam);
        if (plugin_param == joint_info.parameters.end()) {
            // No plugin named: MujocoSystem's built-in joint logic handles it.
            continue;
        }

        JointLimits limits;
        const auto limits_it = joint_limits.find(joint_info.name);
        if (limits_it != joint_limits.end()) {
            limits = limits_it->second;
        }

        auto plugin = load(node, mujoco_model, joint_info, plugin_param->second, limits,
                            state_interfaces, command_interfaces, logger);
        if (!plugin) {
            continue;
        }

        ClaimedInterfaces &claims = claimed[joint_info.name];
        claims.command = plugin->claimed_command_interfaces();
        claims.state = plugin->claimed_state_interfaces();
    }

    return claimed;
}

void MujocoRos2ControlPluginLoader::readAll(const mjData *mujoco_data) {
    for (auto &plugin : plugins_) {
        plugin->read(mujoco_data);
    }
}

void MujocoRos2ControlPluginLoader::writeAll(mjData *mujoco_data, double dt) {
    for (auto &plugin : plugins_) {
        plugin->write(mujoco_data, dt);
    }
}

bool MujocoRos2ControlPluginLoader::performCommandModeSwitch(
        const std::vector<std::string> &start_interfaces,
        const std::vector<std::string> &stop_interfaces) {
    bool accepted = true;
    for (auto &plugin : plugins_) {
        if (!plugin->perform_command_mode_switch(start_interfaces, stop_interfaces)) {
            accepted = false;
        }
    }
    return accepted;
}

void MujocoRos2ControlPluginLoader::activate() {
    for (auto &plugin : plugins_) {
        plugin->activate();
    }
}

void MujocoRos2ControlPluginLoader::deactivate() {
    for (auto &plugin : plugins_) {
        plugin->deactivate();
    }
}

}  // namespace mujoco_ros2_control
