/**
 * @file mujoco_ros2_control_plugin_loader.hpp
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

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_PLUGIN_LOADER_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_PLUGIN_LOADER_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "pluginlib/class_loader.hpp"

#include "mujoco_ros2_control/mujoco_ros2_control_plugin_interface.hpp"

namespace mujoco_ros2_control {

/// The command/state interface names a `<joint>` plugin claimed; see
/// MujocoRos2ControlPluginInterface::claimed_command_interfaces().
struct ClaimedInterfaces {
    std::set<std::string> command;
    std::set<std::string> state;
};

/**
 * @brief Owns the MujocoRos2ControlPluginInterface instances declared in the URDF.
 *
 * Sensor, GPIO and plugin-owned-joint instances are all kept in the same list:
 * once loaded, read()/write()/activate()/deactivate() and command-mode
 * switching are forwarded to all of them uniformly, regardless of which
 * `<ros2_control>` element they came from.
 *
 * @par Destruction order
 * `loader_` is declared before `plugins_` so the instances are destroyed before
 * the class loader unloads their libraries. Reversing these two members unloads
 * the shared objects while live objects still point into them, which shows up as
 * a crash in the destructor rather than at the point of the mistake.
 */
class MujocoRos2ControlPluginLoader {
public:
    MujocoRos2ControlPluginLoader();

    /**
     * @brief Instantiate a plugin for every `<sensor>`.
     *
     * A `<sensor>` names its plugin with a `plugin` parameter. One that does not
     * falls back to a deprecated classifier that guesses IMU, ForceTorque or Pose
     * from its state interface names; a `<sensor>` matching none of those is
     * skipped. A sensor whose plugin fails to load or fails its own
     * registerComponent() is reported and skipped; the simulation still comes up
     * with the remaining sensors.
     *
     * @return The number of plugins successfully registered.
     */
    size_t registerSensors(
            const rclcpp::Node::SharedPtr &node,
            const mjModel *mujoco_model,
            const hardware_interface::HardwareInfo &hardware_info,
            std::vector<hardware_interface::StateInterface> &state_interfaces,
            std::vector<hardware_interface::CommandInterface> &command_interfaces,
            const rclcpp::Logger &logger);

    /**
     * @brief Instantiate a plugin for every `<gpio>`.
     *
     * Unlike a `<sensor>`, a `<gpio>` has no built-in fallback to classify into:
     * one naming no `plugin` parameter is reported with an error and skipped.
     *
     * @return The number of plugins successfully registered.
     */
    size_t registerGpios(
            const rclcpp::Node::SharedPtr &node,
            const mjModel *mujoco_model,
            const hardware_interface::HardwareInfo &hardware_info,
            std::vector<hardware_interface::StateInterface> &state_interfaces,
            std::vector<hardware_interface::CommandInterface> &command_interfaces,
            const rclcpp::Logger &logger);

    /**
     * @brief Instantiate a plugin for every `<joint>` that names one.
     *
     * A `<joint>` naming no `plugin` parameter is left alone here: MujocoSystem
     * keeps driving it entirely through its own built-in joint logic. A joint
     * that does name one only has the specific interfaces the plugin actually
     * claims (MujocoRos2ControlPluginInterface::claimed_command_interfaces()/
     * claimed_state_interfaces(), read right after its registerComponent()
     * succeeds) taken over -- every other interface on that same joint (a
     * different control method with a matching actuator, mimic wiring, effort
     * passthrough) is unaffected and keeps going through the built-in path.
     *
     * @param joint_limits Each joint's URDF-derived limits, resolved by the
     *        caller beforehand (a plugin needs them at registerComponent() time,
     *        before the built-in path would otherwise resolve them).
     * @return Per joint naming a plugin, the interfaces it claimed.
     */
    std::map<std::string, ClaimedInterfaces> registerJoints(
            const rclcpp::Node::SharedPtr &node,
            const mjModel *mujoco_model,
            const hardware_interface::HardwareInfo &hardware_info,
            const std::map<std::string, JointLimits> &joint_limits,
            std::vector<hardware_interface::StateInterface> &state_interfaces,
            std::vector<hardware_interface::CommandInterface> &command_interfaces,
            const rclcpp::Logger &logger);

    /** @brief Forwarded from the hardware component's read(); called per control cycle. */
    void readAll(const mjData *mujoco_data);

    /** @brief Forwarded from the hardware component's write(); called per control cycle. */
    void writeAll(mjData *mujoco_data, double dt);

    /**
     * @brief Forwarded from the hardware component's perform_command_mode_switch().
     * @return False if any owned plugin rejected the switch.
     */
    bool performCommandModeSwitch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces);

    /** @brief Forwarded from the hardware component's on_activate(). */
    void activate();

    /** @brief Forwarded from the hardware component's on_deactivate(). */
    void deactivate();

    /** @brief True when no element declared a plugin, so the hooks can be skipped. */
    bool empty() const { return plugins_.empty(); }

private:
    /// Loads one plugin named by `component_info.parameters.at(kPluginParam)`, or
    /// (for a sensor) by `fallback_plugin_class` when that parameter is absent.
    /// Returns null and logs on failure, same as the public register* methods.
    std::shared_ptr<MujocoRos2ControlPluginInterface> load(
            const rclcpp::Node::SharedPtr &node,
            const mjModel *mujoco_model,
            const hardware_interface::ComponentInfo &component_info,
            const std::string &plugin_class,
            const JointLimits &joint_limits,
            std::vector<hardware_interface::StateInterface> &state_interfaces,
            std::vector<hardware_interface::CommandInterface> &command_interfaces,
            const rclcpp::Logger &logger);

    /// Declared first: must outlive every instance created from it.
    pluginlib::ClassLoader<MujocoRos2ControlPluginInterface> loader_;
    std::vector<std::shared_ptr<MujocoRos2ControlPluginInterface>> plugins_;
};

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_PLUGIN_LOADER_HPP_
