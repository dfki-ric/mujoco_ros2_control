/**
 * @file mujoco_ros2_control_plugin_interface.hpp
 * @brief Base class for pluginlib-loaded MuJoCo <sensor>/<gpio>/<joint> handlers.
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

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_PLUGIN_INTERFACE_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_PLUGIN_INTERFACE_HPP_

#include <limits>
#include <set>
#include <string>
#include <vector>

#include "mujoco/mujoco.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "rclcpp/rclcpp.hpp"

namespace mujoco_ros2_control {

/**
 * @brief The `<param>` on a `<sensor>`, `<gpio>` or `<joint>` that names a
 *        MujocoRos2ControlPluginInterface plugin.
 *
 * An XML attribute cannot be used: ros2_control's parser drops unknown
 * attributes on these elements without error.
 *
 * On a `<sensor>`, its absence is what falls back to
 * MujocoRos2ControlPluginLoader's deprecated built-in classifier, which
 * guesses a plugin from state interface names instead of being told one
 * explicitly. A `<joint>` naming no plugin is simply handled by MujocoSystem's
 * own built-in joint logic instead of a plugin. A `<gpio>` naming no plugin
 * has no built-in fallback and is skipped with an error, since no built-in
 * GPIO handling exists.
 */
constexpr const char *kPluginParam = "plugin";

/**
 * @brief A `<joint>`'s position/velocity/effort/acceleration limits, as resolved
 *        by MujocoSystem from the URDF.
 *
 * Passed into registerComponent() so a `<joint>` plugin can clamp its commands
 * and outputs the same way MujocoSystem's own built-in joint logic does,
 * without re-deriving them from the URDF itself. Meaningless (left at its
 * unbounded default) for a `<sensor>`/`<gpio>`.
 */
struct JointLimits {
    double lower = -std::numeric_limits<double>::infinity();
    double upper = std::numeric_limits<double>::infinity();
    double velocity = std::numeric_limits<double>::infinity();
    double effort = std::numeric_limits<double>::infinity();
    double acceleration = std::numeric_limits<double>::infinity();
};

/**
 * @brief Base class for `<sensor>`/`<gpio>`/`<joint>` handlers loaded through pluginlib.
 *
 * Not to be confused with MujocoRos2PluginInterface: that one is for plugins
 * declared outside `<ros2_control>` entirely (cameras, lidars, services, ...),
 * driven on their own thread or their own step hook. This one binds to one
 * `<ros2_control>` component -- a `<sensor>`, `<gpio>` or `<joint>` -- and is
 * driven by the hardware component's own read()/write() cycle.
 *
 * A component names its plugin through the @ref kPluginParam parameter. It has
 * to be a `<param>` rather than an attribute.
 *
 * @code{.xml}
 * <sensor name="fingertip_touch">
 *   <param name="plugin">mujoco_ros2_control_examples/TouchGridSensor</param>
 *   <param name="site">touch_site</param>
 * </sensor>
 * @endcode
 *
 * Two capabilities motivate a plugin over the built-in IMU/ForceTorque/Pose
 * sensor classes or (for a `<joint>`) the built-in joint logic:
 *
 * - Types those cannot express: anything beyond an IMU, a force/torque or a
 *   pose sensor needs a plugin of its own, as does any `<gpio>` (which has no
 *   built-in handling at all) or a `<joint>` whose actuation the built-in
 *   PID/actuator logic cannot express.
 * - Outputs that do not fit ros2_control's scalar StateInterface model. A MuJoCo
 *   `touch_grid` yields `nchannel * width * height` values per step, which is far
 *   more natural on a topic than as several hundred interfaces. Hence the node
 *   handed to registerComponent() and the activate()/deactivate() hooks.
 *
 * @par Lifetime
 * Instances are owned by MujocoRos2ControlPluginLoader for as long as the hardware
 * component lives, and the class loader that created them outlives the
 * instances. Any `double` registered as a StateInterface or CommandInterface
 * must live in the plugin object itself, never in a container that can
 * reallocate.
 */
class MujocoRos2ControlPluginInterface {
public:
    virtual ~MujocoRos2ControlPluginInterface() = default;

    /**
     * @brief Bind the plugin to one `<sensor>`/`<gpio>`/`<joint>` element, read
     *        its parameters and export its state/command interfaces.
     *
     * Called once while the hardware component initialises. Resolve MuJoCo
     * addresses from @p mujoco_model and append any scalar outputs to
     * @p state_interfaces and any scalar inputs to @p command_interfaces,
     * pointing at storage owned by this object. A read-only component (a
     * sensor) simply never appends to @p command_interfaces.
     *
     * @p node is the simulation node, already spinning, so declaring parameters
     * and creating publishers is allowed here. It is the node running the
     * simulation rather than the hardware component's own: get_node() on the
     * component returns nullptr until long after this point.
     *
     * @param node               The simulation node; outlives this object.
     * @param mujoco_model       The compiled model; addresses stay valid for its lifetime.
     * @param component_info     The `<sensor>`/`<gpio>`/`<joint>` element, including its `<param>` entries.
     * @param joint_limits       The `<joint>`'s URDF-derived limits; unbounded/meaningless for a `<sensor>`/`<gpio>`.
     * @param state_interfaces   Appended to; may be left untouched by a topic-only component.
     * @param command_interfaces Appended to; left untouched by a read-only component.
     * @return False to abort loading, after logging what was wrong with the declaration.
     */
    virtual bool registerComponent(
            const rclcpp::Node::SharedPtr &node,
            const mjModel *mujoco_model,
            const hardware_interface::ComponentInfo &component_info,
            const JointLimits &joint_limits,
            std::vector<hardware_interface::StateInterface> &state_interfaces,
            std::vector<hardware_interface::CommandInterface> &command_interfaces) = 0;

    /**
     * @brief Command interface names this instance claims for a `<joint>`.
     *
     * MujocoSystem's built-in joint logic skips creating its own interface for
     * any name here, so this instance's registerComponent()-exported one is the
     * only one -- every OTHER interface on the same `<joint>` (a different
     * control method with a matching actuator, mimic wiring, effort passthrough)
     * is untouched and keeps working through the built-in path. Always empty for
     * a `<sensor>`/`<gpio>`, which own everything they declare outright.
     */
    virtual std::set<std::string> claimed_command_interfaces() const { return {}; }

    /** @brief Same as claimed_command_interfaces(), for state interfaces. */
    virtual std::set<std::string> claimed_state_interfaces() const { return {}; }

    /**
     * @brief Copy this step's values out of `mjData` (e.g. `sensordata`, `qpos`).
     *
     * Called every read() cycle of the hardware component, in the control loop.
     * Keep it allocation-free. Publishing here is allowed but must never block:
     * use realtime_tools::RealtimePublisher rather than a plain publisher.
     */
    virtual void read(const mjData *mujoco_data) = 0;

    /**
     * @brief Apply this step's commands into `mjData` (e.g. `ctrl`, `qfrc_applied`).
     *
     * Called every write() cycle of the hardware component, with that cycle's
     * control period in @p dt (e.g. for a discrete-time integral/derivative
     * term) -- the MuJoCo timestep can differ from the control loop's rate, so
     * this is the same period MujocoSystem::write() itself was called with, not
     * `mujoco_model->opt.timestep`. Default no-op: a read-only component (a
     * sensor) never overrides this.
     */
    virtual void write(mjData *mujoco_data, double dt) { (void)mujoco_data; (void)dt; }

    /**
     * @brief React to a ros2_control command-mode switch.
     *
     * Mirrors MujocoSystem::perform_command_mode_switch(), forwarded to this
     * instance for the interfaces it owns. Default no-op: only a component
     * exporting switchable command interfaces (e.g. position vs. effort) needs
     * to override this.
     *
     * @return False to reject the switch, same as MujocoSystem's own.
     */
    virtual bool perform_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces) {
        (void)start_interfaces;
        (void)stop_interfaces;
        return true;
    }

    /** @brief Start publishing. Called from the component's on_activate(). */
    virtual void activate() {}

    /** @brief Stop publishing. Called from the component's on_deactivate(). */
    virtual void deactivate() {}

    /** @brief The element name this instance was bound to; set by the loader. */
    const std::string &name() const { return name_; }

    /** @brief Set by MujocoRos2ControlPluginLoader immediately after construction. */
    void set_name(const std::string &name) { name_ = name; }

protected:
    MujocoRos2ControlPluginInterface() = default;

    std::string name_;
};

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_PLUGIN_INTERFACE_HPP_
