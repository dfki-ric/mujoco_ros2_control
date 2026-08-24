/**
 * @file mujoco_sensor_interface.hpp
 * @brief Base class for pluginlib-loaded MuJoCo sensor handlers.
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

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_SENSOR_INTERFACE_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_SENSOR_INTERFACE_HPP_

#include <string>
#include <vector>

#include "mujoco/mujoco.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "rclcpp/rclcpp.hpp"

namespace mujoco_ros2_control {

/**
 * @brief Base class for sensor handlers loaded through pluginlib.
 *
 * This is the extensible counterpart to MujocoSensors, which classifies IMU,
 * force/torque and pose sensors by inspecting their state interface names. That
 * built-in path is unchanged and still runs for every `<sensor>` that does not
 * name a plugin; this one is additive.
 *
 * A `<sensor>` opts in through a `plugin` parameter. This has to be a `<param>`:
 * ros2_control's URDF parser silently discards unknown attributes on the
 * `<sensor>` element, and ComponentInfo::type is always "sensor".
 *
 * @code{.xml}
 * <sensor name="fingertip_touch">
 *   <param name="plugin">mujoco_ros2_control/TouchGridSensor</param>
 *   <param name="site">touch_site</param>
 * </sensor>
 * @endcode
 *
 * Two capabilities motivate this over the built-in classifier:
 *
 * - Sensor types the classifier cannot express. Its dispatch is a substring
 *   match on interface names, so anything that is not an IMU, a force/torque or
 *   a pose sensor cannot be added without editing this package.
 * - Outputs that do not fit ros2_control's scalar StateInterface model. A MuJoCo
 *   `touch_grid` yields `nchannel * width * height` values per step, which is far
 *   more natural on a topic than as several hundred interfaces. Hence the node
 *   handed to registerSensor() and the activate()/deactivate() hooks.
 *
 * @par Lifetime
 * Instances are owned by MujocoSensorPlugins for as long as the hardware
 * component lives, and the class loader that created them outlives the
 * instances. Any `double` registered as a StateInterface must live in the plugin
 * object itself, never in a container that can reallocate.
 */
class MujocoSensorInterface {
public:
    virtual ~MujocoSensorInterface() = default;

    /**
     * @brief Bind the plugin to one `<sensor>` element, read its parameters and
     *        export its state interfaces.
     *
     * Called once while the hardware component initialises. Resolve MuJoCo
     * sensor addresses from @p mujoco_model and append any scalar outputs to
     * @p state_interfaces, pointing at storage owned by this object.
     *
     * @p node is the simulation node, already spinning, so declaring parameters
     * and creating publishers is allowed here. It is the node running the
     * simulation rather than the hardware component's own: get_node() on the
     * component returns nullptr until long after this point.
     *
     * @param node             The simulation node; outlives this object.
     * @param mujoco_model     The compiled model; sensor addresses stay valid for its lifetime.
     * @param sensor_info      The `<sensor>` element, including its `<param>` entries.
     * @param state_interfaces Appended to; may be left untouched by a topic-only sensor.
     * @return False to abort loading, after logging what was wrong with the declaration.
     */
    virtual bool registerSensor(
            const rclcpp::Node::SharedPtr &node,
            const mjModel *mujoco_model,
            const hardware_interface::ComponentInfo &sensor_info,
            std::vector<hardware_interface::StateInterface> &state_interfaces) = 0;

    /**
     * @brief Copy this step's values out of `mjData::sensordata`.
     *
     * Called every read() cycle of the hardware component, in the control loop.
     * Keep it allocation-free. Publishing here is allowed but must never block:
     * use realtime_tools::RealtimePublisher rather than a plain publisher.
     */
    virtual void read(const mjData *mujoco_data) = 0;

    /** @brief Start publishing. Called from the component's on_activate(). */
    virtual void activate() {}

    /** @brief Stop publishing. Called from the component's on_deactivate(). */
    virtual void deactivate() {}

    /** @brief The `<sensor>` name this instance was bound to; set by the loader. */
    const std::string &name() const { return name_; }

    /** @brief Set by MujocoSensorPlugins immediately after construction. */
    void set_name(const std::string &name) { name_ = name; }

protected:
    MujocoSensorInterface() = default;

    std::string name_;
};

}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_SENSOR_INTERFACE_HPP_
