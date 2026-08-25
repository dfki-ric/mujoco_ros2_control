/**
 * @file touch_grid_sensor.hpp
 * @brief Sensor plugin publishing MuJoCo's touch_grid tactile array to a topic.
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

#ifndef MUJOCO_ROS2_CONTROL_EXAMPLES__TOUCH_GRID_SENSOR_HPP_
#define MUJOCO_ROS2_CONTROL_EXAMPLES__TOUCH_GRID_SENSOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "mujoco_ros2_control/mujoco_ros2_plugin_interface.hpp"

namespace mujoco_ros2_control_examples {

/**
 * @brief Publishes a `mujoco.sensor.touch_grid` array as Float64MultiArray.
 *
 * A touch_grid produces `nchannel * width * height` values per step, which does
 * not fit ros2_control's scalar StateInterface model. It therefore exports
 * nothing to ros2_control and is declared as a top-level `<mujoco_ros2_plugin>`
 * rather than as a `<sensor>` inside a `<ros2_control>` block: it gets its own
 * node and its own thread instead of being read by a hardware component in the
 * control loop. Channels are, in order, the force
 * components [normal, tangent, tangent] followed by the torque components
 * [torsional, rolling, rolling], truncated to the model's `nchannel`.
 *
 * The MJCF has to declare the matching engine-plugin sensor. Note that the
 * site's **negative** z axis is the sensing direction, so a downward-facing pad
 * wants the site left at its default orientation rather than flipped:
 *
 * @code{.xml}
 * <sensor>
 *   <plugin name="fingertip_touch" plugin="mujoco.sensor.touch_grid"
 *           objtype="site" objname="touch_site">
 *     <config key="nchannel" value="3"/>
 *     <config key="size"     value="7 7"/>
 *     <config key="fov"      value="45 45"/>
 *     <config key="gamma"    value="0"/>
 *   </plugin>
 * </sensor>
 * @endcode
 *
 * Everything is configured from `<param>` entries on the declaration:
 *
 * - `site`     : the site the touch_grid is attached to (default: the sensor name)
 * - `topic`    : topic to publish on (default: `<sensor name>/touch_grid`)
 * - `frame_id` : frame the taxel values are expressed in (default: the site name)
 * - `publish`  : set "false" to step the sensor without producing topic traffic
 * - `rate`     : sampling rate in simulated Hz (default: the base class's 100)
 *
 * @code{.xml}
 * <mujoco_ros2_plugin name="fingertip_touch"
 *                     plugin="mujoco_ros2_control_examples/TouchGridSensor">
 *   <param name="site">touch_site</param>
 * </mujoco_ros2_plugin>
 * @endcode
 */
class TouchGridSensor : public mujoco_ros2_control::MujocoRos2PluginInterface {
public:
    bool configure(
            const Context &context,
            const mujoco_ros2_control::MujocoRos2PluginInfo &info) override;

    void update(const mjData *mujoco_data, const rclcpp::Time &stamp) override;

private:
    /// Reads a single integer from the engine plugin's config block.
    bool read_int_config(
            const mjModel *mujoco_model, int plugin_instance,
            const char *key, int minimum, int *out);

    /// Reads the "size" config, which holds width and height as one string.
    bool read_size_config(const mjModel *mujoco_model, int plugin_instance);

    std::string site_name_;
    std::string topic_;
    std::string frame_id_;

    int sensor_adr_{-1};
    int sensor_dim_{0};
    int nchannel_{0};
    int width_{0};
    int height_{0};

    std::vector<double> data_;
    std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
        realtime_publisher_;
    bool publish_{true};

    /// Replaced with this sensor's own node logger in configure().
    rclcpp::Logger logger_ = rclcpp::get_logger("touch_grid_sensor");
};

}  // namespace mujoco_ros2_control_examples

#endif  // MUJOCO_ROS2_CONTROL_EXAMPLES__TOUCH_GRID_SENSOR_HPP_
